#!/usr/bin/env python3  
#coding: utf-8

import os  # 用于处理音频文件路径
import subprocess  # 用于调用 mplayer 播放音频
import time  # 用于计算 VLM 超时时间
import struct  # 用于打包虚拟墙点云数据
from math import pi  # 用于角度转弧度
from math import cos, sin, atan2  # 用于计算封路位置 / 卡死检测的转角差

import actionlib  # ROS Action 客户端
import rospy  # ROS Python 接口
import tf  # 查询机器人当前位姿(封路用)
from actionlib_msgs.msg import GoalStatus  # move_base导航状态码
from geometry_msgs.msg import Twist  # 速度消息
from geometry_msgs.msg import PoseWithCovarianceStamped  # 初始位姿消息
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # move_base 导航消息
from map_msgs.msg import OccupancyGridUpdate  # 代价地图的局部更新(目标点吸附用)
from nav_msgs.msg import OccupancyGrid  # 全局代价地图(目标点吸附用)
from nav_msgs.srv import GetPlan, GetPlanRequest  # 问规划器"这个点规划得到吗"(吸附的触发判据)
from sensor_msgs.msg import PointCloud2, PointField  # 虚拟墙(封路)点云
from tf_conversions import transformations  # 欧拉角转四元数


# ========================= 比赛结果配置 =========================

VALID_TASK_IDS = (31, 32, 33, 40, 41, 42, 49, 50, 51)  # 允许出现的9个任务编号

VLM_INVALID_RESULT = -0xFFFF  # VLM可能返回的无效结果

VLM_DEFAULT_RESULT = 65535  # 表示当前还没有新的识别结果


# ========================= 音频配置 =========================

AUDIO_DIR = '/home/abot/yuyin'  # 音频目录

START_AUDIO = 'bsks.mp3'  # 比赛开始

ANSWER_AUDIO_TEMPLATE = '{}{}.mp3'  # 识别后播报：第1张识别42 -> 142.mp3

TASK_ARRIVAL_AUDIO_TEMPLATE = '{}.mp3'  # 到达任务点播报：去42 -> 42.mp3

ARRIVE_FINISH_AUDIO = 'end.mp3'  # 到达9号终点

COMPETITION_END_AUDIO = 'bsjs.mp3'  # 比赛结束

# ========================= VLM配置 =========================

VLM_TIMEOUT = 15.0  # 单次VLM最多等待15秒

VLM_RETRY = 2  # VLM识别失败最多重试2次

# ========================= 方案B: 虚拟墙(封路换路)配置 =========================

VW_TOPIC = '/virtual_wall'  # 封路点云话题; 必须与 costmap_common_params.yaml 里的观测源一致

VW_AHEAD_DIST = 0.30  # 在车头前方多少米处封路(米)

VW_GRID_STEP = 0.15  # 封路点阵间距(米); 3x3 点阵 ≈ 0.3m 见方

# 封路中心离目标多近就不封(米)。推导:
#   封路块半径 0.15×√2 ≈ 0.21 + 目标容差 0.08 + 目标可达门槛 0.06 ≈ 0.35
#   取 0.45 留 0.10m 余量。
# 低于这个值还封 = 把目标点本身围死 → 规划器报 Failed to get a plan, 重试必然失败,
# 所以这时直接放弃、连重试都不做(省下 25 秒必败挣扎)。
SEAL_MIN_GOAL_DIST = 0.37

NAV_TIMEOUT = 25  # 单段导航超时(秒); 走不通就封路换路再给一次


# ========================= 窄道卡死自救(前拱)配置 =========================
# 场景: 终点这一腿用的是 goto_no_timeout, move_base 只要不 abort, 主控就会一直等。
#       TEB 在窄道中间偶尔会判"轨迹不可行"然后原地不动, 车其实再往前挪十几厘米
#       就能重新规划过去。这里做的是: 连续几秒既没位移也没转角 → 取消目标 →
#       低速往前拱一小段 → 重新发同一个目标, 让 TEB 从新的位置重新规划。

STUCK_WINDOW = 4.0          # 秒: 连续这么久"没位移也没转角"才判卡死(至少要攒满 3 秒样本)
STUCK_MIN_MOVE = 0.05       # 米: 时间窗内的净位移小于它算"没动"(AMCL 抖动一般 1~3cm)
STUCK_MIN_TURN = 0.10       # 弧度: 时间窗内转角小于它算"没转"(原地调头/恢复旋转不算卡死)

NUDGE_SPEED = 0.12          # m/s: 前拱速度。窄道里别快, 快了容易蹭墙
NUDGE_DIST = 0.18           # 米: 一次前拱的目标距离(大约一个车身)
NUDGE_MAX_TIME = 3.0        # 秒: 一次前拱最长持续多久(被挡住会提前退出)
NUDGE_STALL_TIME = 1.0      # 秒: 拱了这么久还几乎没动就立刻停, 别让轮子干磨
NUDGE_MIN_PROGRESS = 0.03   # 米: 至少挪了这么多才算"前拱有效", 才值得重发目标
NUDGE_MIN_CLEAR = 0.05      # 米: 前方采样点的最小净空; 低于它说明真堵死了, 不拱

FINISH_MAX_NUDGES = 4       # 终点这一腿最多前拱几次(4 × 18cm ≈ 72cm 的脱困预算)


# ========================= 目标点吸附配置 =========================

SNAP_RADIUS = 0.15  # 搜索半径(米)。40cm 格距下挡板都在离任务点 0.19m 以外, 所以这圈跨不过挡板, 不会吸到隔壁任务点

SNAP_STEP = 0.02  # 搜索步长(米) = 代价地图分辨率

SNAP_MIN_CLEAR = 0.17  # 合格净空(米) = 车半宽 0.14 + TEB 的 min_obstacle_dist 0.03
                       # ⚠️ 这个净空是"到真障碍(致死格100)"的距离, 不是到膨胀带(99)的距离

SNAP_FLOOR_CLEAR = 0.14  # 兜底净空(米) = 车半宽; 低于这个值车物理上根本停不下


# ========================= 导航类 =========================

class navigation_demo:  # 官方导航类

    def __init__(self):  # 初始化导航对象
        self.set_pose_pub = rospy.Publisher(  # 创建初始位姿发布器
            '/initialpose',  # 初始位姿话题
            PoseWithCovarianceStamped,  # 初始位姿消息类型
            queue_size=5  # 消息队列长度
        )

        self.move_base = actionlib.SimpleActionClient(  # 创建move_base Action客户端
            "move_base",  # Action名称
            MoveBaseAction  # Action消息类型
        )

        rospy.loginfo("等待 move_base action server...")  # 输出等待提示

        self.move_base.wait_for_server(  # 等待move_base服务
            rospy.Duration(60)  # 保持官方60秒等待机制
        )

    def goto(self, pos):  # 导航到指定位置
        goal = MoveBaseGoal()  # 创建导航目标
        goal.target_pose.header.frame_id = 'map'  # 使用map坐标系
        goal.target_pose.header.stamp = rospy.Time.now()  # 设置目标时间戳

        goal.target_pose.pose.position.x = pos[0]  # 设置目标X坐标
        goal.target_pose.pose.position.y = pos[1]  # 设置目标Y坐标

        trans_pos = transformations.quaternion_from_euler(  # 将欧拉角转换成四元数
            0,  # Roll为0
            0,  # Pitch为0
            pos[2] / 180 * pi  # 将Yaw从角度转换成弧度
        )

        goal.target_pose.pose.orientation.x = trans_pos[0]  # 设置四元数X
        goal.target_pose.pose.orientation.y = trans_pos[1]  # 设置四元数Y
        goal.target_pose.pose.orientation.z = trans_pos[2]  # 设置四元数Z
        goal.target_pose.pose.orientation.w = trans_pos[3]  # 设置四元数W

        rospy.loginfo(  # 输出导航目标
            "发送导航目标: x=%.3f y=%.3f yaw=%.1f",
            pos[0],
            pos[1],
            pos[2]
        )

        self.move_base.send_goal(  # 发送导航目标
            goal,  # 导航目标
            self._done_cb,  # 导航完成回调
            self._active_cb,  # 导航开始回调
            self._feedback_cb  # 导航反馈回调
        )

        result = self.move_base.wait_for_result(  # 等待导航结果
            rospy.Duration(60)  # 保持官方60秒等待机制
        )

        if not result:  # 判断是否超时
            self.move_base.cancel_goal()  # 超时后取消导航
            rospy.loginfo("Timed out achieving goal")  # 输出超时信息

        else:  # 收到导航结果
            state = self.move_base.get_state()  # 获取导航状态
            rospy.loginfo(  # 输出导航状态
                "move_base state: %d",
                state
            )

        return True  # 保持官方原来的返回方式

    def goto_with_timeout(self, pos, timeout=30):  # 使用指定秒数限制一次导航
        goal = MoveBaseGoal()  # 创建导航目标
        goal.target_pose.header.frame_id = 'map'  # 使用map坐标系
        goal.target_pose.header.stamp = rospy.Time.now()  # 设置目标时间戳
        goal.target_pose.pose.position.x = pos[0]  # 设置目标X坐标
        goal.target_pose.pose.position.y = pos[1]  # 设置目标Y坐标
        trans_pos = transformations.quaternion_from_euler(  # 将欧拉角转换成四元数
            0,  # Roll为0
            0,  # Pitch为0
            pos[2] / 180 * pi  # 将Yaw从角度转换成弧度
        )
        goal.target_pose.pose.orientation.x = trans_pos[0]  # 设置四元数X
        goal.target_pose.pose.orientation.y = trans_pos[1]  # 设置四元数Y
        goal.target_pose.pose.orientation.z = trans_pos[2]  # 设置四元数Z
        goal.target_pose.pose.orientation.w = trans_pos[3]  # 设置四元数W
        rospy.loginfo(  # 输出任务点导航目标
            "发送任务点导航目标: x=%.3f y=%.3f yaw=%.1f，限时%d秒",
            pos[0],
            pos[1],
            pos[2],
            timeout
        )
        self.move_base.send_goal(  # 发送导航目标
            goal,  # 导航目标
            self._done_cb,  # 导航完成回调
            self._active_cb,  # 导航激活回调
            self._feedback_cb  # 导航反馈回调
        )
        result = self.move_base.wait_for_result(  # 等待任务点导航结果
            rospy.Duration(timeout)  # 使用指定的任务点时间限制
        )
        if not result:  # 到达时间限制仍未完成
            self.move_base.cancel_goal()  # 取消当前任务点导航
            rospy.logwarn(  # 输出任务点超时信息
                "任务点导航超过%d秒，取消目标并继续后续流程",
                timeout
            )
            return False  # 返回导航失败
        state = self.move_base.get_state()  # 获取最终导航状态
        rospy.loginfo(  # 输出任务点最终状态
            "任务点导航结束，state=%d",
            state
        )
        if state == GoalStatus.SUCCEEDED:  # 判断是否真正到达目标
            return True  # 返回导航成功
        rospy.logwarn(  # 导航在时间限制内结束但未成功
            "任务点导航未成功，state=%d，继续后续任务",
            state
        )
        return False  # 返回导航失败

    def goto_no_timeout(self, pos, stuck_detector=None,  # 不设超时地导航: 一直等 move_base 自己给结果
                        on_stuck=None, max_recoveries=0):
        """和 goto_with_timeout 一样, 但不设超时。只给9号终点这种"必须走到"的点用:
           走不到就是丢分, 所以让它一直走, 主控中途不掐断。
           实现: 每次等 1 秒看有没有结果, 直到拿到结果或 goal 已进入终态。

           卡死自救(可选): 传入 stuck_detector=返回 True 表示"卡住了", on_stuck=卡住时
           执行的脱困动作(返回 True 表示脱困成功)。触发后取消当前目标, 等 move_base
           松开 cmd_vel, 执行 on_stuck(), 然后把同一个目标重新发一遍。
           max_recoveries=0 时行为和原来完全一样。"""
        goal = MoveBaseGoal()  # 创建导航目标
        goal.target_pose.header.frame_id = 'map'  # 使用map坐标系
        goal.target_pose.header.stamp = rospy.Time.now()  # 设置目标时间戳
        goal.target_pose.pose.position.x = pos[0]  # 目标X
        goal.target_pose.pose.position.y = pos[1]  # 目标Y
        trans_pos = transformations.quaternion_from_euler(  # 将欧拉角转换成四元数
            0,  # Roll为0
            0,  # Pitch为0
            pos[2] / 180 * pi  # 将Yaw从角度转换成弧度
        )
        goal.target_pose.pose.orientation.x = trans_pos[0]  # 设置四元数X
        goal.target_pose.pose.orientation.y = trans_pos[1]  # 设置四元数Y
        goal.target_pose.pose.orientation.z = trans_pos[2]  # 设置四元数Z
        goal.target_pose.pose.orientation.w = trans_pos[3]  # 设置四元数W
        rospy.loginfo(  # 输出目标
            "发送导航目标(不设超时): x=%.3f y=%.3f yaw=%.1f",
            pos[0],
            pos[1],
            pos[2]
        )
        self.move_base.send_goal(  # 发送导航目标
            goal,
            self._done_cb,
            self._active_cb,
            self._feedback_cb
        )
        done_states = (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,  # 终态集合
                       GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                       GoalStatus.RECALLED, GoalStatus.LOST)
        recoveries = 0  # 已经做过几次卡死自救
        while not rospy.is_shutdown():  # 一直等到有结果
            if self.move_base.wait_for_result(rospy.Duration(1.0)):  # 每次等1秒
                break  # 拿到结果了
            if self.move_base.get_state() in done_states:  # goal 已终态
                break  # 不用再等了

            # ---------- 卡死自救: 没结果、goal 还活着, 但车连续几秒没动 ----------
            if (max_recoveries > 0 and recoveries < max_recoveries  # 还有自救预算
                    and stuck_detector is not None and on_stuck is not None
                    and stuck_detector()):  # 判定卡死(每次等待1秒, 正好是一个采样点)
                recoveries += 1  # 消耗一次预算
                rospy.logwarn(  # 输出自救提示
                    "[卡死自救] 连续%.0f秒没有位移, 取消目标, 前拱后重发(%d/%d)",
                    STUCK_WINDOW, recoveries, max_recoveries
                )
                self.move_base.cancel_goal()  # ★ 必须先取消: 否则 move_base 的 cmd_vel 会和前拱抢

                # 等 move_base 真正松开这个目标(状态离开 PENDING/ACTIVE), 最多等 2 秒
                t_cancel = time.time()
                while time.time() - t_cancel < 2.0:
                    if self.move_base.get_state() not in (GoalStatus.PENDING,
                                                          GoalStatus.ACTIVE):
                        break
                    rospy.sleep(0.1)
                self.move_base.wait_for_result(rospy.Duration(0.5))  # 把 PREEMPTED 结果取掉
                rospy.sleep(0.2)  # 再留一点时间让底盘把残速执行完

                if not on_stuck():  # 前拱失败(前方真堵死 / 轮子空转)
                    rospy.logwarn("[卡死自救] 前拱没成功, 放弃本段")
                    break  # 交给调用方的兜底逻辑

                goal.target_pose.header.stamp = rospy.Time.now()  # 刷新时间戳
                self.move_base.send_goal(  # 脱困成功 → 在当前位置重新发同一个目标
                    goal, self._done_cb, self._active_cb, self._feedback_cb
                )
                continue  # 回到等待循环, 让 TEB 从新位置重新规划

        state = self.move_base.get_state()  # 取最终状态
        rospy.loginfo("不设超时的导航结束，state=%d", state)  # 输出状态
        return state == GoalStatus.SUCCEEDED  # 返回是否真正到达

    def cancel(self):  # 取消导航
        self.move_base.cancel_all_goals()  # 取消所有导航目标
        return True  # 返回成功

    def _done_cb(self, status, result):  # 导航完成回调
        rospy.loginfo(  # 输出导航完成信息
            "navigation done! status:%d result:%s",
            status,
            result
        )

    def _active_cb(self):  # 导航激活回调
        rospy.loginfo("[Navi] navigation has be actived")  # 输出导航激活信息

    def _feedback_cb(self, feedback):  # 导航反馈回调
        pass  # 当前不处理导航反馈


# ========================= 比赛主控制器 =========================

class CompetitionController:  # 比赛主控制器
    def __init__(self, navi):  # 初始化比赛控制器
        self.navi = navi  # 保存导航对象
        # ========================= 比赛点位 =========================
        self.start_point = (  # 起点坐标
            0.55,  # 起点X
            -0.15,  # 起点Y
            90.0  # 起点Yaw
        )
        self.image_points = [  # 四个任务图像识别点
            (  # 第1个任务图像识别点
                1.546, -0.287,  84.26
            ),
            (  # 第2个任务图像识别点
                2.925, -1.540,  -0.01
            ),
            (  # 第3个任务图像识别点
                1.557, -2.845, -86.41
            ),
            (  # 第4个任务图像识别点
                0.364, -1.541, 179.12
            )
        ]  # 四个识别点配置结束
        self.finish_point = (  # 9号终点坐标
            3.203, -0.033, -3.5
        )

        self.task_points = {  # 九个任务点坐标
            31: (  # 任务点31
                1.172, -1.151, -0.7
            ),
            32: (  # 任务点32
                1.568, -1.180, 0.6
            ),
            33: (  # 任务点33
                2.000, -1.179, 1.6
            ),
            40: (  # 任务点40
                1.169, -1.543, -0.3
            ),
            41: (  # 任务点41
                1.550, -1.542, 0.0
            ),
            42: (  # 任务点42
                1.974, -1.557, -0.2
            ),
            49: (  # 任务点49
                1.160, -1.955, 0.7
            ),
            50: (  # 任务点50
                1.572, -1.948, -0.7
            ),
            51: (  # 任务点51
                1.938, -1.925, -2.0      
            )
        }  # 九个任务点配置结束
        rospy.loginfo("比赛点位已经加载")  # 输出点位加载信息

        for i, point in enumerate(  # 遍历4个识别点
            self.image_points,
            start=1
        ):
            rospy.loginfo(  # 输出识别点坐标
                "图像点%d = %s",
                i,
                point
            )

        for task_id in VALID_TASK_IDS:  # 遍历9个任务编号
            rospy.loginfo(  # 输出任务点坐标
                "任务点%d = %s",
                task_id,
                self.task_points[task_id]
            )

        rospy.loginfo(  # 输出终点坐标
            "9号终点 = %s",
            self.finish_point
        )

        self.cmd_vel_pub = rospy.Publisher(  # 创建底盘速度发布器
            '/cmd_vel',  # 速度话题
            Twist,  # 速度消息类型
            queue_size=10  # 消息队列长度
        )

        self.audio_dir = AUDIO_DIR  # 保存音频目录

        self.start_audio = START_AUDIO  # 保存比赛开始音频

        self.answer_audio_template = ANSWER_AUDIO_TEMPLATE  # 保存识别结果音频模板

        self.arrive_finish_audio = ARRIVE_FINISH_AUDIO  # 保存到达终点音频

        self.competition_end_audio = COMPETITION_END_AUDIO  # 保存比赛结束音频

        self.task_arrival_audio_template = TASK_ARRIVAL_AUDIO_TEMPLATE  # 保存任务点音频模板

        self.vlm_timeout = VLM_TIMEOUT  # 保存VLM超时时间

        self.vlm_retry = VLM_RETRY  # 保存VLM重试次数

        # ---------- 方案B: 虚拟墙(走不通就封路, 逼规划器换另一条路) ----------

        self.vw_pub = rospy.Publisher(  # 创建虚拟墙点云发布器
            VW_TOPIC,  # 话题
            PointCloud2,  # 消息类型
            queue_size=1  # 队列长度
        )

        self.tf_listener = tf.TransformListener()  # 用于查询 map→base_link 求机器人位姿

        self.seals = []  # 已经封掉的位置列表 [(x, y), ...]

        # ---------- 窄道卡死自救(前拱) ----------
        self._stuck_hist = []  # 卡死检测用的位姿采样窗口 [(time, x, y, yaw), ...]

        # ---------- 目标点吸附(导航失败后换落点, 只用代价地图, 不依赖任何 move_base 服务) ----------

        self.cmap = None  # 本地维护的全局代价地图(每格一个值: 0~98可通行, 99内切膨胀, 100致死, -1未知)
        self.cmap_info = None  # (分辨率, origin_x, origin_y, 宽, 高)

        rospy.Subscriber(  # 整张代价地图(发布端有"新订阅者"回调, 一订阅就会推一张全图)
            '/move_base/global_costmap/costmap',
            OccupancyGrid,
            self._costmap_cb
        )
        rospy.Subscriber(  # 之后的局部更新
            '/move_base/global_costmap/costmap_updates',
            OccupancyGridUpdate,
            self._costmap_update_cb
        )
        rospy.loginfo("[吸附] 已订阅全局代价地图")

        # ---------- 吸附的触发判据: 问规划器"原目标点规划得到吗" ----------
        # ⚠️ move_base.yaml 里这两行是这个判据的前提, 不能删:
        #    make_plan_clear_costmap: false   ← 否则服务会先清掉机器人周围0.52m的地图,
        #                                       把被盖住的目标点"洗干净", 从而谎报可达
        #    make_plan_add_unreachable_goal: false ← 否则路径末尾会追加原目标点, 分不清
        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=5.0)
            self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            self.snap_ready = True
            rospy.loginfo("[吸附] make_plan 服务已就绪(用它判断目标点规划得到吗)")
        except Exception as exc:  # 拿不到就退回"只按净空判断"
            self.make_plan = None
            self.snap_ready = False
            rospy.logwarn("[吸附] 拿不到 make_plan 服务, 退回只按净空判断: %s", exc)

    # ========================= 机器人停止 =========================

    def stop_robot(self):  # 停止机器人
        zero = Twist()  # 创建全零速度消息
        for _ in range(5):  # 连续发送5次
            self.cmd_vel_pub.publish(zero)  # 发布零速度
            rospy.sleep(0.05)  # 等待50毫秒

    # ========================= 窄道卡死自救(前拱) =========================

    def _robot_pose(self):  # 取当前 map 位姿 (x, y, yaw); 取不到返回 None
        try:
            trans, rot = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0)
            )
        except Exception:  # TF 暂时断了, 这一拍当"测不到", 不影响主流程
            return None
        _, _, yaw = transformations.euler_from_quaternion(rot)  # 转成朝向角
        return trans[0], trans[1], yaw

    def _stuck_detector(self):  # 连续 STUCK_WINDOW 秒没位移也没转角 → 返回 True
        """由 goto_no_timeout 每约 1 秒调用一次。用"窗口首尾的净位移"判断,
           而不是路径长度: TEB 在窄道里来回蹭(前进又后退)时净位移≈0, 也应该救。
           原地调头不算卡死(转角超阈值), 因为那是 TEB/恢复行为的正常动作。"""
        pose = self._robot_pose()  # 当前位姿
        if pose is None:  # 测不到就不判, 宁可漏判也不要误判
            return False

        now = time.time()  # 用系统时间, 和 rospy 的仿真时间无关
        self._stuck_hist.append((now, pose[0], pose[1], pose[2]))  # 记录这一拍

        while self._stuck_hist and now - self._stuck_hist[0][0] > STUCK_WINDOW:
            self._stuck_hist.pop(0)  # 丢掉窗口外的旧样本

        if len(self._stuck_hist) < 2:  # 样本太少, 还判断不了
            return False

        t0, x0, y0, yaw0 = self._stuck_hist[0]  # 窗口起点
        if now - t0 < STUCK_WINDOW - 1.0:  # 窗口还没攒满(要至少 3 秒的样本)
            return False

        dx = pose[0] - x0  # X 净位移
        dy = pose[1] - y0  # Y 净位移
        moved = (dx * dx + dy * dy) ** 0.5  # 净位移长度
        d_yaw = abs(atan2(sin(pose[2] - yaw0), cos(pose[2] - yaw0)))  # 转角差(归一到 ±π)

        if moved >= STUCK_MIN_MOVE or d_yaw >= STUCK_MIN_TURN:  # 动过或转过 → 没卡死
            return False

        self._stuck_hist = []  # 判定卡死, 清空窗口, 下一次重新攒
        return True

    def _forward_clear(self, pose):  # 前拱前检查车头前方是否真的有地方走
        """用全局代价地图(GPS 一样的新鲜激光数据)沿车头方向每 5cm 采一个点,
           任何一点净空 < NUDGE_MIN_CLEAR 就拒绝前拱, 防止往墙上硬顶。
           代价地图还没收到时不拦(否则这段救急逻辑在最需要的时候用不上)。"""
        if self.cmap is None:  # 还没收到全局代价地图
            rospy.logwarn("[前拱] 还没收到全局代价地图, 无法预检, 仍然尝试前拱")
            return True

        x, y, yaw = pose  # 当前位姿
        d = 0.05  # 从车头前 5cm 开始采样
        while d <= NUDGE_DIST + 0.10 + 1e-9:  # 一直查到目标距离再往前 10cm
            px = x + d * cos(yaw)  # 采样点 X
            py = y + d * sin(yaw)  # 采样点 Y
            cl = self._clearance(px, py, cap=0.15)  # 该点的净空(到真障碍的距离)
            if cl < NUDGE_MIN_CLEAR:  # 前方真有东西 → 不拱
                rospy.logwarn(
                    "[前拱] 前方 %.2fm 处净空只有 %.2fm (<%.2fm), 不拱",
                    d, cl, NUDGE_MIN_CLEAR
                )
                return False
            d += 0.05  # 下一个采样点
        return True  # 前方一路空旷, 可以拱

    def nudge_forward(self):  # 低速往前拱一小段, 给 TEB 一个新的起点
        """调用前 move_base 的目标必须已经取消(否则两边抢 /cmd_vel)。
           返回 True = 至少挪动了 NUDGE_MIN_PROGRESS, 值得重发目标;
           返回 False = 前方堵死 / 轮子空转, 别浪费重试。"""
        pose0 = self._robot_pose()  # 记录起始位姿
        if pose0 is None:  # 没有位姿就没法量进度
            rospy.logwarn("[前拱] 拿不到 map→base_link 位姿, 放弃前拱")
            return False
        if not self._forward_clear(pose0):  # 前方预检
            return False

        rospy.logwarn(
            "[前拱] 低速前拱 %.0fcm(%.2fm/s), 最多 %.0fs",
            NUDGE_DIST * 100, NUDGE_SPEED, NUDGE_MAX_TIME
        )
        cmd = Twist()  # 前拱速度指令
        cmd.linear.x = NUDGE_SPEED  # 全向底盘, 只给前进方向的速度
        t0 = time.time()  # 开始时刻
        moved = 0.0  # 已经移动的距离
        rate = rospy.Rate(20)  # 20Hz 连续发, 底盘看门狗不会断
        while not rospy.is_shutdown() and time.time() - t0 < NUDGE_MAX_TIME:
            self.cmd_vel_pub.publish(cmd)  # 发一次前进速度
            pose = self._robot_pose()  # 量一下走到哪了
            if pose is not None:
                dx = pose[0] - pose0[0]  # X 位移
                dy = pose[1] - pose0[1]  # Y 位移
                moved = (dx * dx + dy * dy) ** 0.5  # 实际移动距离
                if moved >= NUDGE_DIST:  # 拱够了
                    break
                # 拱了 1 秒还几乎没动 → 前面被顶住了, 立刻停, 别让轮子干磨
                if time.time() - t0 > NUDGE_STALL_TIME and moved < 0.01:
                    rospy.logwarn("[前拱] %.1fs 内几乎没动, 提前停止", NUDGE_STALL_TIME)
                    break
            rate.sleep()
        self.stop_robot()  # 无论结果如何都先把车停稳

        if moved < NUDGE_MIN_PROGRESS:  # 挪得太少 → 前拱无效
            rospy.logwarn("[前拱] 只移动了 %.0fcm, 前方可能真的过不去", moved * 100)
            return False
        rospy.logwarn("[前拱] 前进了 %.0fcm, 准备重新导航", moved * 100)
        return True  # 前拱有效

    # ========================= 播放音频 =========================

    # def play_audio(self, filename):  # 播放指定音频
    #     self.stop_robot()  # 播放前确保机器人停止

    #     if not filename:  # 判断音频名称是否为空
    #         rospy.loginfo("当前阶段没有配置音频，跳过播放")  # 输出跳过信息
    #         return True  # 空音频视为正常
        
    #     path = os.path.join(  # 拼接音频完整路径
    #         self.audio_dir,
    #         filename
    #     )

    #     if not os.path.isfile(path):  # 检查音频文件是否存在
    #         rospy.logerr(  # 输出文件不存在错误
    #             "语音文件不存在: %s",
    #             path
    #         )
    #         return False  # 返回失败

    #     rospy.loginfo(  # 输出播放文件
    #         "播放语音: %s",
    #         path
    #     )

    #     try:  # 尝试播放
    #         subprocess.check_call(  # 调用mplayer并等待播放完成
    #             [
    #                 'mplayer',  # 播放程序
    #                 '-really-quiet',  # 静默播放
    #                 path  # 音频文件路径
    #             ],
    #             stdout=subprocess.DEVNULL,  # 屏蔽标准输出
    #             stderr=subprocess.DEVNULL  # 屏蔽错误输出
    #         )

    #     except (OSError, subprocess.CalledProcessError) as exc:  # 捕获播放异常
    #         rospy.logerr(  # 输出异常
    #             "播放语音失败: %s",
    #             exc
    #         )
    #         return False  # 返回播放失败

    #     self.stop_robot()  # 音频播放结束后再次确保机器人停止

    #     return True  # 返回播放成功

    def play_audio(self, filename, retries=2):
        # 播放指定音频; mplayer提前退出自动重试, 保证播报完整
        self.stop_robot()
        if not filename:
            rospy.loginfo("当前阶段没有配置音频，跳过播放")
            return True
        path = os.path.join(self.audio_dir, filename)
        if not os.path.isfile(path):
            rospy.logerr("语音文件不存在: %s", path)
            return False
        for attempt in range(1, retries + 1):
            # 清掉可能残留的mplayer, 防止声卡被占导致本次播放秒退
            subprocess.call(['pkill', '-f', 'mplayer'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rospy.sleep(0.15)
            rospy.loginfo("播放语音(第%d/%d次): %s", attempt, retries, path)
            proc = subprocess.Popen(['mplayer', '-really-quiet', path],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
            start = time.time()
            proc.wait()                       # 阻塞直到mplayer退出
            elapsed = time.time() - start
            if proc.returncode == 0:
                rospy.loginfo("播放完成: %s, 耗时%.1f秒", path, elapsed)
                self.stop_robot()
                return True
            rospy.logwarn("mplayer异常退出rc=%d, 本次仅播%.1f秒, 准备重试",
                        proc.returncode, elapsed)
        rospy.logerr("语音重试%d次仍失败: %s", retries, path)
        self.stop_robot()
        return False


    # ========================= 识别结果播报 =========================

    def play_answer(self, image_index, result):  # 播放第几张图像识别后的音频
        filename = self.answer_audio_template.format(  # 按图像序号+任务号生成文件名
            image_index,
            result
        )
        return self.play_audio(filename)  # 播放识别结果音频


    # ========================= 任务点播报 =========================

    def play_task_arrival(self, task_id):  # 播放到达任务点后的音频
        filename = self.task_arrival_audio_template.format(  # 只按任务编号生成文件名
            task_id
        )
        return self.play_audio(filename)  # 播放任务点音频


    # ========================= 等待语音唤醒 =========================

    def wait_for_wakeup(self):  # 等待语音唤醒
        rospy.loginfo("等待语音唤醒，请说：你好小创")  # 输出等待提示
        rate = rospy.Rate(10)  # 使用10Hz轮询

        while not rospy.is_shutdown():  # ROS正常运行时持续等待
            start = rospy.get_param(  # 获取官方ASR唤醒参数
                '/asrpro_node/start',
                255
            )
            if start != 255:  # 255表示没有唤醒
                rospy.loginfo(  # 输出唤醒结果
                    "收到语音唤醒，code=%s",
                    start
                )
                rospy.set_param(  # 清除本次唤醒状态
                    '/asrpro_node/start',
                    255
                )
                return True  # 返回已经唤醒

            rate.sleep()  # 等待下一轮检查

        return False  # ROS关闭后返回失败

    # ========================= VLM触发 =========================

    def trigger_vlm(self):  # 触发官方VLM识别
        rospy.set_param(  # 设置官方VLM触发参数
            '/abot_vlm_node/im_flag',
            1
        )

    def get_ai_res(self):  # 获取VLM当前结果
        return (  # 返回结果和时间戳
            rospy.get_param(  # 获取识别结果
                '/abot_vlm_node/im_flog',
                VLM_DEFAULT_RESULT
            ),
            rospy.get_param(  # 获取识别结果时间戳
                '/abot_vlm_node/im_flog_memory_time',
                0.0
            )

        )

    # ========================= VLM识别 =========================

    def recognize_task_image(self, image_index):  # 识别指定任务图像
        for attempt in range(  # 按重试次数循环
            1,
            self.vlm_retry + 1
        ):

            _, old_time = self.get_ai_res()  # 保存当前旧结果时间戳

            rospy.set_param(  # 清空上一次结果
                '/abot_vlm_node/im_flog',
                VLM_DEFAULT_RESULT
            )

            self.trigger_vlm()  # 触发新的VLM识别

            rospy.loginfo(  # 输出识别进度
                "任务图像%d：第%d/%d次VLM识别",
                image_index,
                attempt,
                self.vlm_retry
            )

            deadline = time.time() + self.vlm_timeout  # 计算本轮识别截止时间

            while not rospy.is_shutdown() and time.time() < deadline:  # 等待VLM结果
                result, result_time = self.get_ai_res()  # 获取最新结果
                if (  # 判断是否得到新的合法结果
                    result in VALID_TASK_IDS
                    and
                    result_time > old_time

                ):
                    rospy.loginfo(  # 输出识别成功
                        "任务图像%d识别成功：%d",
                        image_index,
                        result
                    )

                    return result  # 返回识别结果

                if result not in (  # 判断是否为异常结果
                    VLM_DEFAULT_RESULT,
                    VLM_INVALID_RESULT
                ):
                    rospy.logwarn_throttle(  # 限制异常日志频率
                        2.0,
                        "VLM返回了无效任务编号：%s",
                        result
                    )
                rospy.sleep(0.1)  # 每100毫秒检查一次

            rospy.logwarn(  # 输出VLM超时
                "任务图像%d本轮VLM识别超时，准备重试",
                image_index
            )

        return None  # 多次失败后返回空结果


    # ========================= 方案B: 虚拟墙(封路换路) =========================

    # 思路: 一条路走不通时, 不清代价地图、不重试同一条路, 而是直接在"车头前方那条
    #       走不通的缝"上标几个障碍点, 让全局规划器只能换另一条路。
    # 为什么适合随机地形: 它是"撞了才知道", 不需要提前知道障碍物布局。
    # 前提: costmap_common_params.yaml 里必须配好观测源
    #       virtual_wall: {data_type: PointCloud2, topic: /virtual_wall,
    #                      marking: true, clearing: false}

    def _make_wall_cloud(self, points):  # 把坐标列表打包成 PointCloud2
        msg = PointCloud2()  # 创建点云消息
        msg.header.stamp = rospy.Time.now()  # 时间戳
        msg.header.frame_id = 'map'  # 用 map 系, 代价地图不需要做坐标变换
        msg.height = 1  # 无序点云
        msg.width = len(points)  # 点数
        msg.fields = [  # xyz 三个 float32 字段
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = False  # 小端
        msg.point_step = 12  # 每个点占 12 字节
        msg.row_step = 12 * len(points)  # 整帧字节数
        msg.is_dense = True  # 无无效点
        msg.data = b''.join(  # 打包所有点(z 固定为 0)
            struct.pack('<fff', x, y, 0.0) for x, y in points
        )
        return msg

    def publish_walls(self):  # 把当前所有封路点重新发一遍
        self.vw_pub.publish(self._make_wall_cloud(self.seals))  # 代价地图只保留最后一帧

    def seal_ahead(self, goal_xy=None):  # 在车头前方封掉一小块(约0.3m见方), 逼全局规划器换路
        try:  # 取机器人当前位姿
            trans, rot = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0)
            )
        except Exception as exc:  # 取不到变换就跳过, 不影响主流程
            rospy.logwarn("[封路] 拿不到 map→base_link 变换, 本次跳过: %s", exc)
            return False

        _, _, yaw = transformations.euler_from_quaternion(rot)  # 取朝向角
        cx = trans[0] + VW_AHEAD_DIST * cos(yaw)  # 封路中心 X
        cy = trans[1] + VW_AHEAD_DIST * sin(yaw)  # 封路中心 Y

        if goal_xy is not None:  # 封路点离目标太近就不封(否则把目标本身围死)
            dg = ((cx - goal_xy[0]) ** 2 + (cy - goal_xy[1]) ** 2) ** 0.5  # 封路中心到目标距离
            if dg < SEAL_MIN_GOAL_DIST:  # 太近
                rospy.logwarn(
                    "[封路] 封路点 (%.2f, %.2f) 离目标只剩 %.2fm (<%.2fm), 会把目标围死, "
                    "本次不封也不再重试",
                    cx,
                    cy,
                    dg,
                    SEAL_MIN_GOAL_DIST
                )
                return False  # 返回 False → 调用处直接放弃, 不做必败的重试

        for dx in (-VW_GRID_STEP, 0.0, VW_GRID_STEP):  # 3x3 点阵
            for dy in (-VW_GRID_STEP, 0.0, VW_GRID_STEP):
                self.seals.append((cx + dx, cy + dy))  # 累加到封路列表

        self.publish_walls()  # 立刻发给代价地图
        rospy.logwarn(  # 输出封路信息
            "[封路] 已在车头前方 (%.2f, %.2f) 封路, 累计 %d 个点",
            cx,
            cy,
            len(self.seals)
        )
        return True  # 封路成功

    def clear_walls(self):  # 清空所有虚拟墙(发一帧空点云即可)
        if not self.seals:  # 本来就没封过路, 不用发
            return
        self.seals = []  # 清空封路列表
        self.publish_walls()  # 空点云 → 代价地图不再标记这些点
        rospy.loginfo("[封路] 已清空虚拟墙")

    # ========================= 目标点吸附(按净空找落点) =========================

    # 问题: 任务点是场地里固定的, 但挡板是随机摆的。挡板蹭到任务点附近时, 那个格子
    #       在全局代价地图里会被膨胀带盖住 → 全局规划器判"不可达" → Failed to get a plan。
    #
    # ⚠️ 两个角色一定要分清:
    #    【要不要动】= A: 问规划器"原目标点可达吗"。可达就绝不干预, 原样发。
    #    【挪到哪里】= 必须用【净空】。为什么不能用"规划器说可达"来选点?
    #        规划器判可达的门槛只有 (硬阻挡带 − 0.04m) ≈ 6~10cm, 而车半宽是 0.14m ——
    #        一个离挡板 8cm 的点, 规划器说能到, 车物理上根本停不进去; 到了那儿 TEB
    #        就报 not feasible。任务点50/41 曾经吸附 6cm 之后仍然失败, 就是这么来的。
    #
    # 做法: 在目标点周围 SNAP_RADIUS 的圆里由近到远扫候选点, 取【最近的、净空达标】
    #       的那个。40cm 格子的格心天然就是净空峰值(约0.19m), 所以会自然落到格心附近。
    # 半径取 0.15m 的额外好处: 挡板都在离任务点 0.19m 以外, 这圈跨不过挡板, 不可能
    #       吸到隔壁任务点那一格去。
    # 数据来源: 订阅 /move_base/global_costmap/costmap(+ costmap_updates), 本地维护网格。
    #       不可通行 = 值 >= 99(内切99/致命100) 或 < 0(未知)。

    def _costmap_cb(self, msg):  # 收到整张全局代价地图
        # ⚠️ 顺序要紧: 先赋 cmap_info 再赋 cmap。
        #    rospy 不同话题的回调是并行跑的, 反过来的话会出现"cmap 已赋值、
        #    cmap_info 还是 None"的窗口, 那一刻 update 回调解包 None 会直接抛异常。
        self.cmap_info = (  # 记住几何信息
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.width,
            msg.info.height
        )
        self.cmap = list(msg.data)  # 代价数组(每格一个值)

    def _costmap_update_cb(self, msg):  # 收到局部更新, 贴进本地网格
        if self.cmap is None or self.cmap_info is None:  # 还没拿到整张图, 忽略
            return
        res, ox, oy, w, h = self.cmap_info  # 取几何信息
        if msg.x < 0 or msg.y < 0 or msg.x + msg.width > w or msg.y + msg.height > h:
            return  # 越界防御: 切片越界会让列表变长, 之后所有索引都会错位
        for row in range(msg.height):  # 逐行拷贝
            src = row * msg.width  # 源起始下标
            dst = (msg.y + row) * w + msg.x  # 目标起始下标
            self.cmap[dst:dst + msg.width] = msg.data[src:src + msg.width]

    def _clearance(self, x, y, cap=0.24):  # 点(x,y)脚下的净空(米), 最多算到 cap
        # ⚠️ 净空必须从【致死格】(>=100, 真障碍/真墙)量, 不能从 99(内切膨胀格)量!
        #    膨胀硬阻挡带是 0.13m 厚的一圈, 若从它量: 40cm 格子的格心真实净空 0.19m
        #    只会量到 0.19-0.13 = 0.06m, 永远够不到 0.17 的门槛 → 吸附永远落到 fallback。
        #    自己那格则必须"可通行"(<=98), 因为规划器不会往膨胀带里规划。
        if self.cmap is None:  # 还没有地图
            return 0.0
        res, ox, oy, w, h = self.cmap_info  # 取几何信息
        cx = int((x - ox) / res)  # 转成格坐标
        cy = int((y - oy) / res)
        if cx < 0 or cy < 0 or cx >= w or cy >= h:  # 出界
            return 0.0
        base = self.cmap[cy * w + cx]  # 自己那格
        if base >= 99 or base < 0:  # 自己在膨胀带/障碍/未知里 → 停不下
            return 0.0
        max_r = int(cap / res)  # 最多往外扫几圈
        for r in range(1, max_r + 1):  # 一圈一圈往外扫
            for i in range(-r, r + 1):  # 扫这一圈的四个边
                for mx, my in ((cx + i, cy - r), (cx + i, cy + r),
                               (cx - r, cy + i), (cx + r, cy + i)):
                    if mx < 0 or my < 0 or mx >= w or my >= h:  # 出界当障碍
                        return (r - 1) * res
                    c = self.cmap[my * w + mx]  # 这一格
                    if c >= 100 or c < 0:  # ★ 只把致死格(真障碍)和未知当"占地方"
                        return (r - 1) * res  # 净空取下界(保守)
        return cap  # 这个范围内都是空的

    def _goal_reachable(self, point):  # 问一次规划器: 原目标点本身规划得到吗
        """True=可达 / False=不可达 / None=服务不可用(未知)。
           tolerance=0 → 不做任何向外搜索, 只问"原目标点本身"。"""
        if not self.snap_ready:
            return None
        req = GetPlanRequest()  # 构造请求
        req.start.header.frame_id = ''  # 空 → 服务自己用机器人当前位姿当起点
        req.goal.header.frame_id = 'map'
        req.goal.header.stamp = rospy.Time.now()
        req.goal.pose.position.x = point[0]
        req.goal.pose.position.y = point[1]
        req.goal.pose.orientation.w = 1.0  # 朝向随便填, 这里只关心可达性
        req.tolerance = 0.0  # ★ 0 = 不搜索, 只问原点
        try:
            resp = self.make_plan(req)  # 调服务
        except rospy.ServiceException as exc:  # move_base 正忙或服务出事
            rospy.logwarn("[吸附] make_plan 调用失败: %s", exc)
            return None
        return len(resp.plan.poses) > 0  # 有路径 = 可达

    def snap_goal_if_unreachable(self, point):  # ★ 只有"规划器到不了"时才吸附
        """发目标前的预检: 规划器能到 → 原样发(不管净空多挤都不干预);
           规划器到不了 → 才按净空换一个落点。
           (净空偏挤但规划得到的情况, 例如 0.06~0.17m, 就应该原样发 —— 车挤得过去)"""
        r = self._goal_reachable(point)  # True / False / None
        if r is True:  # 规划器说可达 → 绝不干预
            rospy.loginfo("[吸附] (%.3f, %.3f) 规划器可达, 不干预", point[0], point[1])
            return point
        if r is None:  # 服务不可用 → 退回按净空判断
            rospy.logwarn("[吸附] make_plan 不可用, 退回按净空判断")
            return self.snap_goal(point)
        rospy.logwarn(  # 确实规划不到 → 才启用吸附换落点
            "[吸附] 规划器到不了 (%.2f, %.2f), 启用吸附换落点",
            point[0],
            point[1]
        )
        return self.snap_goal(point)

    def snap_goal(self, point):  # 按净空换一个"车停得下"的落点
        # 被 snap_goal_if_unreachable 调用(规划不到时), 也在导航失败后作为补救调用。
        # 目标点脚下净空够 → 原样返回; 不够 → 在 SNAP_RADIUS 圆内找最近的达标点。
        if self.cmap is None:  # 还没收到代价地图, 不吸附, 按原样发
            rospy.logwarn("[吸附] 还没收到全局代价地图, 本次不吸附(检查 costmap 话题)")
            return point

        cl0 = self._clearance(point[0], point[1])  # 先量目标点本身的净空
        if cl0 >= SNAP_MIN_CLEAR:  # 目标点本身就够宽敞 → 不动
            rospy.loginfo(  # ★ 这一行让你能看出"吸附确实跑了, 但判断不需要挪"
                "[吸附] (%.3f, %.3f) 净空 %.3fm ≥ %.2fm, 不需要挪",
                point[0],
                point[1],
                cl0,
                SNAP_MIN_CLEAR
            )
            return point

        near_ok = None  # 最近的、净空达标的点
        best_pt = None  # 净空最大的点(兜底)
        best_cl = 0.0  # 净空最大值
        r = SNAP_STEP  # 由近到远
        while r <= SNAP_RADIUS + 1e-9:  # 一圈一圈往外扫
            n = max(8, int(2 * pi * r / SNAP_STEP))  # 这一圈取几个候选
            for k in range(n):
                a = 2 * pi * k / n  # 方位角
                px = point[0] + r * cos(a)  # 候选点 X
                py = point[1] + r * sin(a)  # 候选点 Y
                cl = self._clearance(px, py)  # 算它的净空
                if cl > best_cl:  # 记录净空最大的
                    best_cl = cl
                    best_pt = (px, py)
                if cl >= SNAP_MIN_CLEAR and near_ok is None:  # 第一个达标的
                    near_ok = (px, py)
            if near_ok is not None:  # 已经找到达标的, 不用再往外找了
                break
            r += SNAP_STEP

        if near_ok is not None:  # 用最近的达标点
            dx = near_ok[0] - point[0]  # X 偏移
            dy = near_ok[1] - point[1]  # Y 偏移
            rospy.logwarn(  # 输出吸附结果
                "[吸附] 目标点被膨胀盖住, (%.2f, %.2f) → (%.2f, %.2f), 挪了 %.0fcm",
                point[0],
                point[1],
                near_ok[0],
                near_ok[1],
                100.0 * (dx * dx + dy * dy) ** 0.5
            )
            return (near_ok[0], near_ok[1], point[2])  # 保留原来的朝向要求

        if best_pt is not None and best_cl >= SNAP_FLOOR_CLEAR:  # 兜底: 至少是个停得下的点
            dx = best_pt[0] - point[0]  # X 偏移
            dy = best_pt[1] - point[1]  # Y 偏移
            rospy.logwarn(  # 输出吸附结果
                "[吸附] 没有净空 %.2fm 的点, 退而用净空最大的 (%.2f, %.2f) 净空 %.2fm, 挪了 %.0fcm",
                SNAP_MIN_CLEAR,
                best_pt[0],
                best_pt[1],
                best_cl,
                100.0 * (dx * dx + dy * dy) ** 0.5
            )
            return (best_pt[0], best_pt[1], point[2])  # 保留原来的朝向要求

        rospy.logwarn(  # 周围实在没地方停
            "[吸附] (%.2f, %.2f) 周围 %.2fm 内找不到能停车的点(最大净空 %.2fm), 用原目标点",
            point[0],
            point[1],
            SNAP_RADIUS,
            best_cl
        )
        return point  # 放弃吸附

    # 触发条件(唯一): 主控这一段导航判定失败, 也就是 goto_with_timeout 返回 False。
    # 它的两种情况:
    #   a) NAV_TIMEOUT 秒内没等到结果 → 超时取消目标 → False
    #   b) move_base 自己 abort 了(控制器失败/振荡) → 没超时就 False
    # ⚠️ move_base 的恢复行为(清代价地图)不是触发条件, 它只是在这段时间里顺便发生:
    #    以现在的参数, controller_patience=15s 时那次"清地图"发生在第15秒,
    #    而主控的 25 秒超时通常先到。
    # ⚠️ 只给"图像识别点"这一腿用。任务点和终点不发虚拟墙(理由见 goto_task_point)。
    def goto_with_fallback(self, point, label, timeout=NAV_TIMEOUT, use_seal=True):  # 走不通就封路换路
        """use_seal=False: 只在失败后换落点重试, 不发虚拟墙(跑点测试用, 免得墙影响后续判断)"""
        # ⚠️ 每段导航开始前先清掉上一段留下的封路:
        #    封路是"这一腿走不通"的证据, 对下一腿没有意义; 留着会累积,
        #    把后面某个目标唯一的那条路堵死 → 全局规划器直接 Failed to get a plan。
        self.clear_walls()

        # ★ 发目标前先问规划器"这个点规划得到吗"(不驱动车, 一次服务调用):
        #   规划得到 → 原样发, 不管净空多挤都不干预 ✓
        #   规划不到 → 才按净空换一个落点, 省掉一次注定失败的尝试
        point = self.snap_goal_if_unreachable(point)

        if self.navi.goto_with_timeout(point, timeout):  # 第1次: 按(可能已换过的)目标点走
            return True  # 到达

        rospy.logwarn(  # 第一次失败
            "%s 第一次没走到, 试着封路换路 + 换个落点再试",
            label
        )

        if not use_seal:  # 调用方明确要求不封路(跑点测试) → 只换落点重试
            rospy.loginfo("%s 本次不封路(调用方要求), 只换落点重试", label)
        elif self.seal_ahead(point):  # 封掉走不通的那条缝(离目标太近就不封)
            rospy.sleep(0.5)  # 等代价地图把虚拟墙标上去
            self.publish_walls()  # 再发一遍确保生效
        else:  # 没封成(封路点离目标太近) → 不跳过重试, 只换落点再试
            rospy.logwarn("%s 本次不封路(离目标太近), 只换落点重试", label)

        point2 = self.snap_goal(point)  # 换个"车停得下"的落点
        if self.navi.goto_with_timeout(point2, timeout):  # 第2次: 换路 + 换落点
            rospy.loginfo("%s 换路/换点后到达", label)
            return True  # 到达

        rospy.logwarn("%s 两次都没到达, 放弃这个点", label)
        return False  # 仍然失败

    # ========================= 图像点导航 =========================

    def goto_image_point(self, image_index):  # 前往指定图像识别点
        point = self.image_points[image_index - 1]  # 根据1~4取得目标坐标

        rospy.loginfo(  # 输出目标
            "前往第%d个任务图像识别点: %s",
            image_index,
            point
        )

        # 原来用的是 navi.goto() (60秒超时且永远返回 True, 失败根本发现不了),
        # 现在改成带超时 + 失败自动封路换路的版本
        self.goto_with_fallback(
            point,
            "图像点%d" % image_index
        )

        self.stop_robot()  # 到点后停止机器人

        return True  # 返回导航完成

    # ========================= 任务点导航 =========================

    def goto_task_point(self, task_id):  # 前往指定任务点
        point = self.task_points[task_id]  # 根据编号获取目标坐标

        rospy.loginfo(  # 输出目标
            "前往任务点%d，坐标=%s",
            task_id,
            point
        )

        self.clear_walls()  # 每段导航前先清掉上一段留下的封路, 免得把任务点堵死

        point = self.snap_goal_if_unreachable(point)  # ★ 规划不到才吸附

        # ⚠️ 任务点这一腿【不发虚拟墙】: 9个任务点彼此只隔约0.38m, 封一块0.3m见方的
        #    障碍再叠加膨胀带 ≈ 挡住 0.57m, 很容易把后面几个任务点一起封死。
        #    但失败后【只换落点、不封路】再试一次是安全的 —— 不封路就不会殃及邻居。
        success = self.navi.goto_with_timeout(  # 第1次: 按原目标点走
            point,
            timeout=NAV_TIMEOUT
        )

        if not success:  # 第一次没到 → 换个"车停得下"的落点再试一次(不封路)
            point2 = self.snap_goal(point)
            if point2 != point:  # 确实换到了别的落点, 才值得再走一次
                rospy.logwarn("任务点%d 第一次没到, 换个落点再试一次", task_id)
                success = self.navi.goto_with_timeout(point2, timeout=NAV_TIMEOUT)

        self.stop_robot()  # 成功或超时后都立即停止机器人

        if success:  # 判断是否到达
            rospy.loginfo(  # 输出导航成功信息
                "任务点%d已到达",
                task_id
            )
        else:  # 没到达
            rospy.logwarn(  # 输出导航失败信息
                "任务点%d未到达，按要求停下并继续下一个任务点",
                task_id
            )

        return success  # 返回本次任务点是否成功到达

    # ========================= 完整比赛流程 =========================

    def run(self):  # 执行完整比赛
        if not self.wait_for_wakeup():  # 等待语音唤醒
            return False  # 没有唤醒就结束

        self.play_audio(  # 播放比赛开始音频
            self.start_audio
        )

        task_sequence = []  # 保存4次识别结果

        for image_index in range(1, 5):  # 依次处理图像1、2、3、4
            self.goto_image_point(image_index)  # 前往当前图像识别点

            result = self.recognize_task_image(  # 调用VLM识别
                image_index
            )

            if result is None:  # 判断VLM是否最终失败
                rospy.logwarn(  # 输出失败原因
                    "任务图像%d识别失败，跳过本张图像并继续识别下一张。",
                    image_index
                )
                self.stop_robot()  # 当前识别失败后确保机器人停稳

                continue  # 不终止比赛，继续处理下一张任务图像

            task_sequence.append((image_index, result))  # 保存原始图像序号和识别结果

            self.play_answer(  # 播放当前图像对应的结果音频
                image_index,  # 第几张任务图像
                result  # 识别出来的任务编号
            )

        rospy.loginfo(  # 输出最终四个任务
            "已完成的任务线索顺序=%s",
            [task_id for _, task_id in task_sequence]
        )

        for task_order, (image_index, task_id) in enumerate(  # 按原始识别顺序导航
            task_sequence,
            start=1
        ):

            rospy.loginfo(  # 输出任务信息
                "第%d个已识别任务：第%d张图像识别结果=%d，前往任务点%d",
                task_order,
                image_index,
                task_id,
                task_id
            )

            success = self.goto_task_point(task_id)  # 以30秒限制尝试进入当前任务点

            self.play_task_arrival(  # 到达任务点后只按任务编号播报
                task_id
            )

            if not success:  # 当前任务点没有在30秒内成功到达
                rospy.logwarn(  # 输出继续执行提示
                    "任务点%d未成功到达，已完成当前30秒窗口，继续下一个任务点",
                    task_id
                )

        rospy.loginfo(  # 输出返回终点提示
            "四个任务点全部完成，前往9号终点"
        )

        self.clear_walls()  # 每段导航前先清掉封路, 免得把终点堵死

        fin = self.snap_goal_if_unreachable(self.finish_point)  # ★ 规划不到才吸附

        # 终点【不设超时】(用 goto_no_timeout): 它是最后一腿, 走不到就是丢分, 所以让它
        # 一直走, 直到 move_base 自己给出成功/失败, 主控中途不掐断。
        # ⚠️ 窄道卡死自救: move_base 在窄道里不 abort 只是原地不动时, 主控会永远等下去,
        #    所以这里把卡死检测挂上去 —— 连续 4 秒不动就取消目标、低速前拱 18cm、
        #    再重发同一个目标, 让 TEB 从新位置重新规划。最多前拱 FINISH_MAX_NUDGES 次。
        self._stuck_hist = []  # 从干净的检测窗口开始
        finish_kwargs = dict(  # 两段兜底用同一套自救参数
            stuck_detector=self._stuck_detector,
            on_stuck=self.nudge_forward,
            max_recoveries=FINISH_MAX_NUDGES,
        )
        ok = self.navi.goto_no_timeout(fin, **finish_kwargs)  # 前往9号终点(不设超时)
        if not ok:  # move_base 自己判定失败 → 再换个落点试一次
            fin2 = self.snap_goal(fin)
            if fin2 != fin:
                rospy.logwarn("9号终点没走到, 换个落点再试一次")
                self._stuck_hist = []  # 重新攒检测窗口
                self.navi.goto_no_timeout(  # 同样不设超时
                    fin2, **finish_kwargs
                )

        self.stop_robot()  # 到达终点后停止

        self.play_audio(  # 播放“已到达终点”
            self.arrive_finish_audio
        )

        self.play_audio(  # 播放“比赛结束”
            self.competition_end_audio
        )

        rospy.loginfo(  # 输出比赛完成
            "比赛流程完成"
        )

        return True  # 返回比赛成功


# ========================= 程序入口 =========================

def main():  # Python程序入口
    rospy.init_node(  # 初始化ROS节点
        'navigation_demo',
        anonymous=True
    )

    try:  # 捕获程序异常
        navi = navigation_demo()  # 创建导航对象

        controller = CompetitionController(  # 创建比赛控制器
            navi
        )

        controller.run()  # 执行比赛流程

    except rospy.ROSInterruptException:  # 捕获ROS正常中断
        pass  # 正常退出

    except Exception as exc:  # 捕获其他异常
        rospy.logerr(  # 输出异常信息
            "比赛主程序异常：%s",
            exc
        )

if __name__ == '__main__':  # 判断是否直接运行本文件

    main()  # 启动主程序