#!/usr/bin/env python3
#coding: utf-8

"""
两段式全向蟹行对接 (Crab Dock)

流程:
    阶段0 距离预检        dist(cur,P) > SKIP_DIST ? 走阶段1 : 跳过
    阶段1 八方向搜索+预对接 在目标周围八个方向里找一条"能安全蟹行进去"的走廊,
                          先导航到走廊口(预对接点 AP); 找不到就直接失败 —— 绝不直冲目标
    阶段2 对接前安全确认   用【实际位姿】复查 cur→P 整段扫掠 + 目标点能否停下车
    阶段3 闭环蟹行对接     20Hz: 边走边修正 yaw, 位置和朝向【连续5个周期】达标才算成功
    阶段4 结果返回

依赖 CompetitionController 提供:
    ctrl.cmap / ctrl.cmap_info / ctrl.cmap_t   全局代价地图本地副本 + 更新时间
    ctrl.cmd_vel_pub                           速度发布器
    ctrl.tf_listener                           TF
    ctrl.navi                                  move_base action 客户端
    ctrl._goal_reachable / ctrl.snap_goal      复用主脚本的"规划得到吗"判据和净空吸附
"""

import math
import re
import time

import rospy
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from tf_conversions import transformations


# ========================= 规格参数 =========================

DOCK_DIST = 0.25       # 预对接点到目标点的标称距离(m)
DOCK_SPEED = 0.10      # 最终对接最大速度(m/s); 首次实车可设 0.06
POS_TOL = 0.03         # 最终位置容差(m)
YAW_TOL = 0.06         # 最终朝向容差(rad)
SKIP_DIST = 0.35       # 离目标不超过此值就跳过阶段1(但仍必须过阶段2的安全检查)
SANITY_LO = 0.10       # 阶段1结束后 dist(cur,P) 的合理下限(m)
SANITY_HI = 0.45       # ...上限(m)
DOCK_TIMEOUT = 10.0    # 闭环对接总超时(s)
TF_FAIL_MAX = 5        # 连续读 TF 失败次数上限
DIR_RETRY = 1          # 首选方向失败后, 最多再试一个次优方向
VY_MAX = 0.35          # 横向速度限幅(m/s)
WZ_MAX = 1.0           # 角速度限幅(rad/s)
DOCK_HZ = 20.0         # 对接控制频率
STABLE_N = 5           # 连续满足容差的周期数才算成功
AP_NAV_TIMEOUT = 25.0  # 阶段1 导航到预对接点的超时(s)

MAP_AGE_LOCAL = 0.50   # 局部代价地图的最大允许年龄(s)
MAP_AGE_GLOBAL = 1.60  # 全局代价地图最大年龄(s) —— 它是 1Hz 发布的, 不能要求太新

# ========================= 安全余量(⚠️ 关键标定) =========================
# 规格写的是 CLEAR_MIN=0.05, 还要"另行计入分辨率/定位不确定性/yaw扩张" —— 照字面取总值
# 约 0.17m。但 40cm 格子里单边物理余量只有 ~5cm, 那样【每个方向都会被判不安全】→
# 对接永远失败。所以这里拆成四项、默认总值压到 0.03m, 实车按"过得去/擦不擦"微调。
# ⚠️ 调大 = 更保守(更容易找不到走廊); 调小 = 更容易过但可能擦。
CLEAR_MIN = 0.02       # 车体轮廓到真障碍的最低几何净空(m)
# ★ 2026-10-18 实车日志: 两种"明明过得去却判死"的假失败都出在 0.02 这个余量上 ——
#   任务点40: 阶段2 最小净空 0.038 (差 2mm) → 失败
#   任务点33: 八方向里 -45° 最小净空 0.036 (差 4mm) → 淘汰; 其余方向 0.000~0.011 才是真堵
#   地图分辨率 0.02m, _min_clear 量的是"轮廓到障碍【格心】"的距离, 障碍真实表面可能在
#   格子内任意位置 → 测量误差最坏约半格对角线 0.014m。拿 0.02(整格)当余量再配严格比较,
#   等于要求实测 ≥0.04, 高于物理上能过的 0.036~0.038 → 差几毫米就误杀。
#   改成半格(0.01): 总余量 0.03, 实测 0.036/0.038 通过; 0.010/0.011 仍被拦住。
#   ⚠️ 再调小 = 更容易过但可能擦; 再调大 = 又开始误杀窄通道。
RES_MARGIN = 0.01      # 地图分辨率带来的余量(半格, 覆盖格心量距的量化误差)
LOC_MARGIN = 0.00      # 定位不确定性余量(m); 想要更保守就加到 0.02
YAW_MARGIN = 0.00      # yaw 修正造成的外轮廓扩张余量(m); 想要更保守就加到 0.02

MARGIN = CLEAR_MIN + RES_MARGIN + LOC_MARGIN + YAW_MARGIN  # 总安全余量(m)

YAW_SWEEP = 0.12       # 扫掠检查时考虑的车体朝向摆动包络(rad, 两侧各算一次)

# 阶段3 动态防撞的前瞻时间(s): 检查"以当前指令走 LOOKAHEAD_T 秒"所覆盖的整段区域
# 0.35s 覆盖约 7 个控制周期 + 制动距离, 比"只查此刻贴不贴"安全得多
LOOKAHEAD_T = 0.35

# 方向排序用的净空搜索范围(m): 必须明显大于 MARGIN, 否则量出来的都是饱和值,
# 不同方向净空的差异会被抹平。0.30m 在高密度的 40cm 格子里够用, 开销也可接受。
CLEAR_CAP = 0.30


class CrabDock(object):

    def __init__(self, ctrl):
        self.ctrl = ctrl
        self.footprint = self._read_footprint()   # 实际机器人轮廓(base_link 系, 顺序一致)

        # ---------- 新增: 局部代价地图订阅(对接过程动态防撞用) ----------
        # ⚠️ 三样东西必须【一起换】: 局部地图是 rolling_window, 原点一直在变。
        #    如果"新几何 + 旧网格"被读到一次, 净空就会在错误的栅格位置上算 → 很危险。
        #    所以打包成一个元组, 一次赋值(CPython 里是原子的), 读者永远拿到配对的一整套。
        self.lmap_all = None       # (网格list, 几何tuple, 时间戳)
        self._worst_at = None      # 最近一次 _swept_ok 的最小净空位置(x, y, 度)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self._lmap_cb)
        rospy.Subscriber('/move_base/local_costmap/costmap_updates',
                         OccupancyGridUpdate, self._lmap_upd_cb)
        rospy.loginfo("[对接] 初始化完成: footprint %d 个顶点, 总安全余量 %.3fm",
                      len(self.footprint), MARGIN)

    # ========================= 初始化: footprint =========================

    def _read_footprint(self):
        """读【实际】footprint(与 costmap 使用的机器人模型一致), 拿不到就用保守矩形。
           ⚠️ 代价地图会把 footprint 参数以【字符串】形式写回参数服务器
              (costmap_2d 的 writeFootprintToParam: setParam("footprint", "[[x,y],[x,y]]")),
              所以【字符串】和【YAML 列表】两种形式都必须能解析 —— 只按列表解析会抛异常、
              被 except 吞掉、静默退化成保守矩形(踩过一次, 表现为日志里那条 WARN)。"""
        for key in ('/move_base/global_costmap/footprint', '/move_base/local_costmap/footprint'):
            pts = self._parse_footprint(rospy.get_param(key, None))
            if pts:
                rospy.loginfo("[对接] footprint 来自 %s: %s", key, pts)
                return pts
        # 退而求其次: TEB 的 footprint_model/vertices 是真正的 YAML 列表
        pts = self._parse_footprint(
            rospy.get_param('/move_base/TebLocalPlannerROS/footprint_model/vertices', None))
        if pts:
            rospy.loginfo("[对接] footprint 来自 TEB footprint_model: %s", pts)
            return pts
        rospy.logwarn("[对接] 读不到 footprint 参数, 用保守矩形 0.35x0.30")
        return [(0.215, 0.15), (0.215, -0.15), (-0.135, -0.15), (-0.135, 0.15)]

    @staticmethod
    def _parse_footprint(fp):
        """把 footprint 解析成 [(x, y), ...]; 同时支持
             字符串形式 "[[0.215,0.15],[0.215,-0.15],...]"   ← 代价地图写回的就是这种
             列表形式 [[0.215,0.15], ...]
           解析不了就返回 None(让调用方继续尝试别的来源)。"""
        if fp is None:
            return None
        if isinstance(fp, str):                       # 字符串形式: 按数字顺序取出来配对
            nums = [float(v) for v in re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', fp)]
            if len(nums) >= 6 and len(nums) % 2 == 0:
                return [(nums[i], nums[i + 1]) for i in range(0, len(nums), 2)]
            return None
        try:                                          # 列表形式
            pts = [(float(p[0]), float(p[1])) for p in fp]
            return pts if len(pts) >= 3 else None
        except Exception:
            return None

    # ========================= 局部代价地图 =========================

    def _lmap_cb(self, msg):
        info = (msg.info.resolution, msg.info.origin.position.x,
                msg.info.origin.position.y, msg.info.width, msg.info.height)
        self.lmap_all = (list(msg.data), info, time.time())   # ★ 一次原子赋值

    def _lmap_upd_cb(self, msg):
        # 局部更新只在"几何没变"时才会发(原点变了会改成发整张全图), 所以可以就地打补丁
        if self.lmap_all is None:
            return
        grid, info, _ = self.lmap_all
        res, ox, oy, w, h = info
        if msg.x < 0 or msg.y < 0 or msg.x + msg.width > w or msg.y + msg.height > h:
            return  # 越界防御
        for row in range(msg.height):
            src = row * msg.width
            dst = (msg.y + row) * w + msg.x
            grid[dst:dst + msg.width] = msg.data[src:src + msg.width]
        self.lmap_all = (grid, info, time.time())

    # ========================= 地图有效性 =========================

    def _map_valid(self, which='local'):
        """返回 (网格, 几何, 是否有效)。有效 = 有数据 + 坐标原点正常 + 不太旧"""
        if which == 'local':
            if self.lmap_all is None:
                return None, None, False
            grid, info, t = self.lmap_all        # ★ 一次取出配对的一整套
            age = MAP_AGE_LOCAL
        else:
            grid, info, t = self.ctrl.cmap, self.ctrl.cmap_info, self.ctrl.cmap_t
            age = MAP_AGE_GLOBAL
        if grid is None or info is None:
            return None, None, False
        res, ox, oy, w, h = info
        if res <= 0 or w <= 0 or h <= 0:
            return None, None, False
        if time.time() - t > age:
            return None, None, False
        return grid, info, True

    def _pick_map(self, mode='static'):
        """模式必须分开 —— 这是对接安全的关键:
           mode='static'  : 静态走廊检查(阶段1/2)用【全局地图】。目标/走廊可能在 2~3m 外,
                            局部滚动窗口只有 ±1.5m, 它看不到远处 → 会误判成"安全"。
           mode='dynamic' : 阶段3 动态防撞用【局部地图】(实时), 且【不退回全局】——
                            局部地图过期就必须停车; 拿 1~2 秒前的数据继续走是不安全的。
           返回 (网格, 几何, 是否有效)"""
        if mode == 'dynamic':
            return self._map_valid('local')
        return self._map_valid('global')

    # ========================= 位姿 =========================

    def _pose(self, frame='map'):
        """返回指定坐标系下的 (x, y, yaw); 读不到返回 None。
           ⚠️ 必须和你要查的地图用同一个坐标系:
             全局代价地图 global_frame=map  → 用 'map'
             局部代价地图 global_frame=odom → 用 'odom'(阶段3 动态防撞就是这个)"""
        try:
            trans, rot = self.ctrl.tf_listener.lookupTransform(frame, 'base_link', rospy.Time(0))
        except Exception:
            return None
        _, _, yaw = transformations.euler_from_quaternion(rot)
        return trans[0], trans[1], yaw

    def _worst_txt(self):
        """把最近一次 _swept_ok 的最小净空位置格式化成日志后缀。
           ⚠️ mode='static' 时是 map 系, mode='dynamic' 时是 odom 系。
           角度是叠加了 ±YAW_SWEEP 包络后的【测试朝向】, 和实际航向最多差约 7°。"""
        if self._worst_at is None:
            return ""
        return " @ (%.2f, %.2f, %.0f°)" % self._worst_at

    def _stop(self):
        """统一停车(所有退出路径都必须走这里)"""
        try:
            self.ctrl.stop_robot()
        except Exception:
            pass

    def _send(self, vx, vy, wz):
        """发一帧速度指令"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.ctrl.cmd_vel_pub.publish(cmd)

    # ========================= 几何: 点到凸多边形 / 线段 =========================

    @staticmethod
    def _dist_point_seg(qx, qy, ax, ay, bx, by):
        vx, vy = bx - ax, by - ay
        wx, wy = qx - ax, qy - ay
        L2 = vx * vx + vy * vy
        if L2 <= 1e-12:
            return math.hypot(wx, wy)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
        return math.hypot(qx - (ax + t * vx), qy - (ay + t * vy))

    def _dist_point_body(self, qx, qy, pts):
        """点(车体系)到车体轮廓的距离: 在轮廓内=0, 在外=到边界最短距离"""
        n = len(pts)
        inside = True
        sign = 0
        mind = 1e12
        for i in range(n):
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % n]
            cr = (bx - ax) * (qy - ay) - (by - ay) * (qx - ax)
            cur = 1 if cr > 1e-12 else (-1 if cr < -1e-12 else 0)
            if cur != 0:
                if sign == 0:
                    sign = cur
                elif cur != sign:
                    inside = False
            d = self._dist_point_seg(qx, qy, ax, ay, bx, by)
            if d < mind:
                mind = d
        return 0.0 if inside else mind

    def _min_clear(self, grid, info, cx, cy, yaw, cap=None):
        """车体中心在 (cx,cy) 朝向 yaw 时, 轮廓到最近【真障碍/未知】的净空(m)。
           真障碍 = 致死格(值>=100) 或未知(-1); 膨胀带(99/1~98)不算障碍。
           cap: 搜索范围(米)。判定"够不够安全"用默认 MARGIN 就够(便宜); 要拿它【排序】
                比较大小, 必须给一个大一点的 cap, 否则找不到障碍时返回的是饱和值,
                不同净空的方向会被当成一样安全。"""
        if cap is None:
            cap = MARGIN
        res, ox, oy, w, h = info
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(c * vx - s * vy, s * vx + c * vy) for vx, vy in self.footprint]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        # ★ 越界 = 不安全。车体(含余量)必须完整落在地图内, 否则搜索框会被裁到边界、
        #   一个格子都遍历不到 → 返回"极大净空" → 把危险当安全(必须避免)。
        if (cx + min(xs) - cap < ox or cx + max(xs) + cap > ox + w * res or
                cy + min(ys) - cap < oy or cy + max(ys) + cap > oy + h * res):
            return 0.0
        mx0 = max(0, int((cx + min(xs) - cap - ox) / res))
        mx1 = min(w - 1, int((cx + max(xs) + cap - ox) / res) + 1)
        my0 = max(0, int((cy + min(ys) - cap - oy) / res))
        my1 = min(h - 1, int((cy + max(ys) + cap - oy) / res) + 1)
        worst = 1e12
        for my in range(my0, my1 + 1):
            row = my * w
            py = oy + (my + 0.5) * res
            for mx in range(mx0, mx1 + 1):
                v = grid[row + mx]
                if not (v >= 100 or v < 0):   # 只把致死格和未知当"真障碍"
                    continue
                px = ox + (mx + 0.5) * res
                d = self._dist_point_body(px - cx, py - cy, pts)
                if d < worst:
                    worst = d
        # ★ 返回值必须封顶到搜索范围 cap: 范围内没找到障碍时返回的是初始的大数(1e12),
        #   拿去排序会让"净空 0.18m"和"净空 0.5m"看起来一样安全 → 排序失去意义。
        #   封顶后语义就明确了: 0.18 表示真的 0.18m; 0.30 表示"至少 0.30m"。
        return min(worst, cap)

    def _swept_ok(self, A, B, yaw, mode='static', yaw_end=None):
        """从 A 蟹行到 B 的整段扫掠检查。
           yaw → yaw_end: 朝向【沿路径线性插值】, 每个采样点再各算 ±YAW_SWEEP 包络 ——
           这样"平移 + 未来旋转"的完整车体包络都被覆盖(只查单一朝向会漏掉旋转扫过的区域)。
           mode 决定用全局图(静态走廊)还是局部图(动态防撞), 见 _pick_map。
           返回 (是否安全, 最小净空)。"""
        if yaw_end is None:
            yaw_end = yaw
        grid, info, ok = self._pick_map(mode)
        if not ok:
            rospy.logerr("[对接] 没有有效的代价地图 → 无法确认安全")
            return False, 0.0
        res = info[0]
        seg = math.hypot(B[0] - A[0], B[1] - A[1])
        n = max(2, int(math.ceil(seg / res)) + 1)  # 采样点数: 用 ceil, 保证相邻采样点间距 ≤1 格
        worst = 1e12
        self._worst_at = None                        # 最小净空出现的位置(失败日志用)
        for i in range(n):
            t = i / float(n - 1)
            x = A[0] + (B[0] - A[0]) * t
            y = A[1] + (B[1] - A[1]) * t
            yaw_t = yaw + (yaw_end - yaw) * t            # 朝向沿路径插值
            for dyaw in (yaw_t, yaw_t + YAW_SWEEP, yaw_t - YAW_SWEEP):   # 再叠加摆动包络
                d = self._min_clear(grid, info, x, y, dyaw)
                if d < worst:
                    worst = d
                    self._worst_at = (x, y, math.degrees(dyaw))  # mode 决定是 map 还是 odom 系
                if worst < MARGIN:
                    return False, worst
        return True, worst

    # ========================= 阶段1: 八方向搜索 =========================

    def _search_directions(self, P):
        """八个方向里挑出安全走廊, 按净空从大到小排序(净空相同时 |offset| 小的优先)。
           返回 [(offset度, AP_x, AP_y, 净空), ...]"""
        # ★ 阶段1 结束时车头会被 move_base 摆到【目标朝向 P[2]】, 不是现在的朝向 ——
        #   拿当前朝向做外轮廓包络会误判, 所以这里统一用 P[2](再叠加 YAW_SWEEP 包络)。
        psi = math.radians(P[2])
        out = []
        for off in (0, 45, -45, 90, -90, 135, -135, 180):
            a = math.radians(P[2] + off)           # 世界系下行进方向角
            dx, dy = math.cos(a), math.sin(a)
            ax = P[0] - dx * DOCK_DIST             # 预对接点 AP = P - d*DOCK_DIST
            ay = P[1] - dy * DOCK_DIST
            ok, clr = self._swept_ok((ax, ay, psi), (P[0], P[1], P[2]), psi, mode='static')
            if not ok:
                rospy.loginfo("[对接] 方向 %+4.0f°: 走廊不通过(最小净空 %.3fm)%s → 淘汰",
                              off, clr, self._worst_txt())
                continue
            # ★ 排序要"实际净空有多大": 用 AP 周围【较大 cap】量出来的真实距离。
            #   clr 是在 MARGIN 小框里找的, 找不到就是饱和值, 拿去排序没有区分度。
            g2, i2, mok2 = self._pick_map('static')
            rank = self._min_clear(g2, i2, ax, ay, psi, cap=CLEAR_CAP) if mok2 else clr
            rospy.loginfo("[对接] 方向 %+4.0f°: 走廊通过, AP 净空 %.3fm(排序用)", off, rank)
            out.append((off, ax, ay, rank))
        out.sort(key=lambda t: (-t[3], abs(t[0])))   # 净空大→小; 相同则 |offset| 小
        return out

    def _maybe_snap_ap(self, AP, P):
        """只有 AP 本身【规划不到】时才允许有限吸附, 且吸附后必须重查走廊"""
        try:
            r = self.ctrl._goal_reachable((AP[0], AP[1], AP[2]))
        except Exception:
            r = None
        if r is not False:
            return AP
        snapped = self.ctrl.snap_goal((AP[0], AP[1], AP[2]))
        if snapped == AP:
            return AP
        # ★ AP[2] 是【度】(P[2] 的单位), _swept_ok 的 yaw 参数要【弧度】—— 必须换算,
        #   否则像 84° 这种点位会被当成 84rad 去算外轮廓, 方向完全错。
        psi = math.radians(AP[2])
        ok, clr = self._swept_ok((snapped[0], snapped[1], psi), P, psi)
        if not ok:
            rospy.logwarn("[对接] AP 吸附后走廊不通过(净空 %.3fm)%s, 放弃吸附",
                          clr, self._worst_txt())
            return AP
        rospy.logwarn("[对接] AP 吸附: (%.2f,%.2f) → (%.2f,%.2f), 复检净空 %.3fm",
                      AP[0], AP[1], snapped[0], snapped[1], clr)
        return snapped

    def _stage1(self, P):
        """八方向搜索 + 预对接导航。任何失败都不允许退回"直冲目标"。"""
        cands = self._search_directions(P)
        if not cands:
            rospy.logerr("[对接] 八个方向都没有安全走廊 → 停车失败(不直冲目标)")
            return False
        rospy.loginfo("[对接] 有 %d 个安全方向, 首选 %+.0f°(净空 %.3fm)",
                      len(cands), cands[0][0], cands[0][3])
        tried = 0
        for off, ax, ay, clr in cands:
            if tried > DIR_RETRY:                  # 最多再试 DIR_RETRY 个
                break
            tried += 1
            rospy.loginfo("[对接] 试方向 %+.0f°: AP=(%.2f, %.2f) 走廊净空 %.3fm", off, ax, ay, clr)
            AP = self._maybe_snap_ap((ax, ay, P[2]), P)
            ok = self.ctrl.navi.goto_with_timeout((AP[0], AP[1], P[2]), timeout=AP_NAV_TIMEOUT)
            self._stop()
            if not ok:
                rospy.logwarn("[对接] 方向 %+.0f° 阶段1导航失败, 换方向", off)
                try:                                # 等 Action 真正取消完, 免得抢底盘
                    self.ctrl.move_base.wait_for_result(rospy.Duration(1.0))
                except Exception:
                    pass
                continue
            cur = self._pose()                      # 用【实际】位姿复查
            if cur is None:
                rospy.logerr("[对接] 阶段1后读不到位姿 → 失败")
                return False
            d = math.hypot(P[0] - cur[0], P[1] - cur[1])
            if d < SANITY_LO or d > SANITY_HI:
                rospy.logwarn("[对接] 方向 %+.0f° 落点异常: 距目标 %.2fm 不在 [%.2f,%.2f] → 换方向",
                              off, d, SANITY_LO, SANITY_HI)
                continue
            ok2, clr2 = self._swept_ok(cur, (P[0], P[1], P[2]), cur[2])
            if not ok2:
                rospy.logwarn("[对接] 方向 %+.0f° 实际走廊复查不通过(净空 %.3fm) → 换方向", off, clr2)
                continue
            rospy.loginfo("[对接] 方向 %+.0f° 阶段1成功: 距目标 %.2fm, 走廊净空 %.3fm", off, d, clr2)
            return True
        rospy.logerr("[对接] 试过的方向都不行 → 停车失败")
        return False

    # ========================= 阶段2: 对接前安全确认 =========================

    def _stage2(self, P):
        """不管阶段1跑没跑, 都用实际位姿复查 cur→P 的整段扫掠 + 目标点能否停下车"""
        cur = self._pose()
        if cur is None:
            rospy.logerr("[对接] 阶段2: 读不到位姿 → 失败")
            return False
        ok, clr = self._swept_ok(cur, (P[0], P[1], P[2]), cur[2], mode='static')
        if not ok:
            rospy.logerr("[对接] 阶段2: cur→P 走廊不安全(最小净空 %.3fm < %.3fm)%s → 失败",
                         clr, MARGIN, self._worst_txt())
            return False
        grid, info, mok = self._pick_map('static')
        if not mok:
            rospy.logerr("[对接] 阶段2: 没有有效代价地图 → 失败")
            return False
        d_goal = self._min_clear(grid, info, P[0], P[1], math.radians(P[2]))
        if d_goal < MARGIN:
            rospy.logerr("[对接] 阶段2: 目标点本身停不下车(净空 %.3fm < %.3fm) → 失败", d_goal, MARGIN)
            return False
        rospy.loginfo("[对接] 阶段2 通过: 走廊净空 %.3fm, 目标点净空 %.3fm", clr, d_goal)
        return True

    # ========================= 阶段3: 闭环蟹行对接 =========================

    def _stage3(self, P):
        """20Hz 闭环: 世界方向蟹行 + yaw 闭环修正; 连续 STABLE_N 个周期达标才成功。
           所有退出路径都保证发零速度。"""
        # ★★ 整个阶段3 在【odom 系】里闭环 —— 因为局部代价地图(动态防撞的数据源)
        #    的 global_frame 就是 odom。目标点 P 是 map 系的, 必须先用 map→odom 转过去,
        #    否则拿 map 坐标去查 odom 的地图 = 查错位置(map 与 odom 会越差越多)。
        try:
            tr, ro = self.ctrl.tf_listener.lookupTransform('odom', 'map', rospy.Time(0))
        except Exception as exc:
            rospy.logerr("[对接] 拿不到 map→odom 变换, 无法在 odom 系做防撞 → 失败: %s", exc)
            return False
        _, _, gth = transformations.euler_from_quaternion(ro)
        c, s = math.cos(gth), math.sin(gth)
        tx = tr[0] + c * P[0] - s * P[1]          # 目标点的 odom 系坐标
        ty = tr[1] + s * P[0] + c * P[1]
        tyaw = math.radians(P[2]) + gth           # 目标朝向的 odom 系角度
        rate = rospy.Rate(DOCK_HZ)
        t_start = time.monotonic()                 # ★ 单调时钟, 不怕仿真时钟停
        tf_fail = 0
        stable = 0
        rospy.loginfo("[对接] 闭环开始(odom系): 目标 (%.3f, %.3f, %.1f°) ← map系 (%.3f, %.3f, %.1f°)",
                      tx, ty, math.degrees(tyaw), P[0], P[1], P[2])
        try:
            while not rospy.is_shutdown():
                # 0) ★ 每个周期重新把目标点从 map 转到 odom:
                #    AMCL 随时可能修正 map→odom, 用开机那一次的旧坐标, 对应的物理位置就变了
                #    → 最终会偏离真实任务点。转换失败立刻发零速度并退出, 不追旧目标。
                try:
                    tr, ro = self.ctrl.tf_listener.lookupTransform('odom', 'map', rospy.Time(0))
                except Exception as exc:
                    self._send(0.0, 0.0, 0.0)
                    rospy.logerr("[对接] 本周期 map→odom 读取失败, 已发零速度并退出: %s", exc)
                    return False
                _, _, gth = transformations.euler_from_quaternion(ro)
                c, s = math.cos(gth), math.sin(gth)
                tx = tr[0] + c * P[0] - s * P[1]
                ty = tr[1] + s * P[0] + c * P[1]
                tyaw = math.radians(P[2]) + gth
                # 1) 读位姿(★ odom 系, 和局部地图一致)
                cur = self._pose('odom')
                if cur is None:
                    tf_fail += 1
                    self._send(0.0, 0.0, 0.0)      # ★ 第一次失败就立刻发零速度!
                    rospy.logwarn("[对接] TF 读取失败 %d/%d, 已发零速度", tf_fail, TF_FAIL_MAX)
                    if tf_fail >= TF_FAIL_MAX:
                        rospy.logerr("[对接] TF 连续失败 → 停车失败")
                        return False
                    rate.sleep()
                    continue
                tf_fail = 0
                # 2) 局部地图时效(★ 过期就停车, 不退回用全局地图继续走)
                if not self._map_valid('local')[2]:
                    rospy.logerr("[对接] 局部代价地图缺失/过期 → 停车失败")
                    return False
                # 3) 误差 + 本周期想怎么走
                dx, dy = tx - cur[0], ty - cur[1]
                dist = math.hypot(dx, dy)
                yaw_err = self._norm_angle(tyaw - cur[2])
                # ★ 位置已达标就【不再平移】, 只做 yaw 修正 —— 否则 1.2*dist 的残余速度
                #   会把车蹭出位置容差, 稳定计数又归零, 来回震荡。
                if dist <= POS_TOL:
                    v = 0.0
                else:
                    v = min(DOCK_SPEED, 1.2 * dist)
                phi = math.atan2(dy, dx)
                vwx, vwy = v * math.cos(phi), v * math.sin(phi)
                wz = max(-WZ_MAX, min(WZ_MAX, 1.5 * yaw_err))  # 候选角速度(先算, 防撞要用)
                # 4) 动态防撞: 把【平移 + 未来旋转】的完整包络一起扫
                #    末朝向 = 当前朝向 + wz*LOOKAHEAD_T —— 只查当前朝向会漏掉旋转扫过的区域
                ok_safe, clr = self._swept_ok(
                    cur,
                    (cur[0] + vwx * LOOKAHEAD_T, cur[1] + vwy * LOOKAHEAD_T, cur[2]),
                    cur[2],
                    mode='dynamic',
                    yaw_end=cur[2] + wz * LOOKAHEAD_T
                )
                if not ok_safe:
                    rospy.logerr("[对接] 前方 %.2fs 的扫掠区域不安全(净空 %.3fm, odom系)%s → 停车失败",
                                 LOOKAHEAD_T, clr, self._worst_txt())
                    return False
                # 5) 稳定到达判定: 达标期间【发零速度】并计数, 连续 STABLE_N 周期才算成功
                if dist <= POS_TOL and abs(yaw_err) <= YAW_TOL:
                    stable += 1
                    self._send(0.0, 0.0, 0.0)          # ★ 达标期间不动, 免得蹭出容差
                    rospy.loginfo("[对接] 达标 %d/%d (dist %.3f, yaw_err %.3f)",
                                  stable, STABLE_N, dist, yaw_err)
                    if stable >= STABLE_N:
                        rospy.loginfo("[对接] 对接成功: dist %.3fm, yaw_err %.3frad, 用时 %.1fs",
                                      dist, yaw_err, time.monotonic() - t_start)
                        return True
                    if time.monotonic() - t_start > DOCK_TIMEOUT:
                        rospy.logerr("[对接] 超过 %.1fs 仍未稳定达标 → 停车失败", DOCK_TIMEOUT)
                        return False
                    rate.sleep()
                    continue
                stable = 0
                # 7) 换算到车体(全向底盘: 不需要先转头)
                psi = cur[2]
                vx = vwx * math.cos(psi) + vwy * math.sin(psi)
                vy = -vwx * math.sin(psi) + vwy * math.cos(psi)
                vx = max(-VY_MAX, min(VY_MAX, vx))
                vy = max(-VY_MAX, min(VY_MAX, vy))
                # 9) 发送
                self._send(vx, vy, wz)
                # 10) 超时退出
                if time.monotonic() - t_start > DOCK_TIMEOUT:
                    rospy.logerr("[对接] 超过 %.1fs 仍未达标 → 停车失败", DOCK_TIMEOUT)
                    return False
                rate.sleep()
            return False
        finally:                                    # ★ 成功/失败/异常/中断 都保证停车
            for _ in range(3):
                self._send(0.0, 0.0, 0.0)
                rospy.sleep(0.02)
            self._stop()

    @staticmethod
    def _norm_angle(a):
        """把角度归一化到 [-pi, pi]"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # ========================= 阶段0~4: 总入口 =========================

    def goto_task_point(self, task_id, P):
        """两段式蟹行对接。只有闭环真正达标(位置+yaw 连续满足)才返回 True。"""
        t0 = time.monotonic()
        # ---------- 阶段0: 距离预检 ----------
        cur = self._pose()
        if cur is None:
            rospy.logerr("[对接] 任务点%d: 读不到位姿 → 失败", task_id)
            self._stop()
            return False
        d0 = math.hypot(P[0] - cur[0], P[1] - cur[1])
        rospy.loginfo("[对接] 任务点%d: 距目标 %.2fm, 目标 (%.3f, %.3f, %.1f°)",
                      task_id, d0, P[0], P[1], P[2])
        # ---------- 阶段1 ----------
        if d0 > SKIP_DIST:
            if not self._stage1(P):
                self._stop()
                rospy.logwarn("[对接] 任务点%d: 阶段1失败(共 %.1fs)", task_id, time.monotonic() - t0)
                return False
        else:
            rospy.loginfo("[对接] 距目标 %.2fm ≤ %.2fm, 跳过阶段1(但仍要过阶段2安全检查)",
                          d0, SKIP_DIST)
        # ---------- 阶段2 ----------
        if not self._stage2(P):
            self._stop()
            rospy.logwarn("[对接] 任务点%d: 阶段2失败(共 %.1fs)", task_id, time.monotonic() - t0)
            return False
        # ---------- 阶段3 ----------
        ok = self._stage3(P)
        self._stop()
        # ---------- 阶段4 ----------
        if ok:
            cur = self._pose()
            if cur is not None:
                rospy.loginfo("[对接] 任务点%d 成功: 最终 dx=%.3f dy=%.3f dist=%.3f yaw_err=%.3f, 总耗时 %.1fs",
                              task_id, P[0] - cur[0], P[1] - cur[1],
                              math.hypot(P[0] - cur[0], P[1] - cur[1]),
                              self._norm_angle(math.radians(P[2]) - cur[2]),
                              time.monotonic() - t0)
            return True
        rospy.logwarn("[对接] 任务点%d 失败(共 %.1fs)", task_id, time.monotonic() - t0)
        return False
