#!/usr/bin/env python3
#coding: utf-8

"""
点位采集工具 (给 navigation_multi_goals.py 的 image_points / task_points / finish_point 用)

两种采集方式:

  1) 把车开到目标位姿, 在这个终端按回车
     → 记录 robot 当前的 map 位姿 (x, y, yaw°)
     → 这是最准的: 用的就是 move_base/AMCL 实际消费的 map→base_link 位姿

  2) 在 RViz 里用 "Publish Point" 工具点地图
     → 记录点击点的 (x, y) + 机器人当前朝向 (yaw°)
     → 适合"车开不过去/只想点地图"的情况; 注意 yaw 取的是车当前的朝向,
        不是你点击位置的朝向。点完记得看一眼是不是你要的朝向。

输出的每一行都可以直接粘到 navigation_multi_goals.py 的点位元组里。

前提:
  - 已经启动定位 (map→odom→base_footprint→base_link 链完整), 否则等不到 TF
  - RViz 的 Fixed Frame 必须是 map (view_navigation.rviz 就是)
  - 用 view_navigation.rviz, 它自带 Publish Point 工具

用法:
  rosrun robot_slam record_points.py
  (或 roslaunch robot_slam record_points.launch —— 会单独启动底盘+定位+RViz)

  ⚠️ 不要把 record_points.launch 和 6-mission.sh 同时跑, 会重复启动 abot_driver 抢串口。
"""

import sys

import rospy
import tf
from math import degrees
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion


class Recorder(object):

    def __init__(self):
        self.listener = tf.TransformListener()
        self.points = []  # 已采集的点 [(x, y, yaw_deg), ...]

        rospy.Subscriber('/clicked_point', PointStamped, self.on_click, queue_size=1)

        rospy.loginfo("等待 map -> base_link 的 TF ...")
        try:
            self.listener.waitForTransform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(30.0)
            )
            rospy.loginfo("TF 已就绪")
        except Exception as exc:
            rospy.logerr("等不到 map->base_link: %s", exc)
            rospy.logerr("先确认定位已经收敛、RViz 里激光和地图重合, 再采点")
            sys.exit(1)

        rospy.loginfo("=" * 60)
        rospy.loginfo("回车 = 记录机器人当前位姿")
        rospy.loginfo("RViz 里 Publish Point 点击 = 记录点击点 x,y + 车当前 yaw")
        rospy.loginfo("输入 q 回车 = 退出")
        rospy.loginfo("=" * 60)

    def current_pose(self):
        """返回 (x, y, yaw_deg); 取不到 TF 时抛异常。"""
        trans, rot = self.listener.lookupTransform(
            'map', 'base_link', rospy.Time(0)
        )
        _, _, yaw = euler_from_quaternion(rot)
        return trans[0], trans[1], degrees(yaw)

    def emit(self, x, y, yaw, source):
        self.points.append((x, y, yaw))
        rospy.loginfo(
            "[%s] 第%d个点: (%.3f, %.3f, %.1f)",
            source, len(self.points), x, y, yaw
        )
        # 这一行可以直接复制到 navigation_multi_goals.py 的点位表里
        print("        (%.3f, %.3f, %.1f),  # %s" % (x, y, yaw, source), flush=True)

    def on_click(self, msg):
        """RViz Publish Point 回调: 用点击的 x,y + 车当前 yaw。"""
        try:
            _, _, yaw = self.current_pose()
        except Exception as exc:
            rospy.logwarn("拿到点击点但取不到当前 yaw: %s", exc)
            yaw = 0.0
        self.emit(msg.point.x, msg.point.y, yaw, "click")

    def _print_summary(self):
        rospy.loginfo("共采集 %d 个点", len(self.points))
        for x, y, yaw in self.points:
            print("        (%.3f, %.3f, %.1f)," % (x, y, yaw), flush=True)

    def run(self):
        # roslaunch 启动时 stdin 往往不是终端, 这时只启用 RViz 点击采集。
        if not sys.stdin.isatty():
            rospy.loginfo("stdin 不是终端 → 只启用 RViz Publish Point 采集, Ctrl-C 退出")
            rospy.spin()
            self._print_summary()
            return

        while not rospy.is_shutdown():
            try:
                line = input()  # 阻塞等回车; 回调在别的线程, 点击照常工作
            except EOFError:
                rospy.loginfo("stdin 已关闭 → 切换为只接收 RViz 点击")
                rospy.spin()
                break
            if line.strip().lower() == 'q':
                break
            try:
                x, y, yaw = self.current_pose()
            except Exception as exc:
                rospy.logwarn("取当前位姿失败: %s", exc)
                continue
            self.emit(x, y, yaw, "robot")

        self._print_summary()


if __name__ == '__main__':
    rospy.init_node('record_points', anonymous=False)
    try:
        Recorder().run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
