#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Relay robot_pose_ekf's fused pose as nav_msgs/Odometry for visualization.

This adapter only receives a pose-with-covariance message, so it cannot infer
real velocity. Keep navigation controllers on /wheel_odom, which contains the
base driver's measured twist.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class OdomEKF:
    def __init__(self):
        rospy.init_node("odom_ekf", anonymous=False)

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        # robot_pose_ekf estimates the pose of base_footprint.
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.output_topic = rospy.resolve_name("output")
        self.input_topic = rospy.resolve_name("input")

        self.ekf_pub = rospy.Publisher("output", Odometry, queue_size=10)
        self.sub = rospy.Subscriber(
            "input", PoseWithCovarianceStamped, self.pub_ekf_odom, queue_size=10
        )
        rospy.loginfo(
            "Republishing fused pose %s -> %s (pose-only; use /wheel_odom for measured twist)",
            self.input_topic,
            self.output_topic,
        )

    def pub_ekf_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        if not odom.header.frame_id:
            odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose = msg.pose

        # This input contains no velocity estimate. Keep the zero-valued twist
        # explicitly marked as unknown rather than falsely certain (zero covariance).
        unknown_variance = 1.0e6
        for i in (0, 1, 2, 3, 4, 5):
            odom.twist.covariance[i * 6 + i] = unknown_variance

        self.ekf_pub.publish(odom)


def main():
    OdomEKF()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
