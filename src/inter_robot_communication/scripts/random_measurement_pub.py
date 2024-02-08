#! /usr/bin/env python3

import rospy
from inter_robot_communication.msg import (
    MeasurementStamped,
)
import random
import sys


def random_measurement_publisher(robot_namespace):
    pub = rospy.Publisher(
        robot_namespace + "/random_measurement", MeasurementStamped, queue_size=10
    )
    rospy.init_node(robot_namespace + "_random_measurement_publisher", anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = MeasurementStamped()
        msg.header.stamp = rospy.Time.now()
        msg.measurement = random.random()

        rospy.loginfo(
            f"Publishing: {msg.measurement} at Time: {msg.header.stamp.to_sec()}"
        )
        pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            rospy.logerr("Usage: random_measurement_pub.py robot_namespace")
        else:
            robot_namespace = sys.argv[1]
            random_measurement_publisher(robot_namespace)
    except rospy.ROSInterruptException:
        pass
