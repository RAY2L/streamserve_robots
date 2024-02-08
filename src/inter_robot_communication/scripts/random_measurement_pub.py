#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import random
import sys

def random_measurement_publisher(robot_namespace):
    pub = rospy.Publisher(robot_namespace + '/random_measurement', Float64, queue_size=10)
    rospy.init_node(robot_namespace + '_random_measurement_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz aka 1 measurements/sec
    while not rospy.is_shutdown():
        # Generate random float as measurement
        random_measurement = random.random()
        rospy.loginfo(random_measurement)
        pub.publish(random_measurement)
        rate.sleep()

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:  # Check if the namespace argument is provided
            rospy.logerr("Usage: random_measurement_pub.py robot_namespace")
        else:
            robot_namespace = sys.argv[1]  # Get the namespace from command line arguments
            random_measurement_publisher(robot_namespace)
    except rospy.ROSInterruptException:
        pass
