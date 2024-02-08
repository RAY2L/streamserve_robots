#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def callback_tb3_0(data):
    rospy.loginfo("TB3_0 Received: %f" % data.data)

def callback_tb3_1(data):
    rospy.loginfo("TB3_1 Received: %f" % data.data)

def listener():
    rospy.init_node('tb3_listener', anonymous=True)
    rospy.Subscriber("/tb3_0/random_measurement", Float64, callback_tb3_0)
    rospy.Subscriber("/tb3_1/random_measurement", Float64, callback_tb3_1)
    rospy.spin()

if __name__ == '__main__':
    listener()
