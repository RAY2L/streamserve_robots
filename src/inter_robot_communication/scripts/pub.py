#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import numpy as np
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# Store timestamps (for plotting) and latencies (for latency calculations)
time_stamps = []
latencies = []

# Callback function for the subscriber
def measurement_callback(msg, robot_namespace):
    rospy.loginfo(f"{robot_namespace} Received measurement at Time: {msg.header.stamp.to_sec()}")
    # Append data to global lists
    time_stamps.append(msg.header.stamp.to_sec())
    current_time = rospy.Time.now().to_sec()
    sent_time = msg.header.stamp.to_sec()
    latency = current_time - sent_time

    latencies.append(latency)
    rospy.loginfo(f"Latency: {latency:.3f} seconds")

def pub_all_to_all(robot_namespace, num_robots):
    rospy.init_node("pub_all_to_all", anonymous=True)
    
    # Publisher for measurements
    pub = rospy.Publisher(f"/{robot_namespace}/data", Image, queue_size=10)

    robot_namespaces = [f"tb3_{i}" for i in range(int(num_robots))]
    other_robot_namespaces = [ns for ns in robot_namespaces if ns != robot_namespace]

    for other_robot_namespace in other_robot_namespaces:
        # Subscriber for the other robot's measurements
        rospy.Subscriber(f"/{other_robot_namespace}/data", Image, measurement_callback, robot_namespace)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Generate a random bitmap as a numpy array
        bitmap = np.random.choice([0, 255], size=(20, 20), p=[0.5, 0.5]).astype(np.uint8)
        
        # Create the Image message
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = 20
        msg.width = 20
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = 20
        msg.data = bitmap.tobytes()
        pub.publish(msg)

        rospy.loginfo(f"{robot_namespace} Published an image at Time: {msg.header.stamp.to_sec()}")

        rate.sleep()

def start_plotting(robot_namespace):
    plt.figure()
    ani = FuncAnimation(plt.gcf(), lambda frame: update_plot(frame, robot_namespace), interval=1000)
    plt.show()

def update_plot(frame, robot_namespace):
    if time_stamps and latencies:
        plt.cla()
        plt.plot(time_stamps[-50:], latencies[-50:], label="Latency")
        plt.xlabel('Time (s)')
        plt.ylabel('Latency (s)')
        plt.title(f'Real-time Latency Plot for {robot_namespace}')
        plt.legend()
        plt.tight_layout()

if __name__ == "__main__":
    try:
        robot_namespace = rospy.myargv(argv=sys.argv)[1]
        num_robots = rospy.myargv(argv=sys.argv)[2]

        plotting_thread = threading.Thread(target=start_plotting, args=(robot_namespace,))
        plotting_thread.start()

        pub_all_to_all(robot_namespace, num_robots)

        plotting_thread.join()

    except rospy.ROSInterruptException:
        pass
    except IndexError:
        rospy.logerr("Usage: script.py robot_namespace num_robots")
