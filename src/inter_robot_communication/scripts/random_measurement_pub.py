#!/usr/bin/env python3

import rospy
from inter_robot_communication.msg import MeasurementStamped
import random
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# store timestamps (for plotting) and latencies (for latency calculations)
time_stamps = []
latencies = []

# Callback function for the subscriber
def measurement_callback(msg, robot_namespace):
    rospy.loginfo(f"{robot_namespace} Received measurement: {msg.measurement} at Time: {msg.header.stamp.to_sec()}")
    # Append data to global lists
    time_stamps.append(msg.header.stamp.to_sec())
    current_time = rospy.Time.now().to_sec()
    sent_time = msg.header.stamp.to_sec()
    latency = current_time - sent_time

    latencies.append(latency)
    rospy.loginfo("Latency: {:.3f} seconds".format(latency))

def random_measurement_publisher(robot_namespace):
    rospy.init_node("random_measurement_publisher", anonymous=True)
    
    # Publisher for measurements
    pub = rospy.Publisher("random_measurement", MeasurementStamped, queue_size=10)
    
    # Determine the other robot's namespace
    other_robot_namespace = "tb3_0" if robot_namespace == "tb3_1" else "tb3_1"
    
    # Subscriber for the other robot's measurements
    rospy.Subscriber(f"/{other_robot_namespace}/random_measurement", MeasurementStamped, measurement_callback, robot_namespace)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = MeasurementStamped()
        msg.measurement = random.random()
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

        rospy.loginfo(f"{robot_namespace} Publishing: {msg.measurement} at Time: {msg.header.stamp.to_sec()}")

        rate.sleep()

def update_plot(frame, robot_namespace):
    if time_stamps and latencies:
        # Clear the current plot
        plt.cla()
        # Plot the new data
        plt.plot(time_stamps[-50:], latencies[-50:], label="Latency")
        plt.xlabel('Time (s)')
        plt.ylabel('Latency (s)')
        # Update plot title with the robot's namespace
        plt.title(f'Real-time Latency Plot for {robot_namespace} Sent Measurements')
        plt.legend()
        plt.tight_layout()

        # Calculate and display the average latency of the last 50 data points
        if len(latencies) > 50:
            avg_latency = sum(latencies[-50:]) / 50
        else:
            avg_latency = sum(latencies) / len(latencies)
        plt.figtext(0.15, 0.85, f'Avg Latency (last 50): {avg_latency:.3f} s', fontsize=9, bbox={"facecolor": "white", "alpha": 0.5, "pad": 5})

def start_plotting(robot_namespace):
    plt.figure()
    ani = FuncAnimation(plt.gcf(), lambda frame: update_plot(frame, robot_namespace), interval=1000)
    plt.show()

if __name__ == "__main__":
    try:
        robot_namespace = rospy.myargv(argv=sys.argv)[1]

        # Start the plotting in a separate thread, passing the robot namespace
        plotting_thread = threading.Thread(target=start_plotting, args=(robot_namespace,))
        plotting_thread.start()

        # Run the ROS node in the main thread
        random_measurement_publisher(robot_namespace)

        plotting_thread.join()

    except rospy.ROSInterruptException:
        pass
    except IndexError:
        rospy.logerr("Usage: random_measurement_pub.py robot_namespace")
