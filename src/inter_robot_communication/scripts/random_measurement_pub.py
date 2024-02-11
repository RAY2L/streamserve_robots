#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from inter_robot_communication.msg import MeasurementStamped
import numpy as np
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# store timestamps (for plotting) and latencies (for latency calculations)
time_stamps = []
latencies = []
robot_namespaces = set(["tb3_0", "tb3_1", "tb3_2"])

# Callback function for the subscriber
def measurement_callback(msg, robot_namespace):
    rospy.loginfo(f"{robot_namespace} Received measurement at Time: {msg.header.stamp.to_sec()}")
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
    pub = rospy.Publisher("random_measurement", Image, queue_size=10)
    
    # # Determine the other robot's namespace
    # other_robot_namespace = "tb3_0" if robot_namespace == "tb3_1" else "tb3_1"

    other_robot_namespaces = robot_namespaces.copy()
    other_robot_namespaces.remove(robot_namespace)

    for other_robot_namespace in other_robot_namespaces:
        # Subscriber for the other robot's measurements
        rospy.Subscriber(f"/{other_robot_namespace}/random_measurement", Image, measurement_callback, robot_namespace)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Define bitmap dimensions
        width, height = 20, 20
        
        # Generate a random bitmap as a numpy array
        bitmap = np.random.choice([0, 255], size=(height, width), p=[0.5, 0.5]).astype(np.uint8)
        
        # Create the Image message
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = height
        msg.width = width
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = width
        msg.data = bitmap.tostring()
        pub.publish(msg)

        rospy.loginfo(f"{robot_namespace} Published an image at Time: {msg.header.stamp.to_sec()}")

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

        # Use numpy to calculate statistics for more efficiency, especially with large datasets
        latencies_array = np.array(latencies[-50:])  # Consider only the last 50 measurements for real-time analysis

        avg_latency = np.mean(latencies_array)
        med_latency = np.median(latencies_array)
        min_latency = np.min(latencies_array)
        max_latency = np.max(latencies_array)
        std_dev_latency = np.std(latencies_array)

        # Display the statistics on the plot
        plt.figtext(0.15, 0.85, f'Avg Latency: {avg_latency:.3f} s\nMed Latency: {med_latency:.3f} s\nMin Latency: {min_latency:.3f} s\nMax Latency: {max_latency:.3f} s\nStd Dev: {std_dev_latency:.3f} s', fontsize=9, bbox={"facecolor": "white", "alpha": 0.5, "pad": 5})

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
