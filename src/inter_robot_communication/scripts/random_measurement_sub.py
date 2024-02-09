#!/usr/bin/env python3
import rospy
from inter_robot_communication.msg import MeasurementStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys

# Store timestamps (as float) and latencies
latencies = []


def callback(data):
    current_time = rospy.Time.now().to_sec()
    sent_time = data.header.stamp.to_sec()
    latency = current_time - sent_time

    latencies.append(latency)
    rospy.loginfo("Latency: {:.3f} seconds".format(latency))


def update_plot(frame):
    plt.cla()
    if latencies:
        plt.plot(latencies, label="Latency")

    # Average latencies
    avg_latency = (
        sum(latencies) / len(latencies) if latencies else 0
    )

    # Annotate the plot with average latency values
    plt.text(
        0.5,
        0.95,
        f"Average Latency: {avg_latency:.6f} seconds",
        transform=plt.gca().transAxes,
        ha="center",
        color="blue",
    )

    plt.xlabel("Message Count")
    plt.ylabel("Latency (seconds)")
    plt.title("Inter-Robot Communication Latency")
    plt.legend()


def listener(other_namespace):
    rospy.Subscriber(f"/{other_namespace}/random_measurement", MeasurementStamped, callback)

    plt.ion()
    # real-time rendering of communication latency graph
    ani = FuncAnimation(plt.gcf(), update_plot, interval=1000)  # Update every 1000 ms

    plt.show(
        block=True
    )  # block=True makes sure the script waits until the plot is closed


if __name__ == "__main__":
    try:
        other_namespace = rospy.myargv(argv=sys.argv)[1]
        listener(other_namespace)
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        rospy.logerr("Usage: random_measurement_pub.py robot_namespace")
