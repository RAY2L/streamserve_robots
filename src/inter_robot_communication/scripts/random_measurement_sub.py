#!/usr/bin/env python3
import rospy
from inter_robot_communication.msg import MeasurementStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Store timestamps (as float) and latencies
latencies_tb3_0 = []
latencies_tb3_1 = []


def callback_tb3_0(data):
    current_time = rospy.Time.now().to_sec()
    sent_time = data.header.stamp.to_sec()
    latency = current_time - sent_time

    latencies_tb3_0.append(latency)
    rospy.loginfo("TB3_0 Latency: {:.3f} seconds".format(latency))


def callback_tb3_1(data):
    current_time = rospy.Time.now().to_sec()
    sent_time = data.header.stamp.to_sec()
    latency = current_time - sent_time

    latencies_tb3_1.append(latency)
    rospy.loginfo("TB3_1 Latency: {:.3f} seconds".format(latency))


def update_plot(frame):
    plt.cla()
    if latencies_tb3_0:
        plt.plot(latencies_tb3_0, label="TB3_0 Latency")
    if latencies_tb3_1:
        plt.plot(latencies_tb3_1, label="TB3_1 Latency")

    # Average latencies
    avg_latency_tb3_0 = (
        sum(latencies_tb3_0) / len(latencies_tb3_0) if latencies_tb3_0 else 0
    )
    avg_latency_tb3_1 = (
        sum(latencies_tb3_1) / len(latencies_tb3_1) if latencies_tb3_1 else 0
    )

    # Annotate the plot with average latency values
    plt.text(
        0.5,
        0.95,
        f"Average Latency TB3_0: {avg_latency_tb3_0:.6f} seconds",
        transform=plt.gca().transAxes,
        ha="center",
        color="blue",
    )
    plt.text(
        0.5,
        0.90,
        f"Average Latency TB3_1: {avg_latency_tb3_1:.6f} seconds",
        transform=plt.gca().transAxes,
        ha="center",
        color="orange",
    )

    plt.xlabel("Message Count")
    plt.ylabel("Latency (seconds)")
    plt.title("Inter-Robot Communication Latency")
    plt.legend()


def listener():
    rospy.init_node("tb3_listener", anonymous=True)
    rospy.Subscriber("/tb3_0/random_measurement", MeasurementStamped, callback_tb3_0)
    rospy.Subscriber("/tb3_1/random_measurement", MeasurementStamped, callback_tb3_1)

    plt.ion()
    # real-time rendering of communication latency graph
    ani = FuncAnimation(plt.gcf(), update_plot, interval=1000)  # Update every 1000 ms

    plt.show(
        block=True
    )  # block=True makes sure the script waits until the plot is closed


if __name__ == "__main__":
    listener()
