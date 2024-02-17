#!/usr/bin/env python3

import sys
import threading

from log import log_data

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy
from inter_robot_communication.msg import ImageUUID

# Store timestamps (for plotting) and latencies (for latency calculations)
time_stamps = []
latencies = []


# Callback function for the subscriber
def receipt_callback(msg, args):
    current_time = rospy.Time.now()
    robot_namespace, received_path = args

    # Record received
    record = {
        "uuid": msg.uuid,
        "timestamp": current_time.to_nsec(),
        "recipient": robot_namespace,
    }
    log_data(received_path, robot_namespace, record)

    time_stamps.append(msg.header.stamp.to_nsec())
    sent_time = msg.header.stamp.to_sec()
    latency = current_time.to_sec() - sent_time

    latencies.append(latency)


def sub_orchestrator(robot_namespace, num_robots, topology, received_path):
    rospy.init_node("sub_orchestrator", anonymous=True)
    # rospy.loginfo(f"topology is set to {topology}")

    if topology == "a":
        robot_namespaces = [f"tb3_{i}" for i in range(num_robots)]
        other_robot_namespaces = [
            ns for ns in robot_namespaces if ns != robot_namespace
        ]

        for other_robot_namespace in other_robot_namespaces:
            # Subscriber for the other robots' measurements
            rospy.Subscriber(
                f"/{other_robot_namespace}/data",
                ImageUUID,
                receipt_callback,
                callback_args=(robot_namespace, received_path),
            )
    elif topology == "l":
        # rospy.loginfo("leader topology")
        rospy.Subscriber(
            "/leader/data",
            ImageUUID,
            receipt_callback,
            callback_args=(robot_namespace, received_path),
        )
    else:
        raise ValueError(
            f"Invalid topology '{topology}'. Expected 'a' for all-to-all or 'l' for leader."
        )


def start_plotting(robot_namespace):
    plt.figure()
    ani = FuncAnimation(
        plt.gcf(), lambda frame: update_plot(frame, robot_namespace), interval=100
    )
    plt.show()


def update_plot(frame, robot_namespace):
    if time_stamps and latencies:
        plt.cla()  # Clear the current axes

        # Convert time_stamps from nanoseconds to seconds for plotting
        time_stamps_sec = [ts / 1e9 for ts in time_stamps[-50:]]  # Last 50 time stamps

        plt.plot(time_stamps_sec, latencies[-50:], label="Latency")  # Plot the last 50 latencies
        plt.xlabel("Time (s)")
        plt.ylabel("Latency (s)")
        plt.title(f"Real-time Latency Plot for {robot_namespace}")
        plt.legend()
        plt.tight_layout()

        # Calculate aggregate statistics for the last 50 latencies
        mean_latency = np.mean(latencies[-50:])
        median_latency = np.median(latencies[-50:])
        max_latency = np.max(latencies[-50:])
        min_latency = np.min(latencies[-50:])
        std_latency = np.std(latencies[-50:])
        n = len(latencies)

        # Position for the stats text; adjust as needed
        text_x = time_stamps_sec[0]
        text_y = max(latencies[-50:]) * 0.8  # Position text at 80% of the max latency

        # Format the stats text
        stats_text = f'Mean: {mean_latency:.3f}s\nMedian: {median_latency:.3f}s\nMax: {max_latency:.3f}s\nMin: {min_latency:.3f}s\nStd: {std_latency:.3f}s\nn: {n}'

        plt.text(text_x, text_y, stats_text, fontsize=9, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))


if __name__ == "__main__":
    try:
        robot_namespace = rospy.myargv(argv=sys.argv)[1]
        num_robots = int(rospy.myargv(argv=sys.argv)[2])
        topology = rospy.myargv(argv=sys.argv)[3]
        received_path = rospy.myargv(argv=sys.argv)[4]
        graph = rospy.myargv(argv=sys.argv)[5]

        # Turn to boolean
        graph = graph.lower() == "true"

        if graph:
            plotting_thread = threading.Thread(target=start_plotting, args=(robot_namespace,))
            plotting_thread.start()

        sub_orchestrator(robot_namespace, num_robots, topology, received_path)

        # This will keep the current thread (main or plotting) alive and process ROS callbacks.
        # keep this thread active so that this thread can continuously receive and process messages from sub'd topics
        rospy.spin()

        # After rospy.spin() returns (which happens after a shutdown signal like Ctrl+C),
        # join the plotting thread to ensure it has finished before the program completely exits.
        if graph:
            plotting_thread.join()


    except rospy.ROSInterruptException:
        pass
    except IndexError:
        rospy.logerr("Usage: script.py robot_namespace num_robots")
