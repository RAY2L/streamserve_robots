#!/usr/bin/env python3

import sys
import threading

from log import log_data

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
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

    if topology == "a":
        robot_namespaces = [f"tb3_{i}" for i in range(int(num_robots))]
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
        plt.cla()
        plt.plot(time_stamps[-50:], latencies[-50:], label="Latency")
        plt.xlabel("Time (s)")
        plt.ylabel("Latency (s)")
        plt.title(f"Real-time Latency Plot for {robot_namespace}")
        plt.legend()
        plt.tight_layout()


if __name__ == "__main__":
    try:
        robot_namespace = rospy.myargv(argv=sys.argv)[1]
        num_robots = rospy.myargv(argv=sys.argv)[2]
        topology = rospy.myargv(argv=sys.argv)[3]
        received_path = rospy.myargv(argv=sys.argv)[4]
        graph = rospy.myargv(argv=sys.argv)[5]
        # Turn graph to boolean
        graph = graph.lower() == "true"

        if graph:
            plotting_thread = threading.Thread(
                target=start_plotting, args=(robot_namespace,)
            )
            plotting_thread.start()

            sub_orchestrator(robot_namespace, num_robots, topology, received_path)

            plotting_thread.join()
        else:
            sub_orchestrator(robot_namespace, num_robots, topology, received_path)

    except rospy.ROSInterruptException:
        pass
    except IndexError:
        rospy.logerr("Usage: script.py robot_namespace num_robots")
