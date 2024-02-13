#!/usr/bin/env python3

import sys
import uuid

from log import log_data

import rospy
from inter_robot_communication.msg import ImageUUID
from sensor_msgs.msg import Image
import numpy as np


def pub_orchestrator(robot_namespace, topology, sent_path):
    rospy.init_node("pub_orchestrator", anonymous=True)

    # Setting publish topic
    if topology == "a":
        # Publisher for measurements
        # Publish one's own topic
        # Add / in front of topic name to specify global (absolute) topic name
        pub = rospy.Publisher(f"/{robot_namespace}/data", ImageUUID, queue_size=10)
    elif topology == "l":
        pub = rospy.Publisher("/leader/data", ImageUUID, queue_size=10)
    else:
        raise ValueError(
            f"Invalid topology '{topology}'. Expected 'a' for all-to-all or 'l' for leader."
        )

    # After setting the topic above
    # Publish message

    bitrate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Generate a random bitmap as a numpy array
        bitmap = np.random.choice([0, 255], size=(20, 20), p=[0.5, 0.5]).astype(
            np.uint8
        )

        # need message ID to uniquely identify
        # Create the ImageUUID message
        # Create the nested Image message part of ImageUUID
        img = Image()
        # img.header.stamp = rospy.Time.now().to_sec()
        img.height = 20
        img.width = 20
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = 20
        img.data = bitmap.tobytes()

        # Create the ImageUUID message and set its fields
        msg = ImageUUID()
        msg.header.stamp = (
            rospy.Time.now()
        )  # Copy the timestamp to the outer header for consistency
        msg.image = img  # Set the Image message you just created
        # Generate and set the UUID here if needed
        msg.uuid = str(uuid.uuid4())

        pub.publish(msg)

        # Record sent
        record = {
            "uuid": msg.uuid,
            "timestamp": msg.header.stamp.to_sec(),
            "sender": robot_namespace,
            # "Recipient": "all" if topology == "a" else "leader",
        }
        log_data(sent_path, robot_namespace, record)

        # rospy.loginfo(
        #     f"{robot_namespace} Published an image at Time: {msg.header.stamp.to_sec()}"
        # )
        # rospy.loginfo(f"Topology is set to: {topology}")

        bitrate.sleep()


if __name__ == "__main__":
    try:
        robot_namespace = rospy.myargv(argv=sys.argv)[1]
        topology = rospy.myargv(argv=sys.argv)[2]
        sent_path = rospy.myargv(argv=sys.argv)[3]

        pub_orchestrator(robot_namespace, topology, sent_path)
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        rospy.logerr("Usage: script.py robot_namespace num_robots")
