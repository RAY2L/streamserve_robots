#!/usr/bin/env python3

import sys
import uuid
import io

from log import log_data

import rospy
from inter_robot_communication.msg import ImageUUID
from PIL import Image as PILImage
from sensor_msgs.msg import Image
import numpy as np


def pub_orchestrator(
    robot_namespace, topology, sent_path, bitrate, bitrate_random, resolution, in_jpg
):
    width, height = resolution

    rospy.init_node("pub_orchestrator", anonymous=True)

    # Setting publish topic
    if topology == "a":
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

    # bitrate = rospy.Rate(bitrate)  # 1 Hz

    while not rospy.is_shutdown():
        # Generate an all-black bitmap as a numpy array
        bitmap = np.zeros((height, width), dtype=np.uint8)

        if in_jpg:
            # Use PIL to create an image from the numpy array and compress it to JPEG
            pil_img = PILImage.fromarray(bitmap)
            buf = io.BytesIO()
            pil_img.save(buf, format="JPEG")
            jpeg_data = buf.getvalue()
            img.data = jpeg_data
            img.encoding = "jpeg"
            img.step = 0  # In JPEG compressed data, step is not used
        else:
            img = Image()
            img.height = height
            img.width = width
            img.encoding = "mono8"
            img.is_bigendian = 0
            img.step = width
            img.data = bitmap.tobytes()

        msg = ImageUUID()
        msg.header.stamp = rospy.Time.now()
        msg.image = img
        # Generate and set the UUID here
        # need uuid to uniquely identify message later on
        msg.uuid = str(uuid.uuid4())

        pub.publish(msg)

        # Record sent
        record = {
            "uuid": msg.uuid,
            "timestamp": msg.header.stamp.to_nsec(),
            "sender": robot_namespace,
        }
        log_data(sent_path, robot_namespace, record)

        # rospy.loginfo(
        #     f"{robot_namespace} Published an image at Time: {msg.header.stamp.to_sec()}"
        # )
        # rospy.loginfo(f"Topology is set to: {topology}")
        # rospy.loginfo(f"bitrate is set to: {bitrate}")
        # rospy.loginfo(f"resolution is set to: {resolution}")
        # rospy.loginfo(f"type(resolution) is set to: {type(resolution)}")

        if bitrate_random:
            # Use an exponential distribution if bitrate_random is True
            current_bitrate = np.random.exponential(bitrate)
            rate = rospy.Rate(current_bitrate)  # Using the randomly generated bitrate
        else:
            rate = rospy.Rate(bitrate)  # Using the fixed bitrate

        rate.sleep()


if __name__ == "__main__":
    try:
        robot_namespace = rospy.myargv(argv=sys.argv)[1]
        topology = rospy.myargv(argv=sys.argv)[2]
        sent_path = rospy.myargv(argv=sys.argv)[3]
        bitrate = int(rospy.myargv(argv=sys.argv)[4])
        bitrate_random = rospy.myargv(argv=sys.argv)[5]
        resolution = rospy.myargv(argv=sys.argv)[6]
        width, height = map(int, resolution.split(","))
        in_jpg = rospy.myargv(argv=sys.argv)[7]

        # Turn to boolean
        bitrate_random = bitrate_random.lower() == "true"
        in_jpg = in_jpg.lower() == "true"

        pub_orchestrator(
            robot_namespace,
            topology,
            sent_path,
            bitrate,
            bitrate_random,
            (width, height),
            in_jpg,
        )
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        rospy.logerr("Usage: script.py robot_namespace num_robots")
