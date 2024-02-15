import argparse

from utils import initialize_logging_dirs

import roslaunch


parser = argparse.ArgumentParser(description="Launch multiple robots in ROS.")
parser.add_argument("--launch", action="store_true")
parser.add_argument(
    "--num_robots", type=int, default=1, help="Number of robots to spawn"
)
parser.add_argument(
    "--topology",
    type=str,
    choices=["a", "l"],
    default="a",
    help="Topology: (a) all-to-all or (l) leader-follower",
)
parser.add_argument("--bitrate", type=int, default=1, help="Bitrate")
parser.add_argument(
    "--bitrate_random",
    action="store_true",
    help="Toggle random average bitrate",
)
parser.add_argument(
    "--resolution",
    type=int,
    nargs=2,
    default=[20, 20],
    help="Image resolution (width, height)",
)
parser.add_argument("--in_jpg", action="store_true", help="Compress images to jpeg")
parser.add_argument("--graph", action="store_true", help="Graph the logged data")
args = parser.parse_args()


if args.launch:
    base_path, sent_path, received_path = initialize_logging_dirs(args.num_robots)

    # Initialize ROS launch
    # Generate uuid for this launch session
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Base CLI arguments for environment and other setups
    base_cli_args = [
        ["turtlebot3_simulations", "environment.launch"]
        # Add other base setups here
    ]

    # Dynamically construct CLI arguments for each robot
    robot_cli_args = [
        [
            "turtlebot3_simulations",
            "spawn_turtlebot.launch",
            f"robot_id:={i}",
            f"num_robots:={args.num_robots}",
            f"topology:={args.topology}",
            f"bitrate:={args.bitrate}",
            f"bitrate_random:={args.bitrate_random}",
            f"resolution:={','.join(map(str, args.resolution))}",
            f"in_jpg:={args.in_jpg}",
            f"sent_path:={sent_path}",
            f"received_path:={received_path}",
            f"graph:={args.graph}",
        ]
        for i in range(args.num_robots)
    ]

    # Combine all CLI arguments
    cli_args = base_cli_args + robot_cli_args

    # Resolve launch arguments and prepare launch files and arguments
    launch_files = []
    for args in cli_args:
        # Resolve/determine the path using the first 2 arguments (package + launch file name)
        roslaunch_file_list = roslaunch.rlutil.resolve_launch_arguments(args[:2])
        roslaunch_args = args[2:]
        # Ensure we have at least one launch file resolved
        if roslaunch_file_list:
            roslaunch_file = roslaunch_file_list[
                0
            ]  # Take the first launch file in the list
            launch_files.append((roslaunch_file, roslaunch_args))

    # Start ROS launch
    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    # This method starts the launch process.
    # It goes through each item in the launch_files list, loads the corresponding launch file with its arguments,
    # and starts the nodes defined within those files.
    parent.start()

    try:
        # Keep the script alive (by running a loop) until the user interrupts with Ctrl+C
        parent.spin()
    finally:
        # Shutdown all launched nodes and cleanup
        parent.shutdown()
