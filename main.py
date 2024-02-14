import argparse
from datetime import datetime
import os

import roslaunch

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Launch multiple robots in ROS.")
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
args = parser.parse_args()


def initialize_logging_dirs(num_robots):
    base_path = "logs/" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    sent_path = os.path.join(base_path, "sent")
    received_path = os.path.join(base_path, "received")

    # Create directories
    os.makedirs(sent_path, exist_ok=True)
    os.makedirs(received_path, exist_ok=True)

    # Initialize CSV files for each robot
    for i in range(num_robots):
        open(os.path.join(sent_path, f"robot_tb3_{i}.csv"), "w").close()
        open(os.path.join(received_path, f"robot_tb3_{i}.csv"), "w").close()

    return base_path, sent_path, received_path


base_path, sent_path, received_path = initialize_logging_dirs(args.num_robots)

# Initialize ROS launch
# Generate uuid for
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
        f"sent_path:={sent_path}",
        f"received_path:={received_path}",
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