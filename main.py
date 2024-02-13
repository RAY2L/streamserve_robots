import argparse
import roslaunch

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Launch multiple robots in ROS.')
parser.add_argument('--num_robots', type=int, default=1, help='Number of robots to spawn')
parser.add_argument('--topology', type=int, default=1, help='(a) all-to-all or (l) leader')
args = parser.parse_args()

# Initialize ROS launch
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# Base CLI arguments for environment and other setups
base_cli_args = [
    ['turtlebot3_simulations', 'environment.launch']
    # Add other base setups here
]

# Dynamically construct CLI arguments for each robot
robot_cli_args = [
    ['turtlebot3_simulations', 'spawn_turtlebot.launch', f'robot_id:={i}', f'num_robots:={args.num_robots}']
    for i in range(args.num_robots)
]

# Combine all CLI arguments
cli_args = base_cli_args + robot_cli_args

# Resolve launch arguments and prepare launch files and arguments
launch_files = []
for args in cli_args:
    roslaunch_file_list = roslaunch.rlutil.resolve_launch_arguments(args[:2])
    roslaunch_args = args[2:]
    # Ensure we have at least one launch file resolved, and take the first one
    if roslaunch_file_list:
        roslaunch_file = roslaunch_file_list[0]  # Take the first file in the list
        launch_files.append((roslaunch_file, roslaunch_args))


# Start ROS launch
parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
parent.start()

try:
    # Keep the script alive until the user interrupts with Ctrl+C
    parent.spin()
finally:
    # Shutdown all launched nodes and cleanup
    parent.shutdown()
