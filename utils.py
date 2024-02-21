from datetime import datetime
import os
import json

def initialize_logging_dirs(args):
    num_robots = args.num_robots
    
    base_path = "logs/" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    sent_path = os.path.join(base_path, "sent")
    received_path = os.path.join(base_path, "received")
    config_path = os.path.join(base_path, "config.json")

    # Create directories
    os.makedirs(sent_path, exist_ok=True)
    os.makedirs(received_path, exist_ok=True)

    # Initialize CSV files for each robot
    for i in range(num_robots):
        open(os.path.join(sent_path, f"robot_tb3_{i}.csv"), "w").close()
        open(os.path.join(received_path, f"robot_tb3_{i}.csv"), "w").close()

    # Write args to a config file
    with open(config_path, "w") as config_file:
        # Convert args to a dictionary and serialize to JSON
        args_dict = vars(args)
        json.dump(args_dict, config_file, indent=4)

    return base_path, sent_path, received_path