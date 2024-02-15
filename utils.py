from datetime import datetime
import os


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
