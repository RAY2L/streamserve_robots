import os
import pandas as pd
import threading


# Create a global lock
# Should it be inside log_data or outside it?
log_lock = threading.Lock()


def log_data(path, robot_id, record):
    # rospy.loginfo(f"file path before: {path}")
    # Need to expand the user's home directory
    expanded_path = os.path.expanduser("~/streamserve_robots")
    new_base_path = os.path.abspath(os.path.join(expanded_path, path))

    # Construct the full file path with the new base path
    file_path = os.path.join(new_base_path, f"robot_{robot_id}.csv")

    # rospy.loginfo(f"file path after: {file_path}")

    global log_lock
    with log_lock:
        # Check if the file is empty to write headers
        write_header = not os.path.exists(file_path) or os.stat(file_path).st_size == 0
        # Append the data to the CSV file
        pd.DataFrame([record]).to_csv(
            file_path, mode="a", header=write_header, index=False
        )
