import glob
import os

import pandas as pd
import matplotlib.pyplot as plt


# given a list of csv files, generate a list of dataframes for those files
def generate_dataframes(csv_files):
    dfs = [0] * len(csv_files)

    for i, f in enumerate(csv_files):
        df = pd.read_csv(f)

        dfs[i] = df

    return dfs


# def combine_send():


def get_files(base_path):
    sent_path = os.path.join(base_path, "sent")
    rcvd_path = os.path.join(base_path, "received")

    sent_csv_files = glob.glob(sent_path + "/*.csv")
    rcvd_csv_files = glob.glob(rcvd_path + "/*.csv")

    return sent_csv_files, rcvd_csv_files


def analysis_orchestrator(logs_path):
    sent_csv_files, rcvd_csv_files = get_files(logs_path)
    sent_dfs, rcvd_dfs = generate_dataframes(sent_csv_files), generate_dataframes(
        rcvd_csv_files
    )

    sent_dfs = pd.concat(sent_dfs)

    for idx, rcvd_df in enumerate(rcvd_dfs):
        joined_df = pd.merge(sent_dfs, rcvd_df, on="uuid", how="inner")

        df_sorted = joined_df.sort_values(by="timestamp_x")
        latency = df_sorted["timestamp_y"] - df_sorted["timestamp_x"]

        # Create a new figure for each plot
        plt.figure(idx)
        plt.plot(
            df_sorted["timestamp_x"].values,
            latency.values,
            label=f"Sender: {df_sorted['sender'].iloc[0]}",
            marker="o",
        )

        # Set up labels, title, and legend for the current figure
        plt.xlabel("Timestamp X")
        plt.ylabel("Latency (Timestamp Y - Timestamp X)")
        plt.title(f'Latency by Timestamp for Sender: {df_sorted["sender"].iloc[0]}')
        plt.legend()
        plt.grid(True)

    # Show all figures at once after the loop
    plt.show()


if __name__ == "__main__":
    analysis_orchestrator("logs/2024-02-14_02-29-51")
