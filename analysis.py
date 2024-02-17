import argparse
import glob
import os

import pandas as pd
import matplotlib.pyplot as plt


def generate_dataframes(csv_files):
    dfs = [0] * len(csv_files)
    for i, f in enumerate(csv_files):
        df = pd.read_csv(f)
        df["timestamp"] = df["timestamp"] / 1e9
        dfs[i] = df
    return dfs


def get_files(base_path):
    sent_path = os.path.join(base_path, "sent")
    rcvd_path = os.path.join(base_path, "received")
    sent_csv_files = glob.glob(sent_path + "/*.csv")
    rcvd_csv_files = glob.glob(rcvd_path + "/*.csv")
    return sent_csv_files, rcvd_csv_files


def analysis_orchestrator(logs_path, plot_graphs):
    sent_csv_files, rcvd_csv_files = get_files(logs_path)
    sent_dfs, rcvd_dfs = generate_dataframes(sent_csv_files), generate_dataframes(rcvd_csv_files)

    sent_dfs = pd.concat(sent_dfs)

    # Initialize an empty list to store aggregate statistics dictionaries
    aggregate_stats_data = []

    for idx, rcvd_df in enumerate(rcvd_dfs):
        joined_df = pd.merge(sent_dfs, rcvd_df, on="uuid", how="inner")
        df_sorted = joined_df.sort_values(by="timestamp_x")
        latency = df_sorted["timestamp_y"] - df_sorted["timestamp_x"]

        # Calculate statistics
        mean_latency = latency.mean()
        median_latency = latency.median()
        max_latency = latency.max()
        min_latency = latency.min()
        std_latency = latency.std()
        q1_latency = latency.quantile(0.25)
        q3_latency = latency.quantile(0.75)
        p99_latency = latency.quantile(0.99)
        n = latency.count()

        # Append a dictionary of statistics to the list
        aggregate_stats_data.append({
            'recipient': df_sorted['recipient'].iloc[0],
            'mean': mean_latency,
            'median': median_latency,
            'max': max_latency,
            'min': min_latency,
            'std': std_latency,
            'q1': q1_latency,
            'q3': q3_latency,
            'n': n,
            'p99': p99_latency,
        })

        if plot_graphs:
            # Plot latency
            plt.figure(idx)
            plt.plot(df_sorted["timestamp_x"].values, latency.values, label=f"Sender: {df_sorted['sender'].iloc[0]}", marker="o")
            plt.xlabel("Timestamp X")
            plt.ylabel("Latency (Timestamp Y - Timestamp X)")
            plt.title(f'Latency by Timestamp for Sender: {df_sorted["sender"].iloc[0]}')
            plt.legend()
            plt.grid(True)

    if plot_graphs:
        plt.show()

    # Convert the list of dictionaries to a DataFrame
    aggregate_stats = pd.DataFrame(aggregate_stats_data).sort_values(by="recipient")

    # Print aggregate statistics without the index column
    print(aggregate_stats.to_string(index=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run offline analysis on logs data")
    parser.add_argument("--logs_path", type=str, help="Logs data path")
    parser.add_argument("--plot", action="store_true", help="Plot the latency graphs (default: False)")
    args = parser.parse_args()

    if args.logs_path:
        analysis_orchestrator(args.logs_path, args.plot)
    else:
        print("No logs path provided.")
