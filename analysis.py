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


if __name__ == "__main__":
    sent_csv_files, rcvd_csv_files = get_files("logs/2024-02-13_19-45-18")
    sent_dfs, rcvd_dfs = generate_dataframes(sent_csv_files), generate_dataframes(rcvd_csv_files)

    sent_dfs = pd.concat(sent_dfs)

    # print(sent_dfs.dtypes)
    # print(rcvd_dfs[0].dtypes)

    # # Ensure the 'uuid' column in sent_dfs is of the same type as in rcvd_dfs
    # if 'uuid' in sent_dfs.columns and 'uuid' in rcvd_dfs[0].columns:
    #     # Convert 'uuid' in sent_dfs to the same type as in the first rcvd_df
    #     sent_dfs['uuid'] = sent_dfs['uuid'].astype(rcvd_dfs[0]['uuid'].dtype)

    # joined_dfs = [0] * len(rcvd_dfs)

    for rcvd_df in rcvd_dfs:
        # Ensure 'uuid' in rcvd_df is of the same type as in sent_dfs before joining
        # rcvd_df['uuid'] = rcvd_df['uuid'].astype(sent_dfs['uuid'].dtype)

        # Now perform the join
        joined = pd.merge(sent_dfs, rcvd_df, on="uuid", how="inner")
        print(joined)

        df_sorted = joined.sort_values(by="timestamp_x")
        latency = df_sorted["timestamp_y"] - df_sorted["timestamp_x"]

        plt.plot(df_sorted['timestamp_x'].values, latency.values, label=f"Sender: {df_sorted['sender'].iloc[0]}", marker='o')

    # print(sent_csv_files, rcvd_csv_files)
    # print(sent_dfs, rcvd_dfs)