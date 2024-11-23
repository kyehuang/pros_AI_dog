"""
This is the file for showing the log of the training process.
"""
import re
import argparse
import pandas as pd
import matplotlib.pyplot as plt


class ShowLog:
    """
    This class is used to show the log of the training process.
    """
    def __init__(self, file_path : str, num_rounds : int):
        self.file_path = file_path
        self.log_data = None
        self.num_rounds = num_rounds

    def read_log(self):
        """
        Read the log file.
        """
        with open(self.file_path, 'r', encoding="utf-8") as file:
            self.log_data = file.read()

    def extract_rewards(self):
        """
        Extract rewards from the log data.
        """
        rewards = [float(r)
                   for r in re.findall(r"Total episode reward: (-?\d+\.\d+)", self.log_data)]
        return rewards

    def group_rewards(self, data_frame):
        """
        Group rewards by rounds.
        """
        data_frame['Round'] = (data_frame['Episode'] - 1) // self.num_rounds + 1
        round_mean = data_frame.groupby('Round')['Total Reward'].mean().reset_index()
        return round_mean

    def plot_rewards(self, round_mean):
        """
        Plot the rewards.
        """
        plt.figure(figsize=(10, 6))
        plt.plot(round_mean['Round'], round_mean['Total Reward'],
                        marker='o', linestyle='-', color='b')
        plt.title(f"Average Total Rewards Over Rounds ({self.num_rounds} Episodes per Round)")
        plt.xlabel(f"Round ({self.num_rounds} Episodes per Round)")
        plt.ylabel("Average Total Reward")
        plt.grid(True)
        plt.savefig('result/average_episode_rewards_per_round.png')
        print("Plot saved as 'average_episode_rewards_per_round.png'")

    def plot_std(self, data_frame):
        """
        Plot the standard deviation of the rewards.
        """
        round_std = data_frame.groupby('Round')['Total Reward'].std().reset_index()
        plt.figure(figsize=(10, 6))
        plt.plot(round_std['Round'], round_std['Total Reward'],
                        marker='o', linestyle='-', color='g')
        plt.title(f"Standard Deviation of Total Rewards Over Rounds ({self.num_rounds} Episodes per Round)")
        plt.xlabel(f"Round ({self.num_rounds} Episodes per Round)")
        plt.ylabel("Standard Deviation of Total Reward")
        plt.grid(True)
        plt.savefig('result/std_episode_rewards_per_round.png')
        print("Plot saved as 'std_episode_rewards_per_round.png'")

    def plot_moving_average(self, data_frame):
        """
        Plot the moving average of the rewards.
        """
        data_frame['Moving Average'] = data_frame['Total Reward'].rolling(window=self.num_rounds).mean()
        plt.figure(figsize=(10, 6))
        plt.plot(data_frame['Episode'], data_frame['Total Reward'],
                    label='Total Reward', color='b', alpha=0.5)
        plt.plot(data_frame['Episode'], data_frame['Moving Average'],
                    label=f'Moving Average ({self.num_rounds} Episodes)', color='r')
        plt.title(f"Total Rewards with Moving Average ({self.num_rounds} Episodes)")
        plt.xlabel("Episode")
        plt.ylabel("Total Reward")
        plt.grid(True)
        plt.legend()
        plt.savefig('result/total_rewards_moving_average.png')
        print("Plot saved as 'total_rewards_moving_average.png'")

    @staticmethod
    def create_df(rewards):
        """
        Create a DataFrame from the rewards.
        """
        data_frame = pd.DataFrame(rewards, columns=["Total Reward"])
        data_frame["Episode"] = data_frame.index + 1
        return data_frame

    @staticmethod
    def plot_cumulative_rewards(data_frame):
        """
        Plot the cumulative rewards.
        """
        data_frame['Cumulative Reward'] = data_frame['Total Reward'].cumsum()

        plt.figure(figsize=(10, 6))
        plt.plot(data_frame['Episode'], data_frame['Cumulative Reward'],
                 marker='o', linestyle='-', color='orange')
        plt.title("Cumulative Total Rewards Over Episodes")
        plt.xlabel("Episode")
        plt.ylabel("Cumulative Total Reward")
        plt.grid(True)
        plt.savefig('result/cumulative_rewards.png')
        print("Plot saved as 'cumulative_rewards.png'")


    def show_log(self):
        """
        Show the log of the training process.
        """
        self.read_log()
        rewards = self.extract_rewards()
        data_frame = self.create_df(rewards)
        round_mean = self.group_rewards(data_frame)
        self.plot_rewards(round_mean)
        self.plot_std(data_frame)
        self.plot_moving_average(data_frame)
        self.plot_cumulative_rewards(data_frame)

if __name__ == "__main__":
    file_path = 'episode_results.log'

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--file_path", type=str, default=file_path)
    parser.add_argument("-r", type=int, default=20)

    # Get arguments
    args = parser.parse_args()
    file_path = args.file_path
    num_rounds = args.r

    # Show log
    show_log = ShowLog(file_path, num_rounds)
    show_log.show_log()
