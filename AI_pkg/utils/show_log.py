import re
import pandas as pd
import matplotlib.pyplot as plt
import argparse

class ShowLog:
    def __init__(self, file_path : str, round : int):
        self.file_path = file_path
        self.log_data = None
        self.round = round

    def read_log(self):
        with open(self.file_path, 'r') as file:
            self.log_data = file.read()

    def extract_rewards(self):
        rewards = [float(r) for r in re.findall(r"Total episode reward: (-?\d+\.\d+)", self.log_data)]
        return rewards
    
    def create_df(self, rewards):
        df = pd.DataFrame(rewards, columns=["Total Reward"])
        df["Episode"] = df.index + 1
        return df

    def group_rewards(self, df):
        df['Round'] = (df['Episode'] - 1) // self.round + 1
        round_mean = df.groupby('Round')['Total Reward'].mean().reset_index()
        return round_mean
    
    def plot_rewards(self, round_mean):
        plt.figure(figsize=(10, 6))
        plt.plot(round_mean['Round'], round_mean['Total Reward'], marker='o', linestyle='-', color='b')
        plt.title(f"Average Total Rewards Over Rounds ({self.round} Episodes per Round)")        
        plt.xlabel(f"Round ({self.round} Episodes per Round)")        
        plt.ylabel("Average Total Reward")
        plt.grid(True)
        plt.savefig('result/average_episode_rewards_per_round.png')
        print("Plot saved as 'average_episode_rewards_per_round.png'")
    
    def plot_std(self, df):        
        round_std = df.groupby('Round')['Total Reward'].std().reset_index()
        plt.figure(figsize=(10, 6))
        plt.plot(round_std['Round'], round_std['Total Reward'], marker='o', linestyle='-', color='g')        
        plt.title(f"Standard Deviation of Total Rewards Over Rounds ({self.round} Episodes per Round)")                
        plt.xlabel(f"Round ({self.round} Episodes per Round)")
        plt.ylabel("Standard Deviation of Total Reward")
        plt.grid(True)
        plt.savefig('result/std_episode_rewards_per_round.png')
        print("Plot saved as 'std_episode_rewards_per_round.png'")

    def plot_moving_average(self, df):
        df['Moving Average'] = df['Total Reward'].rolling(window=self.round).mean()
        plt.figure(figsize=(10, 6))
        plt.plot(df['Episode'], df['Total Reward'], label='Total Reward', color='b', alpha=0.5)
        plt.plot(df['Episode'], df['Moving Average'], label=f'Moving Average ({self.round} Episodes)', color='r')
        plt.title(f"Total Rewards with Moving Average ({self.round} Episodes)")
        plt.xlabel("Episode")
        plt.ylabel("Total Reward")
        plt.grid(True)
        plt.legend()
        plt.savefig('result/total_rewards_moving_average.png')
        print(f"Plot saved as 'total_rewards_moving_average.png'")

    def plot_cumulative_rewards(self, df):
        # 计算累积奖励
        df['Cumulative Reward'] = df['Total Reward'].cumsum()

        # 绘制累积奖励图
        plt.figure(figsize=(10, 6))
        plt.plot(df['Episode'], df['Cumulative Reward'], marker='o', linestyle='-', color='orange')
        plt.title("Cumulative Total Rewards Over Episodes")
        plt.xlabel("Episode")
        plt.ylabel("Cumulative Total Reward")
        plt.grid(True)
        plt.savefig('result/cumulative_rewards.png')
        print("Plot saved as 'cumulative_rewards.png'")


    def show_log(self):
        self.read_log()
        rewards = self.extract_rewards()
        df = self.create_df(rewards)
        round_mean = self.group_rewards(df)
        self.plot_rewards(round_mean)
        self.plot_std(df)
        self.plot_moving_average(df)
        self.plot_cumulative_rewards(df)

if __name__ == "__main__":
    file_path = 'episode_results.log'
    
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--file_path", type=str, default=file_path)
    parser.add_argument("-r", type=int, default=20)
    
    # Get arguments
    args = parser.parse_args()
    file_path = args.file_path
    round = args.r

    # Show log
    show_log = ShowLog(file_path, round)
    show_log.show_log()