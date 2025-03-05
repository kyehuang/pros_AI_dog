#!/usr/bin/env python3
"""
Example script: Publish /spot_states (std_msgs/Float32MultiArray)
and subscribe /spot_actions (trajectory_msgs/JointTrajectoryPoint).

Logs the time interval (delta) between consecutive callbacks.
"""

import time
import roslibpy
import numpy as np
import matplotlib.pyplot as plt

# 全域變數 (不建議在大型專案大量使用，但小範例可接受)
ITERATION_COUNT = 0


def callback_spot_actions(message, last_callback_time):
    """
    Subscriber Callback for /spot_actions topic.
    Logs time delta since last callback was invoked.
    """
    global ITERATION_COUNT
    current_time = time.time()
    delta = current_time - last_callback_time[-1]
    print(f'#{ITERATION_COUNT} 花費時間: {delta:.4f} 秒')
    last_callback_time.append(current_time)
    ITERATION_COUNT += 1

def show_plot_and_statistics(time_elapsed):
    """
    Show scatter plot of time_elapsed and print statistics.
    """
    x_values = range(len(time_elapsed))

    plt.scatter(x_values, time_elapsed, c='blue')
    plt.title("Scatter Plot of Independent Times")
    plt.xlabel("Index")
    plt.ylabel("Time Value")
    plt.grid(True)
    plt.show()

    te_array = np.array(time_elapsed)

    # 計算平均值與標準差
    mean = np.mean(te_array)
    std = np.std(te_array)

    # 設定要排除離群值的閾值（z_score_threshold）
    z_score_threshold = 3  # 通常可設定 2、2.5、3 或你想要的其他範圍

    # 留下 |x - mean| / std < z_score_threshold 的資料
    filtered_time_elapsed = [
        x for x in te_array
        if abs(x - mean) / std < z_score_threshold
    ]
    print(f"average: {np.mean(filtered_time_elapsed):.4f}")
def main():
    """
    Main function:
    1) Connects to ROS Bridge
    2) Publishes data to /spot_states
    3) Subscribes to /spot_actions
    4) Prints time delta between consecutive subscriber callbacks
    """
    # 1. 連線至 ROS Bridge
    ros = roslibpy.Ros(host='localhost', port=9090)
    ros.run()

    if ros.is_connected:
        print('連線成功！')
    else:
        print('連線失敗，請檢查 ROS Bridge 是否已啟動。')
        return

    # 2. 建立 Publisher
    publisher_spot_state = roslibpy.Topic(
        ros,
        '/spot_states',                   # 話題名稱
        'std_msgs/Float32MultiArray'      # 訊息型態
    )

    # 要發送的訊息（Float32MultiArray 需要 layout + data）
    spot_state_data = {
        'layout': {
            'dim': [
                {
                    'label': 'example_dimension',
                    'size': 15,
                    'stride': 15
                }
            ],
            'data_offset': 0
        },
        'data': [
            1.1,  2.2,  3.3,  4.4,  5.5,
            6.6,  7.7,  8.8,  9.9, 10.1,
            11.1, 12.2, 13.1, 14.1, 15.1
        ]
    }

    # 3. 建立 Subscriber
    subscriber_spot_actions = roslibpy.Topic(
        ros,
        '/spot_actions',
        'trajectory_msgs/JointTrajectoryPoint'
    )

    # 用 list 包裝 last_callback_time，讓 callback 能直接更新此參考
    last_callback_time = [time.time()]

    def on_spot_actions_received(msg):
        callback_spot_actions(msg, last_callback_time)

    subscriber_spot_actions.subscribe(on_spot_actions_received)

    # 4. 開始發布訊息
    print('開始發布 float array...')
    publisher_spot_state.advertise()

    try:
        for _ in range(100_000):
            publisher_spot_state.publish(roslibpy.Message(spot_state_data))
            time.sleep(0.005)
    except KeyboardInterrupt:
        print('收到中斷指令，準備結束...')

    time_elapsed = [last_callback_time[i] - last_callback_time[i - 1]
                    for i in range(10, len(last_callback_time))]
    show_plot_and_statistics(time_elapsed)
    # 5. 關閉連線
    subscriber_spot_actions.unsubscribe()
    publisher_spot_state.unadvertise()
    ros.close()
    print('已關閉連線.')


if __name__ == '__main__':
    main()
