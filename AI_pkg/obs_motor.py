"""
Time   : 2024/12/25 15:00
Author : Kye Huang
"""
import threading
from collections import deque
import roslibpy
import matplotlib.pyplot as plt


class MotorAngleSubscriber:
    """
    Show the motor angles of the Spot robot in real-time.
    """
    def __init__(self, ros_client: roslibpy.Ros, topic_name: str) -> None:
        self.client = ros_client
        self.topic = roslibpy.Topic(self.client, topic_name, 'std_msgs/Float32MultiArray')
        self.angles = deque(maxlen=1000)  # 儲存最近100筆數據
        self.lock = threading.Lock()

    def listener_callback(self, message: dict) -> None:
        """
        Callback function for the subscriber.
        """
        with self.lock:
            self.angles.append(message['data'])

    def start_listening(self) -> None:
        """
        Start listening to the topic.
        """
        self.topic.subscribe(self.listener_callback)

    def stop_listening(self) -> None:
        """
        Stop listening to the topic.
        """
        self.topic.unsubscribe()

    def get_angles(self) -> list:
        """
        Get the motor angles.
        """
        with self.lock:
            return list(self.angles)

def plot_angles(subscriber: MotorAngleSubscriber) -> None:
    """
    Plot the motor angles of the Spot robot in real-time.
    """
    plt.ion()
    fig, axs = plt.subplots(4, 1, figsize=(10, 10))  # 4 個子圖，1 列
    fig.tight_layout(pad=3.0)

    try:
        while True:
            angles = subscriber.get_angles()
            if angles:
                groups = [(0, 3), (3, 6), (6, 9), (9, 12)]  # 分組範圍
                for idx, (start, end) in enumerate(groups):
                    axs[idx].cla()  # 清除子圖內容
                    for i in range(start, end):
                        axs[idx].plot([angle[i] for angle in angles], label=f'Motor {i}')
                    axs[idx].set_title(f'Leg {idx + 1}')
                    axs[idx].legend()
                plt.pause(0.005)
    except KeyboardInterrupt:
        pass

def main():
    """
    Main function.
    """
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    subscriber = MotorAngleSubscriber(client, '/spot_motor_state')
    subscriber.start_listening()

    # 啟動繪圖的主線程
    plot_thread = threading.Thread(target=plot_angles, args=(subscriber,))
    plot_thread.start()

    try:
        print('Press Ctrl+C to stop...')
        plot_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.stop_listening()
        client.terminate()

if __name__ == '__main__':
    main()
