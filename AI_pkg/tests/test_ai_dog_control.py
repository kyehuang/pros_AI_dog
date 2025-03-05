"""
This module is used to get observation from AI_dog_node.
"""
import time
from utils.gym_manger import GymManager

def main():
    """
    Main function to test the AI_dog_node class.
    """
    # 建立並初始化 AI_dog_node
    manager = GymManager()
    node, ros_thread = manager.init_ai_dog_node()

    # 您想要發布的動作內容 (此處以範例數值為例)
    actions_to_publish = [
        0.0, 135.0, 90.0,
        0.0, 135.0, 90.0,
        0.0, 135.0, 90.0,
        0.0, 135.0, 90.0
    ]

    try:
        while True:
            # 1) 發布動作指令
            node.publish_spot_actions(actions_to_publish)

            # 2) 重置 latest_data，以確保取得更新後的資料
            node.reset_latest_data()

            # 3) 取得最新資料
            result = node.get_latest_data()
            # 可在此對 result 做判斷或處理
            # print(f"Got result: {result}")

            # 避免 while True 高速空轉，讓CPU爆載；可適度 sleep
            # time.sleep(0.1)

    except KeyboardInterrupt:
        print("收到中斷信號，準備結束程式...")

    finally:
        # 確保在程式結束時正確關閉 node 與對應執行緒
        manager.shutdown_ai_dog_node(node, ros_thread)
        print("AI_dog_node 已關閉.")

if __name__ == "__main__":
    main()
