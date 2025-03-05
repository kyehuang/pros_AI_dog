from utils.gym_manger import GymManager
from ros_receive_and_processing import ai_dog_node
from keyboard_control.keyboard_action import KeyboardAction
import time
import csv

import csv
import time
from typing import Any
import numpy as np
import matplotlib.pyplot as plt

import pandas as pd
import ast
import numpy as np
from datetime import datetime
import csv
from typing import Any

class StoreSpotState:
    """
    Context manager to store Spot states in a CSV file.
    """
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.file = None
        self.csv_writer = None

    def writerow(self, row: list) -> None:
        """
        Write a row to the CSV file.
        """
        if self.csv_writer is None:
            raise ValueError("CSV writer is not initialized. "
                             "Make sure to use this class as a context manager.")
        self.csv_writer.writerow(row)

    def __enter__(self) -> "StoreSpotState":
        """
        Enter the context and open the file for writing.
        """
        self.file = open(self.file_path, mode='w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.file)
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        """
        Exit the context and close the file.
        """
        if self.file:
            self.file.close()
            self.file = None




def step_init(node: ai_dog_node) -> None:
    """
    Initialize the AI_dog_node and the gym environment for testing.
    """
    for _, state in enumerate(KeyboardAction.FORWARD_STEP_INIT):
        node.publish_spot_actions(state)
        time.sleep(0.1)

def step_route(node: ai_dog_node, store_spot_state: StoreSpotState) -> None:
    """
    Route the dog to the target position and log data to CSV.
    """
    action = KeyboardAction.FORWARD_STEP_LEFT + KeyboardAction.FORWARD_STEP_RIGHT
    for _, state in enumerate(action):
        node_state_raw = node.get_latest_data()
        # print(node_state_raw)
        node_state = [node_state_raw["spot_states"][0], node_state_raw["spot_states"][1],
                      node_state_raw["spot_states"][2], node_state_raw["spot_states"][4]]
        node_state.extend(node_state_raw["motor_states"])
        # print(node_state)
        node.publish_spot_actions(state)
        time.sleep(0.1)
        # 添加动作到状态列表并写入 CSV
        node_state.append(state)
        store_spot_state.writerow(node_state)



# 使用方法
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
FILE_PATH = f"data/dog_route_data_{current_time}.csv"
store_spot_state = StoreSpotState(FILE_PATH)

node, ros_thread = GymManager().init_ai_dog_node()
try:
    time.sleep(1)
    node.reset_unity()
    time.sleep(1)
    step_init(node)
    with StoreSpotState(FILE_PATH) as store_spot_state:
        for _ in range(5):
            step_route(node, store_spot_state)
    print(f"[INFO] Saved Spot states to {FILE_PATH}")
    print("[INFO] Data collection is completed.")
finally:
    store_spot_state.__exit__(None, None, None)
    GymManager().shutdown_ai_dog_node(node, ros_thread)
