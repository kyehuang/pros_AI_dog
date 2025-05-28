import sys
import roslibpy
import threading
import copy
import time
import numpy as np
import json
import os

from PyQt6 import QtCore, QtGui, QtWidgets
from ui import Ui_RobotController
from PyQt6.QtWidgets import QApplication, QMainWindow

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from IK.DH import spot_state_creater
from IK.spot_leg import SpotLeg


class RobotControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_RobotController()
        self.ui.setupUi(self)

        # 綁定按鈕事件
        self.ui.pushButton.clicked.connect(self.handle_connect)
        self.ui.pushButton_2.clicked.connect(self.handle_move)
        self.ui.clear.clicked.connect(self.handle_clear_button)
        self.ui.add_list.clicked.connect(self.add_item)
        self.ui.delete_list.clicked.connect(self.delete_item)
        self.ui.run_list.clicked.connect(self.run_list)
        self.ui.save_btn.clicked.connect(self.save_data_to_file)
        self.ui.load_btn.clicked.connect(self.load_data_from_file)
        self.ui.load_list_2.clicked.connect(self.load_data_from_list)

        # 初始化 ROSBridge 客戶端
        self.ros = None
        self.move_topic = None
        self.move_topic_name = '/spot_actions'

        # Initialize the joint lengths and angles
        self.joint_lengths = [0.0801, 0.1501, 0.1451]
        self.angles = [0, 0, 0]
        self.spot_leg = SpotLeg(self.joint_lengths, self.angles)

        # Initialize the base position and rotation
        self.base_position = [0, 0, 0.20]
        self.base_rotation = [0, 0, 0]
        self.base_tilt = [0, 0]
        self.base_translation = [0.3740, 0.1670, 0]
        self.leg_end_position = [
            [0.0, 0.0, 0],
            [0.0, 0.0, 0],
            [0.0, 0.0, 0],
            [0.0, 0.0, 0]
        ]
        self.data = []

        self.action = self.data_arrange(spot_state_creater(
            self.spot_leg,
            self.base_position,
            self.base_rotation,
            self.base_translation,
            self.base_tilt,
            self.leg_end_position
        ))

    def handle_connect(self):
        # print(self.ros.is_connected)
        def connect_thread():
            try:
                self.ros.run()
                if self.ros.is_connected:
                    self.set_status("Connected", "green")
                    self.move_topic = roslibpy.Topic(
                        self.ros, self.move_topic_name, 'trajectory_msgs/JointTrajectoryPoint'
                    )
                else:
                    self.set_status("Failed to connect", "red")
            except Exception as e:
                self.set_status(f"Error: {e}", "red")

        if self.ros and self.ros.is_connected:
            print("[INFO] 已經連接到 ROSBridge")
            self.set_status("Already connected", "green")
            return

        ip_port = self.ui.lineEdit.text().strip()
        if ':' not in ip_port:
            self.set_status("Invalid IP:Port", "orange")
            return

        ip, port = ip_port.split(':')
        self.ros = roslibpy.Ros(host=ip, port=int(port))
        threading.Thread(target=connect_thread).start()
        self.set_status("Connecting...", "blue")

    def handle_move(self):
        if not self.ros or not self.ros.is_connected:
            self.set_status("Not connected", "red")
            return

        if not self.move_topic:
            self.set_status("Publisher not ready", "orange")
            return

        # 更新目前變數
        self.update_variable()

        # 計算關節角度
        angle = spot_state_creater(
            self.spot_leg,
            self.base_position,
            self.base_rotation,
            self.base_translation,
            self.base_tilt,
            self.leg_end_position
        )

        new_action = self.data_arrange(angle)

        # 傳送差值的控制訊號
        self.send_joint_angle_trajectory(self.action, new_action)

        # 更新動作
        self.action = new_action

    def handle_clear_button(self):
        # 清除所有輸入框的內容
        self.ui.doubleSpinBox.setValue(0.0)
        self.ui.doubleSpinBox_2.setValue(0.0)
        self.ui.doubleSpinBox_3.setValue(0.2)
        self.ui.doubleSpinBox_4.setValue(0.0)
        self.ui.doubleSpinBox_5.setValue(0.0)
        self.ui.doubleSpinBox_6.setValue(0.0)
        self.ui.doubleSpinBox_7.setValue(0.0)
        self.ui.doubleSpinBox_8.setValue(0.0)
        self.ui.doubleSpinBox_9.setValue(0.0)
        self.ui.doubleSpinBox_10.setValue(0.0)
        self.ui.doubleSpinBox_11.setValue(0.0)
        self.ui.doubleSpinBox_12.setValue(0.0)
        self.ui.doubleSpinBox_13.setValue(0.0)
        self.ui.doubleSpinBox_14.setValue(0.0)
        self.ui.doubleSpinBox_15.setValue(0.0)
        self.ui.doubleSpinBox_16.setValue(0.0)
        self.ui.doubleSpinBox_17.setValue(0.0)
        self.ui.doubleSpinBox_18.setValue(0.0)
        self.ui.doubleSpinBox_19.setValue(0.0)
        self.ui.doubleSpinBox_20.setValue(0.0)

    def set_status(self, text, color):
        self.ui.label_10.setText(text)
        self.ui.label_10.setStyleSheet(f"color: {color};")

    def data_arrange(self, spot_actions: list) -> list:
        """
        Arrange the data for spot actions
        Args:
            spot_actions (list): List of spot actions
        Returns:
            list: List of arranged spot actions
        """
        actions_copy = copy.deepcopy(spot_actions)

        # LF, RB : first joint
        actions_copy[0] = 90 + actions_copy[0]  # LF
        actions_copy[6] = 90 - actions_copy[6]  # RB
        # LF, RB : second joint
        actions_copy[1] = actions_copy[1]       # LF
        actions_copy[7] = 180 - actions_copy[7] # RB
        # LF, RB : third joint
        actions_copy[2] = 180 - actions_copy[2] # LF
        actions_copy[8] = actions_copy[8]       # RB

        # RF, LB : first joint
        actions_copy[3] = 90 + actions_copy[3]
        actions_copy[9] = 90 - actions_copy[9]
        # RF, LB : second joint
        actions_copy[4]  = 180 -actions_copy[4]
        actions_copy[10] = actions_copy[10]
        # RF, LB : third joint
        actions_copy[5]  = actions_copy[5]
        actions_copy[11] = 180 - actions_copy[11]

        # turn deg to rad
        actions_copy = [self.__deg_to_rad(action) for action in actions_copy]

        return actions_copy

    def update_variable(self):
        """
        Update the base position, rotation, tilt, and leg end positions
        based on the values from the UI.
        """
        # 讀取 Base Position
        self.base_position[0] = float(self.ui.doubleSpinBox.text())
        self.base_position[1] = float(self.ui.doubleSpinBox_2.text())
        self.base_position[2] = float(self.ui.doubleSpinBox_3.text())

        # 讀取 Base Rotation
        self.base_rotation[0] = float(self.ui.doubleSpinBox_4.text())
        self.base_rotation[1] = float(self.ui.doubleSpinBox_5.text())
        self.base_rotation[2] = float(self.ui.doubleSpinBox_6.text())

        # 讀取 Base Tilt
        self.base_tilt[0] = float(self.ui.doubleSpinBox_7.text())
        self.base_tilt[1] = float(self.ui.doubleSpinBox_8.text())

        self.leg_end_position[0][0] = float(self.ui.doubleSpinBox_9.text())
        self.leg_end_position[0][1] = float(self.ui.doubleSpinBox_10.text())
        self.leg_end_position[0][2] = float(self.ui.doubleSpinBox_11.text())
        self.leg_end_position[1][0] = float(self.ui.doubleSpinBox_12.text())
        self.leg_end_position[1][1] = float(self.ui.doubleSpinBox_13.text())
        self.leg_end_position[1][2] = float(self.ui.doubleSpinBox_14.text())
        self.leg_end_position[2][0] = float(self.ui.doubleSpinBox_15.text())
        self.leg_end_position[2][1] = float(self.ui.doubleSpinBox_16.text())
        self.leg_end_position[2][2] = float(self.ui.doubleSpinBox_17.text())
        self.leg_end_position[3][0] = float(self.ui.doubleSpinBox_18.text())
        self.leg_end_position[3][1] = float(self.ui.doubleSpinBox_19.text())
        self.leg_end_position[3][2] = float(self.ui.doubleSpinBox_20.text())

    @staticmethod
    def __deg_to_rad(deg: float) -> float:
        """
        Convert degrees to radians.

        Args:
            deg (float): Angle in degrees.

        Returns:
            float: Angle in radians.
        """
        return deg * 0.01745329252

    def add_item(self):
        print("add item")
        self.update_variable()
        print("data:", self.data)
        try:
            values = {
                "base_position": copy.deepcopy(self.base_position),
                "base_rotation": copy.deepcopy(self.base_rotation),
                "base_tilt": copy.deepcopy(self.base_tilt),
                "leg_end_position": copy.deepcopy(self.leg_end_position)
            }
            self.data.append(values)
            self.ui.listWidget.addItem(f"參數組 {len(self.data)}: {values}")

        except Exception as e:
            print(f"Error adding item: {e}")
            self.set_status("Failed to add item", "red")

    def delete_item(self):
        row = self.ui.listWidget.currentRow()
        if row >= 0:
            self.ui.listWidget.takeItem(row)  # 從列表中刪除
            try:
                del self.data[row]  # 從資料中刪除
                print(f"已刪除第 {row} 筆資料")
            except IndexError:
                print("資料索引錯誤，無法刪除")
        else:
            print("請先選取要刪除的項目")
            self.set_status("未選取任何項目", "orange")
        print("data:", self.data)

    def send_joint_angle_trajectory(self, start_action, target_action, step=10, delay=0.1):
        for i in range(1, step + 1):
            t = i / (step + 1)
            interpolated = [
                (1 - t) * start_action[j] + t * target_action[j]
                for j in range(len(target_action))
            ]
            self.publish_joint_angles(interpolated)
            time.sleep(delay)

        # 傳送最後一筆資料，確保精準達標
        self.publish_joint_angles(target_action)

    def publish_joint_angles(self, angles):
        msg = roslibpy.Message({
            'positions': angles,
            'velocities': [0.0] * len(angles),
            'accelerations': [0.0] * len(angles),
            'effort': [0.0] * len(angles),
            'time_from_start': {'secs': 1, 'nsecs': 0}
        })
        self.move_topic.publish(msg)
        self.set_status("Move command sent", "green")

    def run_list(self):
        if not self.data:
            print("No data to run")
            return
        angle = []

        for item in self.data:
            base_position = item["base_position"]
            base_rotation = item["base_rotation"]
            base_tilt = item["base_tilt"]
            leg_end_position = item["leg_end_position"]
            angle.append(self.data_arrange(
                spot_state_creater(
                    self.spot_leg,
                    base_position,
                    base_rotation,
                    self.base_translation,
                    base_tilt,
                    leg_end_position
                )
            ))
        # Send the trajectory
        for i in range(len(angle) - 1):
            self.send_joint_angle_trajectory(angle[i], angle[i + 1], step=10, delay=0.1)

    def save_data_to_file(self):
        file_path = "data.json"
        print(f"[SAVE] 儲存 {file_path}")
        print(f"data: {self.data}")
        try:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(self.data, f, indent=2)
            self.set_status(f"資料已儲存到 {file_path}", "green")
            print(f"[SAVE] 儲存 {len(self.data)} 筆資料到 {file_path}")
        except Exception as e:
            print(f"[SAVE ERROR] {e}")
            self.set_status("儲存失敗", "red")

    def load_data_from_file(self):
        file_path="data.json"
        try:
            print(f"[LOAD] 載入 {file_path}")
            with open(file_path, "r", encoding="utf-8") as f:
                self.data = json.load(f)

            self.ui.listWidget.clear()
            for i, item in enumerate(self.data):
                self.ui.listWidget.addItem(f"參數組 {i+1}: {item}")

            self.set_status(f"已從 {file_path} 載入 {len(self.data)} 筆資料", "green")
            print(f"[LOAD] 載入 {len(self.data)} 筆資料")
        except Exception as e:
            print(f"[LOAD ERROR] {e}")
            self.set_status("載入失敗", "red")

    def load_data_from_list(self):
        row = self.ui.listWidget.currentRow()
        print(f"row: {row}")
        if row >= 0:
            item = self.data[row]
            self.base_position = item["base_position"]
            self.base_rotation = item["base_rotation"]
            self.base_tilt = item["base_tilt"]
            self.leg_end_position = item["leg_end_position"]
            print(f"載入參數組 {row + 1}: {item}")
            print(f"base_position: {self.base_position}")
            print(f"base_rotation: {self.base_rotation}")
            print(f"base_tilt: {self.base_tilt}")

            # 更新 UI
            self.ui.doubleSpinBox.setValue(self.base_position[0])
            self.ui.doubleSpinBox_2.setValue(self.base_position[1])
            self.ui.doubleSpinBox_3.setValue(self.base_position[2])
            self.ui.doubleSpinBox_4.setValue(self.base_rotation[0])
            self.ui.doubleSpinBox_5.setValue(self.base_rotation[1])
            self.ui.doubleSpinBox_6.setValue(self.base_rotation[2])
            self.ui.doubleSpinBox_7.setValue(self.base_tilt[0])
            self.ui.doubleSpinBox_8.setValue(self.base_tilt[1])
            # for i in range(4):
            #     self.ui.doubleSpinBox_9.setValue(item["leg_end_position"][i][0])
            #     self.ui.doubleSpinBox_10.setValue(item["leg_end_position"][i][1])
            #     self.ui.doubleSpinBox_11.setValue(item["leg_end_position"][i][2])

        else:
            print("請先選取要載入的項目")
            self.set_status("未選取任何項目", "orange")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlApp()
    window.show()
    sys.exit(app.exec())