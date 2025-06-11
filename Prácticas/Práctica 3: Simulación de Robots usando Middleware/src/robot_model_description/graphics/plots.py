import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

bag_directory = "/home/aalbeerto-02/mysdr_ws/src/robot_model_description/graphics/rosbag2_2025_06_11-16_10_52"
arm_joint_names = [
    "arm_column_link_joint",
    "arm_1_link_joint",
    "arm_2_link_joint",
    "arm_gripper_link_joint",
    "gripper_left_link_joint",
    "gripper_right_link_joint"
]
wheel_names = ["wheel_up_left_link_joint", "wheel_down_left_link_joint", 
                "wheel_up_right_link_joint", "wheel_down_right_link_joint"]

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_directory, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)
joint_states_arm = []
joint_states_wheels = []
imu_data = []

while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic == '/joint_states':
        joint_state_msg = deserialize_message(data, get_message('sensor_msgs/msg/JointState'))
        filtered_efforts_arm = []
        for an in arm_joint_names:
            if an in joint_state_msg.name:
                i = joint_state_msg.name.index(an)
                filtered_efforts_arm.append(abs(joint_state_msg.effort[i]))
            else:
                filtered_efforts_arm.append(0.0)
        joint_states_arm.append((t / 1e9, filtered_efforts_arm))
        filtered_positions_wheels = []
        for wn in wheel_names:
            if wn in joint_state_msg.name:
                i = joint_state_msg.name.index(wn)
                filtered_positions_wheels.append(joint_state_msg.position[i])
            else:
                filtered_positions_wheels.append(np.nan)
        joint_states_wheels.append((t / 1e9, filtered_positions_wheels))
    elif topic == '/imu/data':
        imu_msg = deserialize_message(data, get_message('sensor_msgs/msg/Imu'))
        acc = imu_msg.linear_acceleration
        imu_data.append((t / 1e9, acc.x, acc.y, acc.z))

# GRÁFICA 1: TIEMPO VS G-PARCIAL
df_arm = pd.DataFrame(joint_states_arm, columns=['time', 'partial_gain'])
efforts_arm = pd.DataFrame(df_arm['partial_gain'].tolist(), columns=arm_joint_names)
df_arm = pd.concat([df_arm['time'], efforts_arm], axis=1)
df_arm['time'] = df_arm['time'] - df_arm['time'].iloc[0]
plt.figure(figsize=(12, 8))
for joint in arm_joint_names:
    plt.plot(df_arm['time'], df_arm[joint], label=f'{joint}')
plt.xlabel('TIEMPO')
plt.ylabel('G-PARCIAL')
plt.title('GRÁFICA 1: TIEMPO VS G-PARCIAL')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# GRÁFICA 2: TIEMPO VS POSICIÓN DE CADA UNA DE LAS RUEDAS
df_wheels = pd.DataFrame(joint_states_wheels, columns=['time', 'wheels_position'])
positions_wheels = pd.DataFrame(df_wheels['wheels_position'].tolist(), columns=wheel_names)
df_wheels = pd.concat([df_wheels['time'], positions_wheels], axis=1)
df_wheels['time'] = df_wheels['time'] - df_wheels['time'].iloc[0]
plt.figure(figsize=(12, 8))
for wheel in wheel_names:
    plt.plot(df_wheels['time'], df_wheels[wheel], label=f'{wheel}')
plt.xlabel('TIEMPO')
plt.ylabel('POSICIÓN DE CADA UNA DE LAS RUEDAS')
plt.title('GRÁFICA 2: TIEMPO VS POSICIÓN DE CADA UNA DE LAS RUEDAS')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# GRÁFICA 3: TIEMPO VS ACELERACIÓN DE LAS RUEDAS
df_imu = pd.DataFrame(imu_data, columns=['time', 'x_acceleration', 'y_acceleration', 'z_acceleration'])
df_imu['time'] = df_imu['time'] - df_imu['time'].iloc[0]
plt.figure(figsize=(12, 8))
plt.plot(df_imu['time'], df_imu['x_acceleration'], label='X Axis Wheels Acceleration')
plt.plot(df_imu['time'], df_imu['y_acceleration'], label='Y Axis Wheels Acceleration')
plt.plot(df_imu['time'], df_imu['z_acceleration'], label='Z Axis Wheels Acceleration')
plt.xlabel('TIEMPO')
plt.ylabel('ACELERACIÓN DE LAS RUEDAS')
plt.title('GRÁFICA 3: TIEMPO VS ACELERACIÓN DE LAS RUEDAS')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
