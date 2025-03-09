#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque

# 全局变量存储数据
start_time = None  # 记录起始时间
time_data = []  # 时间戳（从 0 开始）
ax_data = []    # 线性加速度 x
ay_data = []    # 线性加速度 y
az_data = []    # 线性加速度 z
wx_data = []    # 角速度 x
wy_data = []    # 角速度 y
wz_data = []    # 角速度 z

# IMU 回调函数
def imu_callback(msg):
    global start_time

    # 获取当前时间戳
    current_time = msg.header.stamp.to_sec()

    # 如果是第一次收到数据，记录起始时间
    if start_time is None:
        start_time = current_time

    # 计算相对于起始时间的时间戳
    relative_time = current_time - start_time

    # 获取线性加速度
    ax = msg.linear_acceleration.x * 9.8
    ay = msg.linear_acceleration.y * 9.8
    az = msg.linear_acceleration.z * 9.8

    # 获取角速度
    wx = msg.angular_velocity.x
    wy = msg.angular_velocity.y
    wz = msg.angular_velocity.z

    # 存储数据
    time_data.append(relative_time)
    ax_data.append(ax)
    ay_data.append(ay)
    az_data.append(az)
    wx_data.append(wx)
    wy_data.append(wy)
    wz_data.append(wz)

# 初始化 ROS 节点
rospy.init_node("imu_plotter")

# 订阅 IMU 话题
rospy.Subscriber("/livox/imu", Imu, imu_callback)

# 创建图形
plt.figure(figsize=(12, 8))

# 实时更新图形
while not rospy.is_shutdown():
    plt.clf()  # 清空当前图形

    # 绘制线性加速度曲线
    plt.plot(time_data, ax_data, color="r", label="ax")
    # plt.plot(time_data, ay_data, color="g", label="ay")
    # plt.plot(time_data, az_data, color="b", label="az")

    # # 绘制角速度曲线
    # plt.plot(time_data, wx_data, color="c", label="wx")
    # plt.plot(time_data, wy_data, color="m", label="wy")
    # plt.plot(time_data, wz_data, color="y", label="wz")

    # 设置图形标题和标签
    plt.title("IMU X direction")
    plt.xlabel("Time (s)")
    plt.ylabel("Value (m/s^2)")
    plt.legend(loc="upper right")

    # 设置横轴范围（从 0 开始）
    if len(time_data) > 0:
        plt.xlim(0, time_data[-1])  # 横轴从 0 开始，到最新时间戳结束

    # 刷新图形
    plt.pause(0.01)

# 显示图形（如果 ROS 退出）
plt.show()