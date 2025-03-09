#!/usr/bin/env python3

import rospy
import rosbag
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

# 读取 bag 文件中的 IMU 数据
def read_imu_from_bag(bag_file, topic_name):
    global start_time

    # 打开 bag 文件
    bag = rosbag.Bag(bag_file)

    # 遍历 bag 文件中的消息
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
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

        # # 实时更新图形
        # update_plot()

    # 关闭 bag 文件
    bag.close()

# 更新图形
def update_plot():
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

# 主函数
def main():
    # 初始化 ROS 节点
    rospy.init_node("imu_plotter")

    # 创建图形
    plt.figure(figsize=(12, 8))
    
    # 读取 bag 文件中的 IMU 数据
    bag_file = "/mnt/g/datasets/se_2.bag"  # 替换为你的 bag 文件路径
    topic_name = "/livox/imu"  # 替换为你的 IMU 话题名称
    read_imu_from_bag(bag_file, topic_name)

    # 显示图形
     # 绘制线性加速度曲线
    plt.plot(time_data, ax_data, color="r", label="ax")
    # plt.plot(time_data, ay_data, color="g", label="ay")
    # plt.plot(time_data, az_data, color="b", label="az")

    # # 绘制角速度曲线
    # plt.plot(time_data, wx_data, color="c", label="wx")
    # plt.plot(time_data, wy_data, color="m", label="wy")
    # plt.plot(time_data, wz_data, color="y", label="wz")
    # plt.tick_params(labelsize=23)
    # labels = ax.get_xticklabels() + ax.get_yticklabels()
    # [label.set_fontname('Times New Roman') for label in labels]
    plt.xticks(fontsize=20)  # 调整 x 轴刻度标签的字体大小
    plt.yticks(fontsize=20)  # 调整 y 轴刻度标签的字体大小

    #设置横纵坐标的名称以及对应字体格式
    font2 = {'family' : 'Times New Roman',
    'weight' : 'normal',
    'size'   : 20,
    }

    # 设置图形标题和标签
    plt.title("IMU X direction",font2)
    plt.xlabel("Time (s)",font2)
    plt.ylabel("Value (m/s^2)",font2)
    plt.legend(loc="upper right",fontsize=20)

    # 设置横轴范围（从 0 开始）
    if len(time_data) > 0:
        plt.xlim(0, time_data[-1])  # 横轴从 0 开始，到最新时间戳结束
    plt.show()

if __name__ == "__main__":
    main()