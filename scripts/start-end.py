import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def read_trajectory_from_file(file_path):
    """
    从文件中读取数据，提取每行的第2-3个数作为XYZ坐标。
    :param file_path: 文件路径
    :return: 包含所有轨迹点的NumPy数组，形状为 (N, 3)
    """
    trajectory = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 3:  # 确保每行至少有3个数
                x = float(parts[1])  # 第2个数
                y = float(parts[2])  # 第3个数
                z = 0  # 如果没有Z坐标，默认为0
                if len(parts) >= 4:  # 如果有第4个数，作为Z坐标
                    z = float(parts[3])
                trajectory.append([x, y, z])
    return np.array(trajectory)

def plot_trajectory(trajectory):
    """
    绘制轨迹，并在第一个点和最后一个点标记红蓝点，局部放大这些位置。
    :param trajectory: 轨迹点，形状为 (N, 3)
    """
    if trajectory.size == 0:
        print("没有读取到轨迹数据！")
        return

    # 创建3D图形
    fig = plt.figure(figsize=(12, 6))

    # 绘制完整轨迹
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label='trajectory', color='gray', linewidth=1)
    ax1.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], color='red', s=100, label='Start point')
    ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], color='blue', s=100, label='End point')
    ax1.set_xlabel('X(m)')
    ax1.set_ylabel('Y(m)')
    ax1.set_zlabel('Z(m)')
    # ax1.set_title('完整轨迹')
    ax1.legend()

    # 局部放大起点和终点
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='gray', linewidth=1)
    ax2.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], color='red', s=100, label='Sart point')
    ax2.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], color='blue', s=100, label='End point')

    # 设置局部放大范围
    start_point = trajectory[0]
    end_point = trajectory[-1]
    margin = 1.0  # 放大范围的边距
    ax2.set_xlim([min(start_point[0], end_point[0]) - margin, max(start_point[0], end_point[0]) + margin])
    ax2.set_ylim([min(start_point[1], end_point[1]) - margin, max(start_point[1], end_point[1]) + margin])
    ax2.set_zlim([min(start_point[2], end_point[2]) - margin, max(start_point[2], end_point[2]) + margin])
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title('local set big')
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # 文件路径
    file_path = '/home/hyq/slam/lvio/src/LICO-mid360-main/data/hhh/LICO_xihu.txt'  # 替换为你的文件路径

    # 读取轨迹数据
    trajectory = read_trajectory_from_file(file_path)

    # 绘制轨迹
    plot_trajectory(trajectory)