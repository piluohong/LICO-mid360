import matplotlib.pyplot as plt
import matplotlib
import numpy as np

# 设置字体为支持中文的字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 设置字体为 SimHei
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 文件路径
file_path = "/home/hyq/slam/lvio/src/LICO-mid360-main/data/备份/kntsNum.txt"

# 读取文件数据
with open(file_path, "r") as file:
    data = [int(line.strip()) for line in file]

# 创建图形
plt.figure(figsize=(12, 7))  # 增加图形宽度

# 绘制线段
for i in range(len(data) - 1):
    x_values = [i, (i + 1)]  # 增加横轴间隔（乘以2）
    y_values = [data[i], data[i + 1]]  # 当前点和下一个点的纵坐标
    plt.plot(x_values, y_values, color="black", linewidth=2)  # 统一使用黑色线条

# 设置图形标题和标签
plt.title("Adaptive knots number")  # 中文标题
plt.yticks(np.arange(1, 5, 1),fontsize=16)  # 设置 y 轴刻度单位为 1，范围为 1 到 4
plt.xticks(fontsize=16)
plt.xlabel("Time(s)",fontsize=20)  # 中文横轴标签
plt.ylabel("Value",fontsize=20)  # 中文纵轴标签

# 显示图形
plt.show()