import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取 kntsNum.txt 文件
def read_kntsnum(file_path):
    with open(file_path, "r") as file:
        data = [int(line.strip()) for line in file]
    return data

# 读取 TUM 格式的轨迹文件
def read_tum(file_path):
    with open(file_path, "r") as file:
        data = [list(map(float, line.strip().split())) for line in file]
    return np.array(data)

# 文件路径
kntsnum_file = "/home/hyq/slam/lvio/src/LICO-mid360-main/data/adaptive_kntsNum_copy_1.txt"
tum_file = "/home/hyq/slam/lvio/src/LICO-mid360-main/data/hhh/LICO_hilly_orchard_1.txt"

# 读取文件
kntsnum_data = read_kntsnum(kntsnum_file)
tum_data = read_tum(tum_file)

# 以行数较小的文件为基准
min_length = min(len(kntsnum_data), len(tum_data))
kntsnum_data = kntsnum_data[:min_length]
tum_data = tum_data[:min_length]

# 定义颜色映射
color_map = {
    1: "blue",
    2: "green",
    3: "yellow",
    4: "red"
}

# 创建 3D 图形
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# 提取 TUM 数据中的 X, Y, Z 坐标
x = tum_data[:, 1]
y = tum_data[:, 2]
z = tum_data[:, 3]

# 按顺序绘制线条（黑色细线）
ax.plot(x, y, z, color="black", linewidth=2, alpha=0.5, label="Traj")

# 绘制散点（颜色根据 kntsnum 的值设置）
for i in range(min_length):
    color = color_map.get(kntsnum_data[i], "black")  # 根据 kntsnum 的值选择颜色
    ax.scatter(x[i], y[i], z[i], color=color, s=25, alpha=1.0)  # 调整点的大小

# 设置图形标题和标签
# ax.set_title("Traj")
ax.tick_params(axis="x", labelsize=12)
ax.tick_params(axis="y", labelsize=12)
ax.tick_params(axis="z", labelsize=12)
ax.set_xlabel("X(m)",fontsize=14)
ax.set_ylabel("Y(m)",fontsize=14)
ax.set_zlabel("Z(m)",fontsize=14)

# 添加图例
legend_elements = [
    plt.Line2D([0], [0], marker="o", color="w", label="1: blue", markersize=10, markerfacecolor="blue"),
    plt.Line2D([0], [0], marker="o", color="w", label="2: green", markersize=10, markerfacecolor="green"),
    plt.Line2D([0], [0], marker="o", color="w", label="3: yellow", markersize=10, markerfacecolor="yellow"),
    plt.Line2D([0], [0], marker="o", color="w", label="4: red", markersize=10, markerfacecolor="red")
]
ax.legend(handles=legend_elements, loc="lower center", bbox_to_anchor=(0.5, -0.15), ncol=4)

# 显示图形
plt.tight_layout()
plt.show()