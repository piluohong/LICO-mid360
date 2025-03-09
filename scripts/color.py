import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap, Normalize

# 自定义颜色映射：从红色到紫色，中间有平滑过渡
colors = [
    (1, 0, 0),      # 红色 (R=1, G=0, B=0)
    (1, 1, 0),      # 黄色 (R=1, G=1, B=0)
    (0, 1, 0),      # 绿色 (R=0, G=1, B=0)
    (0, 1, 1),      # 青色 (R=0, G=1, B=1)
    (0, 0, 1),      # 蓝色 (R=0, G=0, B=1)
    (0.5, 0, 0.5),  # 紫色 (R=0.5, G=0, B=0.5)
]
cmap = LinearSegmentedColormap.from_list("custom_rainbow", colors, N=256)

# 定义值范围
vmin, vmax = 2, 10  # 范围从 -1 到 3

# 创建一个从 -1 到 4 的渐变条（垂直方向），确保 3 以上都是紫色
gradient = np.linspace(vmin, vmax + 1, 256).reshape(-1, 1)  # 垂直方向的渐变

# 创建图像
fig, ax = plt.subplots(figsize=(1, 6))  # 调整画布大小以适应垂直方向
fig.subplots_adjust(left=0.5)

# 显示渐变条
im = ax.imshow(gradient, aspect='auto', cmap=cmap, norm=Normalize(vmin=vmin, vmax=vmax))

# 隐藏坐标轴
ax.set_xticks([])
ax.set_yticks([])

# 添加颜色条（垂直方向）
cbar = plt.colorbar(im, ax=ax, orientation='vertical')
cbar.set_label('Z Value (m)',fontsize=20)

# 设置颜色条数字的字体大小
cbar.ax.tick_params(labelsize=16)  # 设置刻度数字的字体大小

plt.show()