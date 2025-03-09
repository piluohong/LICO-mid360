import matplotlib.pyplot as plt




# 读取数据
def read_data(filename):
    with open(filename, 'r') as file:
        # 读取每一行并转换为浮点数
        data = [float(line.strip()) for line in file]
    return data



# 读取四个TXT文件
file1 = '/home/hyq/slam/lvio/src/LICO-mid360-main/data/1_timecost.txt'
file2 = '/home/hyq/slam/lvio/src/LICO-mid360-main/data/2_timecost.txt'
file3 = '/home/hyq/slam/lvio/src/LICO-mid360-main/data/3_timecost.txt'
file4 = '/home/hyq/slam/lvio/src/LICO-mid360-main/data/adaptive_timecost.txt'

# 读取四个文件的数据
data1 = read_data(file1)
data2 = read_data(file2)
data3 = read_data(file3)
data4 = read_data(file4)


# 创建 x 轴数据（假设是时间步或索引）
x = range(len(data1))
# data = [0]

# 绘制曲线
plt.plot(x, data1, label='1', marker='o')
plt.plot(x, data2, label='2', marker='s')
plt.plot(x, data3, label='3', marker='^')
# plt.plot(x, data, label='4', marker='d')
plt.plot(x, data4, label='adaptive', marker='x')

# 设置图例
plt.legend()

# 设置x轴和y轴标签
plt.xlabel('Time Step')
plt.ylabel('Data Value')

# 设置图表标题
plt.title('Data Value Over Time')

# 显示图表
plt.show()

