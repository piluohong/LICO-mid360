#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import signal
import sys
import os

class OdomToTUM:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('odom_to_tum', anonymous=True)
        print("++++++ 开始记录 Odometry 数据 +++++")

        # 检查文件路径是否存在，如果不存在则创建目录
        output_path = '/home/hyq/slam/lvio/src/LICO-mid360-main/data/hhh/LIO_street_04.txt'
        output_dir = os.path.dirname(output_path)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # 打开文件（'w' 模式会覆盖原有内容，'a' 模式会追加内容）
        self.output_file = open(output_path, 'w')

        # 注册信号处理函数，用于捕获 Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # 订阅 Odometry 话题
        self.odom_sub = rospy.Subscriber('/lidarAndcameraOdom', Odometry, self.odom_callback, queue_size=10)

        # 初始化上一个时间戳
        self.last_timestamp = None

    def odom_callback(self, msg):
        # 获取时间戳（单位为秒）
        timestamp = msg.header.stamp.to_sec()

        # # 检查时间戳是否与上一个相同
        # if timestamp == self.last_timestamp:
        #     print(f"警告：跳过重复时间戳 {timestamp}")
        #     return

        # # 更新上一个时间戳
        # self.last_timestamp = timestamp

        # 获取位置信息
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        z = position.z

        # 获取姿态信息（四元数）
        orientation = msg.pose.pose.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # 将数据写入文件（TUM 格式：时间戳 x y z qx qy qz qw）
        self.output_file.write(f"{timestamp:.6f} {x:.16f} {y:.16f} {z:.16f} {qx:.16f} {qy:.16f} {qz:.16f} {qw:.16f}\n")

    def run(self):
        # 保持节点运行，等待回调函数被触发
        rospy.spin()

    def __del__(self):
        # 对象销毁时关闭文件
        if hasattr(self, 'output_file') and not self.output_file.closed:
            self.output_file.close()

    def signal_handler(self, sig, frame):
        # 捕获 Ctrl+C 信号，关闭文件并退出程序
        print("\n检测到 Ctrl+C，正在关闭文件并退出...")
        if hasattr(self, 'output_file') and not self.output_file.closed:
            self.output_file.close()
        rospy.signal_shutdown("用户中断")
        sys.exit(0)

if __name__ == '__main__':
    try:
        recorder = OdomToTUM()
        recorder.run()
    except rospy.ROSInterruptException:
        pass  # 如果 ROS 被中断，正常退出