#!/usr/bin/env python3
import rospy
import serial
import time
from std_msgs.msg import UInt8MultiArray

def read_serial_data(ser):
    """
    从串口读取数据并解析
    """
    try:
        if ser.in_waiting > 0:  # 检查是否有数据可读
            data = ser.read(ser.in_waiting).decode('utf-8')  # 读取所有可用数据
            if '#' in data and '$' in data:
                start = data.find('#') + 1  # 找到#的位置并跳过它
                end = data.find('$')       # 找到$的位置
                if end > start and end - start == 1:  # 确保数据长度为1个字符
                    hex_data = data[start:end]  # 提取数据部分
                    if len(hex_data) == 1:  # 确保数据长度为1个字符
                        # 将字符串转换为一个8位整数
                        start = int(hex_data[0], 16)

                        # 将数据存储到参数服务器中
                        rospy.set_param('/asrpro_node/start', start)

    except Exception as e:
        rospy.logerr(f"Error reading serial data: {e}")

def main():
    rospy.init_node('asrpro_node', anonymous=True)  # 初始化节点

    # 配置串口
    port = rospy.get_param('~port', '/dev/asrpro')  # 串口设备文件路径
    baudrate = rospy.get_param('~baudrate', 115200)    # 波特率
    timeout = rospy.get_param('~timeout', 1.0)       # 超时时间

    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)  # 打开串口
        rospy.loginfo(f"Serial Port initialized: {port} at {baudrate} baud")
    except serial.SerialException as e:
        rospy.logerr(f"Unable to open port: {e}")
        return

    rate = rospy.Rate(1)  # 设置循环频率为10Hz
    while not rospy.is_shutdown():
        read_serial_data(ser)  # 调用读取串口数据的函数
        rate.sleep()  # 按照指定频率暂停

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass