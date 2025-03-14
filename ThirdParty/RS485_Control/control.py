#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import time

def send_hex_command():
    rospy.init_node('serial_command_node')
    port = rospy.get_param('~port', '/dev/ttyUSB0')  # 串口名称，默认值可根据需要修改
    hex_command = rospy.get_param('~hex_command', '01 06 00 02 00 1E A8 02')  # HEX 指令，默认值可根据需要修改 CRC-16 MODBUS

    # 打开串口
    try:
        ser = serial.Serial(port, baudrate=9600, timeout=1)
        rospy.loginfo("已打开串口 %s", port)
    except serial.SerialException as e:
        rospy.logerr("打开串口错误: %s", e)
        return

    # 将 HEX 字符串转换为字节
    try:
        command_bytes = bytes.fromhex(hex_command)
    except ValueError as e:
        rospy.logerr("无效的 HEX 指令: %s", e)
        return

    # 发送指令
    ser.write(command_bytes)
    rospy.loginfo("已发送指令: %s", hex_command)

    # 等待一段时间以确保指令发送完成
    rospy.sleep(1)

    # 关闭串口
    ser.close()
    rospy.loginfo("已关闭串口 %s", port)

if __name__ == "__main__":
    try:
        send_hex_command()
    except rospy.ROSInterruptException:
        pass