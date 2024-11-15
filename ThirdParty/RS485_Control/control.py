import serial
import time

def send_hex_command(port, hex_command):
    # 打开串口
    try:
        ser = serial.Serial(port, baudrate=9600, timeout=1)
        print(f"Opened port {port}")
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    # 将 HEX 字符串转换为字节
    try:
        command_bytes = bytes.fromhex(hex_command)
    except ValueError as e:
        print(f"Invalid HEX command: {e}")
        return

    # 发送指令
    ser.write(command_bytes)
    print(f"Sent command: {hex_command}")

    # 等待一段时间以确保指令发送完成
    time.sleep(1)

    # 关闭串口
    ser.close()
    print(f"Closed port {port}")

if __name__ == "__main__":
    port_name = "COM6"  # 替换为你的串口名称
    hex_command = "01 06 00 02 00 1E A8 02"  # 替换为你的 HEX 指令

    send_hex_command(port_name, hex_command)
