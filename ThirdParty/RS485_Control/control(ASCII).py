import serial
import time

def send_ascii_command(port, ascii_command):
    # 打开串口
    try:
        ser = serial.Serial(port, baudrate=9600, timeout=1)
        print(f"Opened port {port}")
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    # 将 ASCII 字符串转换为字节
    command_bytes = ascii_command.encode('ascii')

    # 发送指令
    ser.write(command_bytes)
    print(f"Sent command: {ascii_command}")

    # 等待一段时间以确保指令发送完成
    time.sleep(1)

    # 关闭串口
    ser.close()
    print(f"Closed port {port}")

if __name__ == "__main__":
    port_name = "COM6"  # 替换为你的串口名称
    ascii_command = "your_ascii_command_here"  # 替换为你的 ASCII 指令

    send_ascii_command(port_name, ascii_command)