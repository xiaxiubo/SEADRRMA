#!/usr/bin/env python3
"""
USB转485继电器控制测试程序
每2秒切换一次继电器状态，总共运行10秒
"""

import serial
import time


def main():
    # 继电器控制命令（含CRC校验码）
    relay_on_cmd = bytes([0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A])  # 打开
    relay_off_cmd = bytes([0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA])  # 关闭

    print(f"打开继电器命令: {relay_on_cmd.hex(' ').upper()}")
    print(f"关闭继电器命令: {relay_off_cmd.hex(' ').upper()}")
    print()

    try:
        # 打开串口
        ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

        print(f"串口 {ser.port} 已打开")
        print(f"波特率: {ser.baudrate}")
        print("开始测试，每2秒切换一次继电器状态...\n")

        start_time = time.time()
        relay_state = False  # False=关闭, True=打开

        while time.time() - start_time < 10:
            # 切换继电器状态
            relay_state = not relay_state

            if relay_state:
                ser.write(relay_on_cmd)
                print(f"[{time.time() - start_time:.1f}s] 发送打开命令")
            else:
                ser.write(relay_off_cmd)
                print(f"[{time.time() - start_time:.1f}s] 发送关闭命令")

            # 读取响应（如果有）
            time.sleep(0.1)
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"    响应: {response.hex(' ').upper()}")

            # 等待2秒
            time.sleep(2)

        # 测试结束，关闭继电器
        ser.write(relay_off_cmd)
        print(f"\n[{time.time() - start_time:.1f}s] 测试结束，关闭继电器")

        ser.close()
        print("串口已关闭")

    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("请检查：")
        print("1. USB转485设备是否已连接")
        print("2. 设备是否在 /dev/ttyUSB0")
        print("3. 是否有权限访问串口（可能需要 sudo 或加入 dialout 组）")
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
        if 'ser' in locals() and ser.is_open:
            ser.write(relay_off_cmd)
            ser.close()
            print("已关闭继电器和串口")
    except Exception as e:
        print(f"发生错误: {e}")


if __name__ == "__main__":
    main()
