#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import project_bootstrap  # noqa: F401

from communication.sea_motor_comm import DriveCommand, SEARealtimeComm


CST_MODE = 10
ENABLE_OPERATION = 0x000F
SWITCH_ON = 0x0007
SHUTDOWN = 0x0006


def torque_nm_to_target_units(torque_nm: float, max_torque_nm: float) -> int:
    # 0x6071 目标力矩是按额定最大力矩的 1/1000 来表示的。
    # 这里把物理单位 Nm 换算成驱动器需要的归一化数值。
    return int(round(torque_nm / max_torque_nm * 1000.0))


def print_state(prefix: str, state) -> None:
    # 输出尽量保持简洁，但保留调参时最有用的几个量：
    # 状态字、模式、位置、速度、力矩、跟随误差、两个编码器值。
    print(
        "{prefix} statusword=0x{sw:04X} mode={mode} pos={pos:.6f}rad vel={vel:.6f}rad/s "
        "torque={torque:.3f}Nm fe={fe:.6f}rad enc1={enc1:.6f}rad enc2={enc2:.6f}rad".format(
            prefix=prefix,
            sw=state.statusword,
            mode=state.mode_display,
            pos=state.position_rad,
            vel=state.velocity_rad_s,
            torque=state.torque_nm,
            fe=state.following_error_rad,
            enc1=state.encoder1_rad,
            enc2=state.encoder2_rad,
        ),
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="SEA joint CST torque demo using communication.sea_motor_comm")
    parser.add_argument("--ifname", default="eno1")
    parser.add_argument("--duration", type=float, default=5.0, help="Torque hold duration in seconds.")
    parser.add_argument("--cycle-time", type=float, default=0.005, help="Control cycle time in seconds.")
    parser.add_argument("--torque-nm", type=float, default=-10.0, help="Command torque in Nm.")
    parser.add_argument("--max-torque-nm", type=float, default=61.0, help="Joint maximum torque after gear reduction.")
    parser.add_argument("--print-every", type=int, default=20, help="Print one sample every N cycles.")
    args = parser.parse_args()

    target_units = torque_nm_to_target_units(args.torque_nm, args.max_torque_nm)
    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)

    try:
        # 先完成 EtherCAT 上电、扫描、PDO 映射和 OP 切换。
        comm.connect()
        print(f"Connected on {args.ifname}. CST mode={CST_MODE}, target_torque={target_units} (for {args.torque_nm} Nm).", flush=True)

        # CST 的简化使能流程：
        # shutdown -> switch on -> enable operation
        for controlword in (SHUTDOWN, SWITCH_ON, ENABLE_OPERATION):
            for _ in range(10):
                wkc = comm.cycle(
                    DriveCommand(
                        controlword=controlword,
                        mode_of_operation=CST_MODE,
                        target_torque=0,
                )
                )
                state = comm.read_state_si()
            print_state(f"enable cw=0x{controlword:04X} wkc={wkc}", state)

        # 在指定时间内持续输出目标力矩。
        # 循环里先写命令，再完成一次过程数据交换，然后读取反馈做日志。
        start = time.monotonic()
        cycle = 0
        last_state = None
        while time.monotonic() - start < args.duration:
            wkc = comm.cycle(
                DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=target_units,
                )
            )
            state = comm.read_state_si()
            last_state = state
            if cycle % max(args.print_every, 1) == 0:
                print_state(f"run cycle={cycle} wkc={wkc}", state)
            cycle += 1

        # 退出前把力矩指令清零，避免最后一帧还保持受力。
        for _ in range(10):
            wkc = comm.cycle(
                DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=0,
                )
            )
            state = comm.read_state_si()
        if last_state is not None:
            print_state(f"stop wkc={wkc}", state)
        print("CST torque test done.", flush=True)
        return 0
    finally:
        comm.close()


if __name__ == "__main__":
    raise SystemExit(main())
