#!/usr/bin/env python3
"""Conservative single-joint CST/PD sine test for gain tuning."""
from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication.sea_motor_comm import (
    CST_MODE,
    ENABLE_OPERATION,
    SHUTDOWN,
    SWITCH_ON,
    DriveCommand,
    SEARealtimeComm,
    cia402_state_name,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = PROJECT_ROOT / "logs"


def filename_number(value: float, decimals: int) -> str:
    """Format a numeric parameter as a filesystem-friendly stable token."""
    return f"{value:.{decimals}f}".replace("-", "m").replace(".", "p")


def experiment_stem(args: argparse.Namespace, timestamp: str) -> str:
    return (
        f"single_joint_pd_j{args.joint_index}"
        f"_amp{filename_number(args.amplitude_deg, 2)}deg"
        f"_f{filename_number(args.frequency, 3)}hz"
        f"_kp{filename_number(args.kp, 1)}"
        f"_kd{filename_number(args.kd, 1)}"
        f"_{timestamp}"
    )


def torque_units(torque_nm: float, rated_torque_nm: float) -> int:
    return int(max(-1000, min(1000, round(1000.0 * torque_nm / rated_torque_nm))))


def smoothstep(u: float) -> float:
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def envelope(elapsed_s: float, duration_s: float, ramp_s: float) -> float:
    edge = min(max(ramp_s, 0.0), duration_s * 0.5)
    if edge <= 0.0:
        return 1.0
    return smoothstep(min(elapsed_s / edge, (duration_s - elapsed_s) / edge))


class SingleJointBus:
    def __init__(self, comm: SEARealtimeComm, slave_index: int):
        self.comm = comm
        self.slave_index = slave_index

    def select(self) -> None:
        if self.comm.roles is None:
            raise RuntimeError("EtherCAT roles are unavailable")
        if not 0 <= self.slave_index < len(self.comm.master.slaves):
            raise ValueError(f"slave index {self.slave_index} is out of range")
        slave = self.comm.master.slaves[self.slave_index]
        name = (getattr(slave, "name", "") or "").lower()
        if "somanet" not in name and "circulo" not in name:
            raise ValueError(f"slave[{self.slave_index}] is not a SOMANET/Circulo drive: {name!r}")
        self.comm.drive = slave
        self.comm.roles = type(self.comm.roles)(
            box_index=self.comm.roles.box_index,
            drive_index=self.slave_index,
        )

    def read(self):
        self.select()
        return self.comm.read_state_si()

    def exchange(self, command: DriveCommand) -> int:
        self.select()
        self.comm.write_command(command)
        self.comm.send_processdata()
        return self.comm.receive_processdata(5_000)

    def preheat(self, cycles: int = 50) -> tuple[int, ...]:
        observed = []
        for _ in range(max(cycles, 1)):
            self.comm.send_processdata()
            observed.append(self.comm.receive_processdata(5_000))
            time.sleep(self.comm.cycle_time_s)
        return tuple(observed)

    def transition(self, controlword: int, expected_state: str, cycles: int = 300) -> None:
        command = DriveCommand(controlword, CST_MODE, 0)
        for _ in range(cycles):
            wkc = self.exchange(command)
            state = self.read()
            if state.statusword & 0x0008:
                raise RuntimeError(f"drive fault: statusword=0x{state.statusword:04X}")
            if wkc == self.comm.expected_wkc and cia402_state_name(state.statusword) == expected_state:
                return
            time.sleep(self.comm.cycle_time_s)
        raise RuntimeError(f"timeout entering {expected_state}; statusword=0x{self.read().statusword:04X}")

    def enable(self) -> None:
        for controlword, expected in (
            (SHUTDOWN, "Ready to switch on"),
            (SWITCH_ON, "Switched on"),
            (ENABLE_OPERATION, "Operation enabled"),
        ):
            self.transition(controlword, expected)

    def stop(self, cycles: int = 20) -> None:
        zero = DriveCommand(ENABLE_OPERATION, CST_MODE, 0)
        for _ in range(max(cycles, 1)):
            try:
                self.exchange(zero)
            except Exception:
                break
        try:
            self.exchange(DriveCommand(SHUTDOWN, CST_MODE, 0))
        except Exception:
            pass


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Single-joint low-frequency PD tuning test")
    p.add_argument("--ifname", default="eno1")
    p.add_argument("--joint-index", type=int, default=0)
    p.add_argument("--cycle-time", type=float, default=0.005)
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--settle-s", type=float, default=3.0, help="Hold initial position before sine motion")
    p.add_argument("--amplitude-deg", type=float, default=2.0)
    p.add_argument("--frequency", type=float, default=0.10)
    p.add_argument("--ramp-s", type=float, default=2.0)
    p.add_argument("--kp", type=float, default=30.0)
    p.add_argument("--kd", type=float, default=5.0)
    p.add_argument("--torque-limit-nm", type=float, default=6.0)
    p.add_argument("--rated-torque-nm", type=float, default=61.0)
    p.add_argument("--max-position-error-deg", type=float, default=8.0)
    p.add_argument("--max-velocity-rad-s", type=float, default=1.0)
    p.add_argument("--max-wkc-errors", type=int, default=3)
    p.add_argument("--print-every", type=int, default=20)
    p.add_argument("--out-dir", default=str(DEFAULT_LOG_DIR))
    p.add_argument("--execute", action="store_true")
    return p


def validate(args: argparse.Namespace) -> None:
    positive = (args.cycle_time, args.duration, args.rated_torque_nm, args.max_position_error_deg,
                args.max_velocity_rad_s, args.max_wkc_errors)
    if any(value <= 0 for value in positive):
        raise ValueError("cycle/duration/rated torque/safety limits must be positive")
    if args.settle_s < 0 or args.settle_s >= args.duration:
        raise ValueError("--settle-s must be in [0, duration)")
    if args.amplitude_deg < 0 or args.frequency < 0 or args.kp < 0 or args.kd < 0:
        raise ValueError("amplitude, frequency, Kp and Kd must be non-negative")
    if not 0 < args.torque_limit_nm <= args.rated_torque_nm:
        raise ValueError("torque limit must be in (0, rated torque]")


def run(args: argparse.Namespace) -> Path | None:
    validate(args)
    comm = SEARealtimeComm(args.ifname, args.cycle_time)
    bus: SingleJointBus | None = None
    control_started = False
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        comm.connect()
        bus = SingleJointBus(comm, args.joint_index)
        bus.select()
        observed_wkc = bus.preheat()
        initial = bus.read()
        print(
            f"slave[{args.joint_index}] expected_wkc={comm.expected_wkc} "
            f"observed={min(observed_wkc)}..{max(observed_wkc)} "
            f"pos={initial.position_rad:+.6f}rad vel={initial.velocity_rad_s:+.6f}rad/s "
            f"sw=0x{initial.statusword:04X} ({cia402_state_name(initial.statusword)})"
        )
        if initial.statusword == 0 or any(wkc != comm.expected_wkc for wkc in observed_wkc[-5:]):
            raise RuntimeError("invalid PDO feedback or unstable WKC; refusing to enable")
        if not args.execute:
            print("Monitor-only. Add --execute only after checking joint index, support and E-stop.")
            return None

        center = initial.position_rad
        amplitude = math.radians(args.amplitude_deg)
        max_error = math.radians(args.max_position_error_deg)
        control_started = True
        bus.enable()

        out_dir = Path(args.out_dir).expanduser().resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = experiment_stem(args, timestamp)
        path = out_dir / f"{stem}.csv"
        config_path = out_dir / f"{stem}_config.json"
        config = vars(args).copy()
        config.update(
            {
                "created_at": timestamp,
                "csv_path": str(path),
                "joint_center_rad": center,
                "initial_statusword": initial.statusword,
                "expected_wkc": comm.expected_wkc,
            }
        )
        with config_path.open("w", encoding="utf-8") as stream:
            json.dump(config, stream, ensure_ascii=False, indent=2)
        print(f"CSV log: {path}")
        print(f"Config:  {config_path}")
        fields = ("cycle", "time_s", "ref_rad", "position_rad", "error_rad", "velocity_rad_s",
                  "ref_velocity_rad_s", "command_torque_nm", "measured_torque_nm", "target_torque",
                  "torque_saturated", "statusword", "wkc")
        start_ns = time.monotonic_ns()
        deadline_ns = start_ns
        period_ns = round(args.cycle_time * 1e9)
        cycle = 0
        wkc_errors = 0

        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            while not stop_requested:
                elapsed = (time.monotonic_ns() - start_ns) / 1e9
                if elapsed >= args.duration:
                    break
                state = bus.read()
                motion_t = max(0.0, elapsed - args.settle_s)
                if elapsed < args.settle_s:
                    env = 0.0
                else:
                    env = envelope(motion_t, args.duration - args.settle_s, args.ramp_s)
                phase = 2.0 * math.pi * args.frequency * motion_t
                ref = center + env * amplitude * math.sin(phase)
                ref_velocity = env * amplitude * 2.0 * math.pi * args.frequency * math.cos(phase)
                error = ref - state.position_rad
                raw_torque = args.kp * error + args.kd * (ref_velocity - state.velocity_rad_s)
                torque = max(-args.torque_limit_nm, min(args.torque_limit_nm, raw_torque))
                saturated = int(abs(raw_torque) > args.torque_limit_nm)

                reason = None
                if state.statusword & 0x0008:
                    reason = f"drive fault 0x{state.statusword:04X}"
                elif abs(error) > max_error:
                    reason = f"position error {math.degrees(error):+.2f}deg exceeds limit"
                elif abs(state.velocity_rad_s) > args.max_velocity_rad_s:
                    reason = f"velocity {state.velocity_rad_s:+.3f}rad/s exceeds limit"
                if reason:
                    print(f"STOP: {reason}")
                    break

                target = torque_units(torque, args.rated_torque_nm)
                wkc = bus.exchange(DriveCommand(ENABLE_OPERATION, CST_MODE, target))
                wkc_errors = wkc_errors + 1 if wkc != comm.expected_wkc else 0
                writer.writerow({
                    "cycle": cycle, "time_s": elapsed, "ref_rad": ref, "position_rad": state.position_rad,
                    "error_rad": error, "velocity_rad_s": state.velocity_rad_s,
                    "ref_velocity_rad_s": ref_velocity, "command_torque_nm": torque,
                    "measured_torque_nm": state.torque_nm, "target_torque": target,
                    "torque_saturated": saturated, "statusword": state.statusword, "wkc": wkc,
                })
                if cycle % max(args.print_every, 1) == 0:
                    print(
                        f"cycle={cycle:5d} t={elapsed:6.3f}s ref={math.degrees(ref-center):+6.3f}deg "
                        f"pos={math.degrees(state.position_rad-center):+6.3f}deg "
                        f"err={math.degrees(error):+6.3f}deg vel={state.velocity_rad_s:+.3f}rad/s "
                        f"tau={torque:+.2f}Nm sat={saturated} wkc={wkc}"
                    )
                if wkc_errors >= args.max_wkc_errors:
                    print(f"STOP: WKC {wkc} != expected {comm.expected_wkc}")
                    break
                cycle += 1
                deadline_ns += period_ns
                remaining = deadline_ns - time.monotonic_ns()
                if remaining > 0:
                    time.sleep(remaining / 1e9)
                else:
                    deadline_ns = time.monotonic_ns()
        print(f"Log saved to {path}")
        return path
    finally:
        if bus is not None and control_started:
            bus.stop()
        comm.close()


def main() -> int:
    run(parser().parse_args())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
