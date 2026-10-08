#!/usr/bin/env python3
from __future__ import annotations

import argparse

import project_bootstrap  # noqa: F401

from communication.sea_motor_comm import CST_MODE, DriveCommand, SEARealtimeComm, SHUTDOWN, state_name, summarize_wkc


def main() -> int:
    parser = argparse.ArgumentParser(description="Diagnose EtherCAT + PDO + CiA 402 readiness.")
    parser.add_argument("--ifname", default="eno1")
    parser.add_argument("--cycle-time", type=float, default=0.005)
    parser.add_argument("--preheat-cycles", type=int, default=100)
    parser.add_argument("--stable-cycles", type=int, default=5)
    parser.add_argument("--enable-timeout", type=int, default=300)
    args = parser.parse_args()

    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
    try:
        comm.connect()
        print(f"Connected on {args.ifname}. expected_wkc={comm.expected_wkc}", flush=True)
        print(f"Master state after connect: {state_name(comm.master.state)}", flush=True)
        for item in comm.describe_slaves():
            print(
                "  slave[{idx}] name={name!r} man={man} id={prod} rev={rev} state={state} in={inp} out={out}".format(
                    idx=item.index,
                    name=item.name,
                    man=item.vendor_id,
                    prod=item.product_code,
                    rev=item.revision,
                    state=item.state_label,
                    inp=item.input_size,
                    out=item.output_size,
                ),
                flush=True,
            )
        print("XML process data targets:", flush=True)
        for row in comm.describe_xml_targets():
            print(row, flush=True)
        print("Runtime TxPDO layout:", flush=True)
        for row in comm.describe_pdo_layout("tx"):
            print(row, flush=True)
        print("Runtime RxPDO layout:", flush=True)
        for row in comm.describe_pdo_layout("rx"):
            print(row, flush=True)
        print("Runtime RxPDO command objects:", flush=True)
        for row in comm.describe_required_rxpdo():
            print(row, flush=True)

        preheat = comm.preheat_pdo(
            DriveCommand(controlword=SHUTDOWN, mode_of_operation=CST_MODE, target_torque=0),
            cycles=args.preheat_cycles,
            stable_cycles=args.stable_cycles,
            require_statusword_nonzero=False,
        )
        print(
            "PDO preheat: success={success} expected_wkc={expected} observed_wkc={observed} "
            "statusword=0x{sw:04X} mode={mode} cia402={cia402}".format(
                success=preheat.success,
                expected=preheat.expected_wkc,
                observed=summarize_wkc(preheat.observed_wkc),
                sw=preheat.last_diagnostics.statusword,
                mode=preheat.last_diagnostics.mode_display,
                cia402=preheat.last_diagnostics.cia402_state,
            ),
            flush=True,
        )
        if not preheat.success:
            if preheat.samples:
                print(preheat.sample_summary(), flush=True)
            return 1

        transitions = comm.enable_cia402(mode_of_operation=CST_MODE, timeout_cycles_per_step=args.enable_timeout)
        for result in transitions:
            diag = result.last_diagnostics
            print(
                "  step={step} success={success} expected_state={expected} observed_wkc={observed} "
                "statusword=0x{sw:04X} mode={mode} cia402={cia402} remote={remote} warning={warning} fault={fault}".format(
                    step=result.step_name,
                    success=result.success,
                    expected=result.expected_state,
                    observed=summarize_wkc(result.observed_wkc),
                    sw=diag.statusword,
                    mode=diag.mode_display,
                    cia402=diag.cia402_state,
                    remote=int(diag.remote),
                    warning=int(diag.warning),
                    fault=int(diag.fault),
                ),
                flush=True,
            )
            if not result.success:
                if result.samples:
                    print(result.sample_summary(), flush=True)
                return 1

        stabilized = comm.stabilize_enabled_state(mode_of_operation=CST_MODE, cycles=20, stable_cycles=args.stable_cycles)
        print(
            "Enabled-state stabilization: success={success} observed_wkc={observed} "
            "statusword=0x{sw:04X} mode={mode} cia402={cia402}".format(
                success=stabilized.success,
                observed=summarize_wkc(stabilized.observed_wkc),
                sw=stabilized.last_diagnostics.statusword,
                mode=stabilized.last_diagnostics.mode_display,
                cia402=stabilized.last_diagnostics.cia402_state,
            ),
            flush=True,
        )
        if not stabilized.success:
            if stabilized.samples:
                print(stabilized.sample_summary(), flush=True)
            return 1

        print("Drive communication diagnose completed successfully.", flush=True)
        return 0
    finally:
        comm.close()


if __name__ == "__main__":
    raise SystemExit(main())
