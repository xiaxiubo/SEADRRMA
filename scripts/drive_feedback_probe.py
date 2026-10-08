#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict

import project_bootstrap  # noqa: F401

from communication.sea_motor_comm import CST_MODE, DriveCommand, ENABLE_OPERATION, SEARealtimeComm, SHUTDOWN, summarize_wkc


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe SEA drive position/velocity/encoder feedback after startup.")
    parser.add_argument("--ifname", default="eno1")
    parser.add_argument("--cycle-time", type=float, default=0.005)
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--print-every", type=int, default=1)
    args = parser.parse_args()

    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
    wkc_samples: list[int] = []
    encoder_feedback_samples: dict[int, list] = defaultdict(list)
    drive_position_samples: list[int] = []
    pdo_encoder_samples: dict[int, list[int]] = defaultdict(list)

    try:
        comm.connect()
        print(f"Connected on {args.ifname}. expected_wkc={comm.expected_wkc}", flush=True)
        for item in comm.describe_slaves():
            print(
                "  slave[{idx}] name={name!r} state={state} in={inp} out={out}".format(
                    idx=item.index,
                    name=item.name,
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
            cycles=100,
            stable_cycles=5,
            require_statusword_nonzero=False,
        )
        if not preheat.success:
            print("PDO preheat failed.", flush=True)
            if preheat.samples:
                print(preheat.sample_summary(), flush=True)
            return 1

        transitions = comm.enable_cia402(mode_of_operation=CST_MODE, timeout_cycles_per_step=300)
        for result in transitions:
            if not result.success:
                print(f"CiA 402 transition failed at {result.step_name}.", flush=True)
                if result.samples:
                    print(result.sample_summary(), flush=True)
                return 1

        stabilized = comm.stabilize_enabled_state(mode_of_operation=CST_MODE, cycles=20, stable_cycles=5)
        if not stabilized.success:
            print("Enabled-state stabilization failed.", flush=True)
            if stabilized.samples:
                print(stabilized.sample_summary(), flush=True)
            return 1

        encoder1_config = comm.read_encoder_config(1)
        encoder2_config = comm.read_encoder_config(2)
        for config in (encoder1_config, encoder2_config):
            print(
                "encoder{ch} config: type={type_name}({type_code}) port={port} resolution={resolution} "
                "polarity={polarity} offset={offset} index={index}".format(
                    ch=config.channel,
                    type_name=config.encoder_type_name,
                    type_code=config.encoder_type,
                    port=config.sensor_port,
                    resolution=config.resolution,
                    polarity=int(config.polarity),
                    offset=config.singleturn_offset,
                    index=config.index_availability,
                ),
                flush=True,
            )

        for cycle_index in range(args.cycles):
            wkc = comm.cycle(
                DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=0,
                )
            )
            wkc_samples.append(wkc)
            state = comm.read_state()
            state_si = comm.read_state_si()
            diag = comm.read_drive_diagnostics()
            encoder1_feedback = comm.read_encoder_feedback(1)
            encoder2_feedback = comm.read_encoder_feedback(2)
            encoder_feedback_samples[1].append(encoder1_feedback)
            encoder_feedback_samples[2].append(encoder2_feedback)
            drive_position_samples.append(state.position)
            pdo_encoder_samples[1].append(state.encoder1)
            pdo_encoder_samples[2].append(state.encoder2)
            if cycle_index % max(args.print_every, 1) == 0:
                print(
                    "cycle={cycle} wkc={wkc} sw=0x{sw:04X} mode={mode} cia402={cia402} "
                    "pos={pos} vel={vel} torque={torque} pdo_enc1={pdo_enc1} pdo_enc2={pdo_enc2} "
                    "enc1_raw={enc1_raw} enc1_adj={enc1_adj} enc1_vel={enc1_vel} "
                    "enc2_raw={enc2_raw} enc2_adj={enc2_adj} enc2_vel={enc2_vel} "
                    "pos_rad={pos_rad:.6f} vel_rad_s={vel_rad:.6f} "
                    "enc1_rad={enc1_rad:.6f} enc2_rad={enc2_rad:.6f}".format(
                        cycle=cycle_index,
                        wkc=wkc,
                        sw=diag.statusword,
                        mode=diag.mode_display,
                        cia402=diag.cia402_state,
                        pos=state.position,
                        vel=state.velocity,
                        torque=state.torque,
                        pdo_enc1=state.encoder1,
                        pdo_enc2=state.encoder2,
                        enc1_raw=encoder1_feedback.raw_position,
                        enc1_adj=encoder1_feedback.adjusted_position,
                        enc1_vel=encoder1_feedback.velocity_rpm,
                        enc2_raw=encoder2_feedback.raw_position,
                        enc2_adj=encoder2_feedback.adjusted_position,
                        enc2_vel=encoder2_feedback.velocity_rpm,
                        pos_rad=state_si.position_rad,
                        vel_rad=state_si.velocity_rad_s,
                        enc1_rad=state_si.encoder1_rad,
                        enc2_rad=state_si.encoder2_rad,
                    ),
                    flush=True,
                )

        encoder1_assessment = comm.assess_encoder_feedback_samples(
            channel=1,
            config=encoder1_config,
            feedback_samples=encoder_feedback_samples[1],
            drive_position_samples=drive_position_samples,
            pdo_encoder_samples=pdo_encoder_samples[1],
        )
        encoder2_assessment = comm.assess_encoder_feedback_samples(
            channel=2,
            config=encoder2_config,
            feedback_samples=encoder_feedback_samples[2],
            drive_position_samples=drive_position_samples,
            pdo_encoder_samples=pdo_encoder_samples[2],
        )
        print(encoder1_assessment.summary_line(), flush=True)
        print(encoder2_assessment.summary_line(), flush=True)
        print(f"WKC summary: {summarize_wkc(wkc_samples)}", flush=True)
        return 0
    finally:
        comm.close()


if __name__ == "__main__":
    raise SystemExit(main())
