#!/usr/bin/env python3
"""Minimal test to debug EtherCAT connection hang."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from communication.sea_motor_comm import SEARealtimeComm

print("Creating SEARealtimeComm...", flush=True)
comm = SEARealtimeComm(ifname="eno1", cycle_time_s=0.005)
print("SEARealtimeComm created!", flush=True)

print("Calling comm.open()...", flush=True)
try:
    comm.open()
    print("comm.open() succeeded!", flush=True)
except Exception as e:
    print(f"comm.open() failed: {e}", flush=True)
    sys.exit(1)

print("Calling comm.scan()...", flush=True)
try:
    comm.scan()
    print("comm.scan() succeeded!", flush=True)
except Exception as e:
    print(f"comm.scan() failed: {e}", flush=True)
    comm.close()
    sys.exit(1)

print("Calling comm.load_xml_slave_profiles()...", flush=True)
try:
    comm.load_xml_slave_profiles()
    print("comm.load_xml_slave_profiles() succeeded!", flush=True)
except Exception as e:
    print(f"comm.load_xml_slave_profiles() failed: {e}", flush=True)
    comm.close()
    sys.exit(1)

print("All steps completed successfully!", flush=True)
comm.close()
