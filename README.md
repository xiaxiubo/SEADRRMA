# SEADRRMA Joint Test Suite

This repository is the portable joint-test release for the SEA controller work. It
contains the hardware communication path, controller implementations, deployment
artifacts, commissioning scripts, and offline analysis tools needed to reproduce a
single-joint test on another controller.

The complete manuscript package is in [`paper_latex/`](paper_latex/README.md), including
the current TRACE draft, previous DR-RMA draft, bibliography, figures, editable
artwork, PDF snapshots, and plotting scripts. Run `python tools/build_paper.py`
from the repository root to compile the current draft.

## Start here

Do not begin with a learned controller. Commission a new controller in this order:

1. Read `00_plan_tree.md`, `01_architecture_overview.md`, and
   `02_pdo_passthrough_design.md`.
2. Create the Python environment from `environment.yml`.
3. Run `test_ethercat_connection.py` and the read-only probes in `scripts/`.
4. Verify encoder units, signs, offsets, gear ratio, PDO layout, WKC, CiA 402 state,
   torque conversion, watchdog, and physical emergency stop.
5. Run the controller with torque output disabled and inspect all logged channels.
6. Apply conservative drive-level and software limits before enabling torque.

EtherCAT `OP` and a valid WKC prove process-data communication only. They do not prove
that the drive is enabled, the torque sign is correct, or physical motion is safe.

## Repository layout

| Path | Purpose |
| --- | --- |
| `communication/` | SEA drive and RS485/EtherCAT encoder transport |
| `controllers/` | TRACE/DRRMA, RMA, PPO, LQR, and PD-DOB source code and model loaders |
| `control_flow/` | Hardware experiment entry points and calibration procedures |
| `scripts/` | EtherCAT, PDO, drive, sensor, torque, and friction commissioning tools |
| `analysis/` | Plotting and metric extraction for saved experiment logs |
| `deployment_package_drrma_final_v1/` | DRRMA ONNX interface, benchmark, and metadata |
| `deployment_package_trace_hardware_final_v1_20260726/` | Hardware TRACE v1 interface, calibration, and metadata |
| `portable_runtime/` | Hardware-agnostic ONNX runner with safety gates and tests |

Model weights are deliberately not published in this source repository. Restore them
from the private release bundle using `MODEL_ARTIFACTS.sha256`, then verify every hash
before inference. The historical latency JSON files retain the machine paths on which they were
generated. Those paths are provenance only and must not be copied into a new runtime
configuration.

## Environment

The original hardware stack used Python 3.10, NumPy, SciPy, PyYAML, and PySOEM:

```bash
conda env create -f environment.yml
conda activate torch_env
```

Controller-specific packages such as PyTorch or ONNX Runtime are documented in each
deployment package. Vendor EtherCAT configuration, interface names, and permissions
remain machine-specific.

## Portable ONNX smoke test

The `portable_runtime/` package is the recommended first integration target when the
new controller does not share the original EtherCAT stack:

```bash
cd portable_runtime
python -m pip install -e ".[dev]"
python scripts/check_model.py --model models/trace_hardware_control.onnx
python scripts/run_joint_test.py \
  --config config/joint_profile.example.json \
  --model models/trace_hardware_control.onnx \
  --duration 10
```

The default backend is a mock plant and physical output is disabled. Copy a selected
ONNX model into `portable_runtime/models/` locally; that directory intentionally does
not track model weights.

## Hardware safety

This is research software, not a certified safety controller. Use an independent
emergency stop, guarded workspace, drive-level current/velocity/travel limits, and a
separate communication watchdog. Never infer physical safety from an offline ONNX
check, a simulation, EtherCAT `OP`, or a successful graph-latency benchmark.

The complete target-controller cycle must be measured from sensor acquisition through
actuator output. ONNX inference time alone is not complete control-loop latency.

## Data policy

Raw experiment logs, datasets, model weights, training checkpoints, credentials,
personal machine configuration, caches, and generated plots are not part of this
public source release.
