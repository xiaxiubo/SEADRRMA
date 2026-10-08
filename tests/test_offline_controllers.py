from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from controllers.ppo.ppo_controller import PpoActorController
from controllers.rma.rma_controller import RmaOnnxActorController
from controllers.trace_onnx_controller import TRACEOnnxController


ROOT = Path(__file__).resolve().parents[1]


class OfflineControllerTests(unittest.TestCase):
    def test_ppo_actor(self) -> None:
        controller = PpoActorController(warmup_steps=0)
        action, torque = controller.run_actor(np.zeros(9, dtype=np.float32))
        self.assertTrue(np.isfinite(action))
        self.assertTrue(np.isfinite(torque))
        self.assertLessEqual(abs(action), 1.0)

    def test_rma_onnx_actor(self) -> None:
        controller = RmaOnnxActorController()
        action, torque = controller.run_actor(np.zeros(9, dtype=np.float32))
        self.assertTrue(np.isfinite(action))
        self.assertTrue(np.isfinite(torque))
        self.assertLessEqual(abs(action), 1.0)

    def test_trace_onnx_actor(self) -> None:
        model = (
            ROOT
            / "deployment_package_trace_hardware_final_v1_20260726"
            / "onnx"
            / "trace_hardware_control.onnx"
        )
        controller = TRACEOnnxController(model, num_threads=1)
        outputs = controller.run_inference(
            np.zeros(8, dtype=np.float32),
            elapsed_s=0.0,
        )
        self.assertEqual(len(outputs), 8)
        self.assertTrue(np.all(np.isfinite(outputs)))
        self.assertLessEqual(abs(outputs[0]), 1.0)


if __name__ == "__main__":
    unittest.main()

