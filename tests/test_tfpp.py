import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from ksae_2026_autumn.tfpp import prepare_run, run


class TFPPTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.garage = root / "garage"
        self.output = root / "output"
        for folder in (
            "garage/leaderboard/leaderboard",
            "garage/leaderboard/data",
            "garage/team_code",
            "garage/scenario_runner",
            "carla/PythonAPI/carla",
            "model",
        ):
            (root / folder).mkdir(parents=True, exist_ok=True)
        # An independent stand-in validates the evaluator CLI/environment contract.
        evaluator = self.garage / "leaderboard/leaderboard/leaderboard_evaluator_local.py"
        evaluator.write_text(
            "import sys, os, json\nfrom pathlib import Path\n"
            "assert os.environ['IS_BENCH2DRIVE'] == 'False'\n"
            "assert os.environ['STOP_AFTER_METER'] == '-1'\n"
            "target = Path(sys.argv[sys.argv.index('--checkpoint') + 1])\n"
            "target.write_text(json.dumps({'kind': 'test_evaluator'}))\n",
            encoding="utf-8",
        )
        (self.garage / "team_code/sensor_agent.py").touch()
        (self.garage / "leaderboard/data/debug.xml").write_text("<routes/>")
        (root / "model/config.json").write_text("{}")
        (root / "model/model_0030_0.pth").write_bytes(b"test-weight")
        self.model = root / "model"
        self.config = root / "tfpp.yaml"
        self.config.write_text(
            yaml.safe_dump(
                {
                    "garage_root": str(self.garage),
                    "carla_root": str(root / "carla"),
                    "model_dir": str(self.model),
                    "routes": "leaderboard/data/debug.xml",
                    "output_dir": str(self.output),
                }
            )
        )

    def test_dry_run_does_not_create_output(self):
        self.assertEqual(run(self.config, dry_run=True), 0)
        self.assertFalse(self.output.exists())

    def test_launch_and_metadata_without_carla(self):
        self.assertEqual(run(self.config), 0)
        self.assertEqual(
            json.loads((self.output / "result.json").read_text()), {"kind": "test_evaluator"}
        )
        metadata = json.loads((self.output / "run.json").read_text())
        self.assertEqual(metadata["exit_code"], 0)
        self.assertEqual(len(metadata["weight_sha256"]), 64)
        with self.assertRaises(FileExistsError):
            run(self.config)

    def test_prevents_implicit_ensemble(self):
        (self.model / "model_0030_1.pth").touch()
        with self.assertRaisesRegex(ValueError, "exactly one"):
            prepare_run(self.config)

    def test_clears_inherited_benchmark_environment(self):
        with patch.dict(os.environ, {"IS_BENCH2DRIVE": "True", "PYTHONPATH": "/wrong"}):
            _, env, _ = prepare_run(self.config)
        self.assertEqual(env["IS_BENCH2DRIVE"], "False")
        self.assertNotIn("/wrong", env["PYTHONPATH"])

    def test_rejects_benchmark_routes_in_local_evaluator(self):
        route = self.garage / "Bench2Drive/leaderboard/data/routes.xml"
        route.parent.mkdir(parents=True)
        route.write_text("<routes/>")
        config = yaml.safe_load(self.config.read_text())
        config["routes"] = str(route)
        self.config.write_text(yaml.safe_dump(config))
        with self.assertRaisesRegex(ValueError, "Bench2Drive evaluator"):
            prepare_run(self.config)
