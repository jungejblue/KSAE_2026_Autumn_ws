"""Focused telemetry tests without CARLA, GPU, PyTorch, or model downloads."""

import importlib.util
import json
import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from ksae_2026_autumn.telemetry import RouteTelemetry, write_json
from ksae_2026_autumn.telemetry_check import check_run
from ksae_2026_autumn.telemetry_runtime import instrument_submit


def vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def control():
    return NS(
        throttle=0.2,
        brake=0.0,
        steer=-0.1,
        hand_brake=False,
        reverse=False,
        manual_gear_shift=False,
        gear=0,
    )


class Actor:
    def __init__(self, actor_id=1, kind="vehicle.lincoln.mkz_2020"):
        self.id, self.type_id = actor_id, kind
        self.bounding_box = NS(
            location=vec(), extent=vec(2, 1, 0.8), rotation=NS(roll=0, pitch=0, yaw=0)
        )

    def get_control(self):
        return control()


class SnapshotActor:
    def get_transform(self):
        return NS(location=vec(1, 2, 0.5), rotation=NS(roll=0, pitch=0, yaw=90))

    def get_velocity(self):
        return vec(0, 4, 0)

    def get_acceleration(self):
        return vec()

    def get_angular_velocity(self):
        return vec(0, 0, 90)


class World:
    def __init__(self):
        self.frame = 100
        self.reads = 0

    def get_snapshot(self):
        self.reads += 1
        return NS(
            frame=self.frame,
            timestamp=NS(elapsed_seconds=10 + (self.frame - 100) * 0.05),
            find=lambda actor_id: SnapshotActor(),
        )

    def get_actors(self):
        return [Actor(), Actor(2, "walker.pedestrian.0001")]

    def tick(self):
        raise AssertionError("Logger must not tick the world")


class Agent:
    def __init__(self):
        self.initialized = False
        self.config = NS(
            backbone="transFuser", lidar_seq_len=1, data_save_freq=1, inital_frames_delay=0
        )
        self.lidar_buffer = deque(maxlen=1)
        self.step = -1
        self.calls = 0
        self.output = control()

    def run_step(self, data, timestamp):
        self.calls += 1
        self.step += 1
        if self.initialized:
            self.lidar_buffer.append("frame")
        self.initialized = True
        return self.output


class Manager:
    def _tick_scenario(self):
        self.events.append("before")
        ego_action = self._agent_wrapper()
        self.ego_vehicles[0].apply_control(ego_action)
        self.events.append("after")

    def duplicate(self):
        ego_action = self._agent_wrapper()
        self.ego_vehicles[0].apply_control(ego_action)
        self.ego_vehicles[0].apply_control(ego_action)


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.rid = "RouteScenario_24211_rep0"

    def make_run(self, enabled=True, length=4):
        meta = {
            "run_id": "test",
            "exit_code": 0,
            "finished_at": "test",
            "runtime": {"python": "test"},
            "logging_enabled": enabled,
            "expected_route_ids": [self.rid],
        }
        write_json(self.root / "run.json", meta)
        (self.root / "exit_code.txt").write_text("0\n")
        write_json(
            self.root / "result.json",
            {
                "entry_status": "Finished",
                "eligible": True,
                "_checkpoint": {
                    "progress": [1, 1],
                    "records": [
                        {
                            "route_id": self.rid,
                            "status": "Completed",
                            "scores": {"score_route": 100},
                            "infractions": {},
                        }
                    ],
                },
            },
        )
        info = {
            "run_id": "test",
            "route_id": self.rid,
            "repetition": 0,
            "branch_id": "clean",
            "sensor_specs": [
                {"id": "rgb", "type": "sensor.camera.rgb"},
                {"id": "lidar", "type": "sensor.lidar.ray_cast"},
            ],
            "simulation": {"synchronous_mode": True, "fixed_delta_seconds": 0.05},
        }
        world, agent = World(), Agent()
        route = RouteTelemetry(self.root / "routes" / self.rid, info, enabled, world, Actor())
        for index in range(length):
            world.frame = 100 + index
            data = {"rgb": (world.frame, None), "lidar": (world.frame, None)}
            returned = route.call_agent(agent.run_step, agent, data, (index + 1) * 0.05)
            self.assertIs(returned, agent.output)
            route.submitted(returned, world.frame)
        route.close("stop_scenario")
        return route, world, agent

    @property
    def ticks(self):
        return self.root / "routes" / self.rid / "ticks.jsonl"

    def rows(self):
        return [json.loads(line) for line in self.ticks.read_text().splitlines()]

    def save_rows(self, rows):
        self.ticks.write_text("".join(json.dumps(r) + "\n" for r in rows))

    def assert_error(self, code):
        report = check_run(self.root)
        self.assertEqual(report["status"], "FAIL", report)
        self.assertIn(code, report["error_counts"], report)

    def test_complete_trace_and_pass_through(self):
        route, world, agent = self.make_run()
        self.assertEqual((agent.calls, world.reads, route.rows), (4, 4, 4))
        self.assertEqual(check_run(self.root)["status"], "PASS")
        self.assertEqual(self.rows()[0]["warmup_reason"], "initialization")
        self.assertEqual(self.rows()[1]["lidar_input_frame_pairs"], [[100, 101]])
        self.assertAlmostEqual(self.rows()[0]["ego"]["rpy_rad"][2], 1.5707963267948966)

    def test_off_has_counts_without_snapshot_collection(self):
        route, world, agent = self.make_run(enabled=False)
        self.assertEqual((agent.calls, world.reads, route.rows), (4, 0, 0))
        self.assertFalse(self.ticks.exists())
        self.assertEqual(check_run(self.root)["status"], "PASS")

    def test_duplicate_frame_detected(self):
        self.make_run()
        rows = self.rows()
        rows.append(rows[-1])
        self.save_rows(rows)
        self.assert_error("duplicate_frame")

    def test_missing_middle_frame_detected(self):
        self.make_run()
        rows = self.rows()
        del rows[1]
        self.save_rows(rows)
        self.assert_error("frame_gap_or_reverse")

    def test_invalid_control_detected(self):
        self.make_run()
        rows = self.rows()
        rows[1]["submitted_control"]["throttle"] = 1.5
        self.save_rows(rows)
        self.assert_error("control_range")

    def test_changed_valid_control_detected(self):
        self.make_run()
        rows = self.rows()
        rows[1]["submitted_control"]["steer"] = 0.2
        self.save_rows(rows)
        self.assert_error("changed_control")

    def test_truncated_json_detected(self):
        self.make_run()
        self.ticks.write_text(self.ticks.read_text()[:-10])
        self.assert_error("invalid_row")

    def test_sensor_frame_mismatch_detected(self):
        self.make_run()
        rows = self.rows()
        rows[1]["sensor_frames"]["rgb"] -= 1
        self.save_rows(rows)
        self.assert_error("sensor_frame")

    def test_nonnumeric_state_detected(self):
        self.make_run()
        rows = self.rows()
        rows[1]["ego"]["velocity_mps"][0] = float("nan")
        self.save_rows(rows)
        self.assert_error("state_numbers")

    def test_wrong_clock_detected(self):
        self.make_run()
        rows = self.rows()
        rows[1]["sim_time_s"] += 0.02
        self.save_rows(rows)
        self.assert_error("time_step")

    def test_nonempty_result_is_not_sufficient(self):
        self.make_run()
        write_json(self.root / "result.json", {"_checkpoint": {"records": []}})
        self.assert_error("evaluator_not_finished")

    def test_hook_preserves_order_and_control_identity(self):
        events = []
        action = control()
        manager = Manager()
        manager.events = events
        manager._agent_wrapper = lambda: events.append("agent") or action
        manager.ego_vehicles = [
            NS(apply_control=lambda c: (self.assertIs(c, action), events.append("apply")))
        ]
        instrumented = instrument_submit(
            Manager._tick_scenario, lambda m, c: events.append("logged")
        )
        instrumented(manager)
        self.assertEqual(events, ["before", "agent", "apply", "logged", "after"])

    def test_hook_not_called_after_failed_submission(self):
        events = []
        manager = Manager()
        manager.events = events
        manager._agent_wrapper = control

        def failed(control):
            raise RuntimeError("RPC failure")

        manager.ego_vehicles = [NS(apply_control=failed)]
        with self.assertRaisesRegex(RuntimeError, "RPC failure"):
            instrument_submit(Manager._tick_scenario, lambda m, c: events.append("logged"))(manager)
        self.assertNotIn("logged", events)

    def test_unsupported_hook_shape_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "found 2"):
            instrument_submit(Manager.duplicate, lambda m, c: None)

    def test_no_record_when_no_submission(self):
        route, world, agent = self.make_run(length=0)
        self.assert_error("call_count")

    def test_route_identity_mixing_detected(self):
        self.make_run()
        rows = self.rows()
        rows[1]["route_id"] = "another-route"
        self.save_rows(rows)
        self.assert_error("mixed_identity")

    def test_real_logging_adapter_calls_parent_once(self):
        self.make_run(length=0)
        info = json.loads((self.ticks.parent / "route.json").read_text())
        path = Path(__file__).resolve().parents[1] / "src/ksae_2026_autumn/logging_agent.py"
        spec = importlib.util.spec_from_file_location("_telemetry_test_adapter", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"sensor_agent": NS(SensorAgent=Agent)}):
            spec.loader.exec_module(module)
        agent = module.LoggingSensorAgent()
        context = RouteTelemetry(self.root / "adapter", info, True, World(), Actor())
        agent._telemetry_context = context
        returned = agent.run_step({"rgb": (100, None), "lidar": (100, None)}, 0.05)
        self.assertEqual(agent.calls, 1)
        self.assertIs(returned, agent.output)
        context.submitted(returned, 100)
        context.close("stop_scenario")

    def test_pending_command_is_reported_on_close(self):
        self.make_run(length=0)
        info = json.loads((self.ticks.parent / "route.json").read_text())
        agent = Agent()
        context = RouteTelemetry(self.root / "pending", info, True, World(), Actor())
        context.call_agent(agent.run_step, agent, {"rgb": (100, None), "lidar": (100, None)}, 0.05)
        context.close("stop_scenario")
        summary = json.loads((self.root / "pending/summary.json").read_text())
        self.assertEqual(summary["agent_returns"], 1)
        self.assertEqual(summary["submit_calls"], 0)
        self.assertTrue(summary["errors"])


if __name__ == "__main__":
    unittest.main()
