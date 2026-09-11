"""Run the controlled lead-stop scenario with E2E/fallback handover and recovery."""

import ast
import copy
import hashlib
import json
import math
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

from .dual_control import Prefix, Protocol, Settings, Stall, state_error
from .telemetry import write_json as write

GARAGE_EVALUATOR_SHA256 = "685b57298b953beec3a7f8c40445a6cca980163e7208a1c8257762fbc104d24b"
GARAGE_ROUTE_SHA256 = "6040c2abe83a738bd434c87751f92505bc481deafff68f7fb1826acf11fc4240"


def adapt_source(source):
    tree = ast.parse(source)
    original = ast.parse("scenario_name = config.scenario_configs[0].name").body[0]
    replacement = ast.parse("""scenario_name = (
        "ControlledLeadStop" if config.name == "RouteScenario_70001" and not config.scenario_configs
        else config.scenario_configs[0].name
    )""").body[0]
    hits = 0
    for cls in tree.body:
        if not isinstance(cls, ast.ClassDef) or cls.name != "LeaderboardEvaluator":
            continue
        for method in cls.body:
            if not isinstance(method, ast.FunctionDef) or method.name != "_load_and_run_scenario":
                continue
            for i, statement in enumerate(method.body):
                if ast.dump(statement) == ast.dump(original):
                    method.body[i] = ast.copy_location(replacement, statement)
                    hits += 1
    if hits != 1:
        raise ValueError(f"Expected one statistics-name assignment, found {hits}")
    ast.fix_missing_locations(tree)
    return ast.unparse(tree) + "\n"


def prepare_evaluator(original, output):
    original, output = Path(original), Path(output)
    before = original.read_bytes()
    digest = hashlib.sha256(before).hexdigest()
    if digest != GARAGE_EVALUATOR_SHA256:
        raise ValueError("Unsupported evaluator source; no adaptation performed")
    meta = json.loads((output / "run.json").read_text())
    if meta["expected_route_ids"] != ["RouteScenario_70001_rep0"]:
        raise ValueError("Adaptation is restricted to the bundled custom route")
    folder = output / "runtime"
    folder.mkdir(exist_ok=False)
    target = folder / "leaderboard_evaluator_fixture.py"
    adapted = adapt_source(before.decode("utf-8"))
    windowed = os.environ.get("KSAE_DUAL_WINDOWED") == "1"
    if windowed:
        if adapted.count("-RenderOffScreen") != 1:
            raise ValueError("Expected exactly one offscreen launch option")
        adapted = adapted.replace("-RenderOffScreen", "-windowed -ResX=1280 -ResY=720", 1)
    compile(adapted, str(target), "exec")
    target.write_text(adapted)
    write(
        folder / "adaptation.json",
        {
            "original_path": str(original),
            "original_sha256": digest,
            "runtime_path": str(target),
            "runtime_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            "scope": "custom route statistics label; optional windowed visualization",
            "windowed": windowed,
            "original_file_modified": False,
            "fallback_modified": False,
        },
    )
    meta["evaluator_runtime_adaptation"] = str(folder / "adaptation.json")
    write(output / "run.json", meta)
    return target


def clear_parking_slots(scenario, original):
    if scenario.config.name == "RouteScenario_70001":
        scenario.available_parking_locations = []
        return None
    return original(scenario)


def main():
    import carla
    from leaderboard.scenarios import route_scenario, scenario_manager
    from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle

    from fallback_control.carla_adapter import CarlaFallback
    from fallback_control.types import Config
    from ksae_2026_autumn import telemetry_runtime as base
    from ksae_2026_autumn import violation_runtime
    from ksae_2026_autumn.telemetry import control_dict

    source = Path(route_scenario.__file__)
    if hashlib.sha256(source.read_bytes()).hexdigest() != GARAGE_ROUTE_SHA256:
        raise RuntimeError("Unsupported Garage route_scenario.py; no scenario patch applied")
    # This process runs only the bundled custom route. No ambient traffic is used.
    route_scenario.BackgroundBehavior = lambda *args, **kwargs: Idle()
    original_parking = route_scenario.RouteScenario._get_parking_slots
    route_scenario.RouteScenario._get_parking_slots = lambda scenario: clear_parking_slots(
        scenario, original_parking
    )
    original_install = base.install_hooks
    mode = os.environ["KSAE_DUAL_MODE"]
    scenario_config = Path(os.environ["KSAE_DUAL_CONFIG"])
    settings = json.loads(scenario_config.read_text())
    protocol_settings = Settings(**settings["protocol"])
    cfg = Config(**json.loads((scenario_config.parent / settings["fallback_config"]).read_text()))

    class ObservedFallback(CarlaFallback):
        def _state(self, snap):
            state, bound = super()._state(snap)
            self.recorded_input = {"ego": asdict(state), "steer_bound": bound}
            return state, bound

        def _obstacles(self, snapshot, ego, ego_z):
            obs = super()._obstacles(snapshot, ego, ego_z)
            self.recorded_obstacles = [asdict(o) for o in obs]
            return obs

    def install(output):
        metadata = json.loads((output / "run.json").read_text())
        metadata["action_timestamp_note"] = (
            "Launcher FIFO is disabled. The compute-stall channel holds the last actual "
            "TF++ command for 3s; generated/source frames and actual applied age are logged."
        )
        metadata["controlled_fixture"] = {
            "mode": mode,
            "route": "70001",
            "official_benchmark": False,
            "fault_onset": (
                "eligible, gap <=50 m, speed >=6 m/s, previous throttle >=0.2 and brake <=0.01"
            ),
            "delay_metadata_note": (
                "launcher FIFO disabled; 3s hold-last-command compute stall recorded in fault.json"
            ),
            "forced_takeover_after_s": 0.75,
            "fault_kind": "hold_last_command_compute_stall",
            "horizon_s": 5.0,
            "lead_departure_after_onset_s": 18.0,
            "controller_commit": metadata["research_commit"],
        }
        write(output / "run.json", metadata)
        cleanup = original_install(output)
        cls = scenario_manager.ScenarioManager
        old_load = cls.load_scenario

        def load(self, scenario, agent, route_index, rep_number):
            if scenario.config.name != "RouteScenario_70001":
                raise RuntimeError("Dual runtime only accepts bundled RouteScenario_70001")
            ego = scenario.ego_vehicles[0]
            world = ego.get_world()
            if len(world.get_actors().filter("vehicle.*")) != 1:
                raise RuntimeError("Controlled fixture requires ego only before lead spawn")
            pt = settings["lead_spawn"]
            wp = world.get_map().get_waypoint(carla.Location(x=pt["x"], y=pt["y"], z=pt["z"]))
            if wp is None or wp.road_id != pt["road"] or wp.lane_id != pt["lane"]:
                raise RuntimeError("Fixture map/road mismatch")
            bp = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
            bp.set_attribute("role_name", "controlled_lead")
            tf = wp.transform
            tf.location.z += 0.3
            lead = world.try_spawn_actor(bp, tf)
            if lead is None:
                raise RuntimeError("Lead spawn failed; do not count this as a successful trial")
            scenario.other_actors.append(lead)
            lead.apply_control(carla.VehicleControl(brake=1.0))
            for light in world.get_actors().filter("traffic.traffic_light*"):
                light.set_state(carla.TrafficLightState.Green)
                light.freeze(True)
            result = old_load(self, scenario, agent, route_index, rep_number)
            ctx = self._telemetry_context
            fb = ObservedFallback(ego, scenario.route, cfg)
            protocol = Protocol(mode, protocol_settings)
            channel = Stall(mode != "clean", protocol.c.stall_ticks)
            reference = (
                Prefix(os.environ["KSAE_DUAL_REFERENCE_TICKS"]) if mode == "takeover" else None
            )
            if reference:
                write(
                    ctx.directory / "prefix_reference.json",
                    {
                        "path": str(reference.path),
                        "sha256": hashlib.sha256(reference.path.read_bytes()).hexdigest(),
                        "onset_tick": reference.onset_tick,
                        "candidate_tick": reference.onset_tick + protocol.c.takeover_after_ticks,
                        "method": (
                            "replay submitted ego controls until candidate; "
                            "check observed ego/lead states"
                        ),
                        "full_snapshot_restore": False,
                    },
                )
            first_frame = None
            ctx.metadata.update(
                branch_id="dual_" + mode,
                fixture="stationary_lead_then_depart",
                fallback_information_source="carla_ground_truth",
                ground_truth_usage="fallback and fixture only; TF++ retains original sensor inputs",
                forced_switch_scenario=True,
                settings=asdict(protocol.c),
            )
            write(ctx.directory / "route.json", ctx.metadata)
            write(
                ctx.directory / "fallback_context.json",
                {
                    "config": asdict(cfg),
                    "geometry": asdict(fb.geometry),
                    "route_xy_rh_m": fb.route.xy.tolist(),
                    "route_station_m": fb.route.s.tolist(),
                    "route_yaw_rad": fb.route.yaw.tolist(),
                    "route_curvature_per_m": fb.route.kappa.tolist(),
                    "route_width_m": fb.route.width.tolist(),
                    "unsupported_stations_m": fb.route.unsupported_s,
                },
            )
            write(
                ctx.directory / "fixture.json",
                {
                    "mode": mode,
                    "lead_id": lead.id,
                    "lead_spawn": pt,
                    "background_traffic": False,
                    "traffic_lights": "frozen_green",
                    "physics": "normal",
                    "avoidance": "braking/stop in the original lane; no lane change",
                    "settings": asdict(protocol.c),
                    "controller_commit": metadata["research_commit"],
                },
            )
            original_call = ctx.call_agent

            def call(callback, active_agent, input_data, timestamp):
                nonlocal first_frame
                start = time.perf_counter()
                original_call(callback, active_agent, input_data, timestamp)
                row = ctx.pending
                snap = world.get_snapshot()
                frame = row["frame"]
                if snap.frame != frame:
                    raise RuntimeError("World advanced during E2E")
                if first_frame is None:
                    first_frame = frame
                tick = frame - first_frame
                # End normally so a blocked collision baseline still retains complete logs.
                if tick >= settings["time_limit_ticks"]:
                    self._running = False
                row["fixture_time_limit"] = tick >= settings["time_limit_ticks"]
                et, lt = (
                    snap.find(ego.id).get_transform(),
                    snap.find(lead.id).get_transform(),
                )
                fwd = et.get_forward_vector()

                def project(p):
                    return p.x * fwd.x + p.y * fwd.y

                gap = min(project(p) for p in lead.bounding_box.get_world_vertices(lt)) - max(
                    project(p) for p in ego.bounding_box.get_world_vertices(et)
                )
                speed = row["ego"]["speed_mps"]
                base_eligible = not row["warmup"] and not row["initial_control_delay_active"]
                previous = channel.previous
                armed = (
                    previous is not None
                    and previous[0]["throttle"] >= 0.2
                    and previous[0]["brake"] <= 0.01
                )
                eligible = base_eligible and armed
                prefix_row = None
                replay_info = None
                effective_control = row["e2e_control"]
                effective_source = row["action_source_frame"]
                trigger_override = None
                if reference:
                    trigger_override = tick == reference.onset_tick
                    candidate_tick = reference.onset_tick + protocol.c.takeover_after_ticks
                    if tick <= candidate_tick:
                        prefix_row = reference.row(tick)
                        replay_info = state_error(row, prefix_row)
                        replay_info.update(
                            reference_tick=tick, controls_replayed=tick < candidate_tick
                        )
                        effective_control = prefix_row["e2e_control"]
                        effective_source = (
                            first_frame
                            + prefix_row["current_e2e_action_source_frame"]
                            - reference.first
                        )
                        # Record current TF++ output separately from replayed prefix commands.
                        row["raw_tfpp_control"] = copy.deepcopy(row["e2e_control"])
                        row["raw_tfpp_action_source_frame"] = row["action_source_frame"]
                        eligible = prefix_row["dual"]["eligible"]
                onset = protocol.trigger is None and (
                    trigger_override
                    if trigger_override is not None
                    else (
                        eligible
                        and 0 < gap <= protocol.c.trigger_gap_m
                        and speed >= protocol.c.trigger_min_speed_mps
                    )
                )
                if onset:
                    write(
                        ctx.directory / "fault.json",
                        {
                            "frame": frame,
                            "route_tick": tick,
                            "gap_m": gap,
                            "speed_mps": speed,
                            "kind": "hold_last_command_compute_stall",
                            "duration_ticks": protocol.c.stall_ticks,
                            "enabled": mode != "clean",
                            "candidate_after_ticks": protocol.c.takeover_after_ticks,
                            "held_actual_previous_e2e_control": previous[0],
                            "horizon_ticks": protocol.c.horizon_ticks,
                        },
                    )
                delay = channel.step(frame, effective_control, effective_source, onset)
                e2e = carla.VehicleControl(**delay["output_control"])
                # Reset only unapplied controller history; retain observed route progress.
                cold = protocol.takeover is None or protocol.recovery is not None
                if cold:
                    progress = fb.route._progress
                    fb.controller.reset()
                    fb.route._progress = progress
                    fb.actuator.reset()
                begin = time.perf_counter()
                candidate = fb.run_step(snap)
                wall_ms = (time.perf_counter() - begin) * 1000
                diag = copy.deepcopy(fb.last_diagnostics)
                diag["controller_input"] = getattr(fb, "recorded_input", {})
                diag["obstacle_inputs"] = getattr(fb, "recorded_obstacles", [])
                if diag.get("frame") != frame or world.get_snapshot().frame != frame:
                    raise RuntimeError("Fallback frame mismatch")
                good = (
                    diag.get("emergency") is False
                    and diag.get("reason") == "sampled_mpc"
                    and diag.get("valid_candidates", 0) > 0
                    and wall_ms <= 50.0
                )
                fresh = delay["actual_age_ticks"] == 0 and delay["selected_source_frame"] == frame
                choice = protocol.step(frame, eligible, gap, speed, fresh, good, trigger_override)
                velocity = snap.find(lead.id).get_velocity()
                lead_speed = math.hypot(velocity.x, velocity.y)
                if choice["lead_released"]:
                    yaw_error = math.atan2(
                        math.sin(math.radians(pt["yaw"] - lt.rotation.yaw)),
                        math.cos(math.radians(pt["yaw"] - lt.rotation.yaw)),
                    )
                    lead_control = carla.VehicleControl(
                        throttle=max(0.0, min(0.6, 0.3 * (8.0 - lead_speed))),
                        brake=max(0.0, min(0.2, 0.2 * (lead_speed - 8.0))),
                        steer=max(-0.3, min(0.3, yaw_error)),
                    )
                else:
                    lead_control = carla.VehicleControl(brake=1.0)
                lead.apply_control(
                    lead_control
                )  # Only the lead; evaluator submits ego exactly once.
                chosen = candidate if choice["selected"] == "FALLBACK" else e2e
                if replay_info and replay_info["controls_replayed"]:
                    chosen = carla.VehicleControl(**prefix_row["submitted_control"])
                row["current_e2e_action_source_frame"] = effective_source
                row["e2e_control"] = copy.deepcopy(effective_control)
                row["action_delay"] = delay
                row["dual"] = {
                    **choice,
                    "mode": mode,
                    "eligible": eligible,
                    "base_eligible": base_eligible,
                    "trigger_override": trigger_override,
                    "prefix_replay": replay_info,
                    "lead_gap_m": gap,
                    "lead_speed_mps": lead_speed,
                    "ego_speed_mps": speed,
                    "e2e_fresh": fresh,
                    "fallback_verified": good,
                    "fallback_frame": frame,
                    "fallback_control": control_dict(candidate),
                    "buffered_e2e_control": control_dict(e2e),
                    "fallback_diagnostics": diag,
                    "fallback_wall_ms": wall_ms,
                    "cold_control_history": cold,
                    "lead_control": control_dict(lead_control),
                    "integration_wall_ms": (time.perf_counter() - start) * 1000,
                }
                row["controller"] = choice["selected"]
                row["action_source_frame"] = (
                    frame if choice["selected"] == "FALLBACK" else delay["selected_source_frame"]
                )
                row["injected_delay_ticks"] = (
                    0 if choice["selected"] == "FALLBACK" else delay["actual_age_ticks"]
                )
                row["action_observation_age_s"] = (
                    (frame - row["action_source_frame"]) * 0.05
                    if row["action_source_frame"] is not None
                    else None
                )
                if choice["event"]:
                    print(
                        f"[DUAL] tick={tick} event={choice['event']} "
                        f"controller={choice['selected']} gap={gap:.2f}m "
                        f"speed={speed:.2f}m/s age={delay['actual_age_ticks'] * 0.05:.2f}s",
                        flush=True,
                    )
                return chosen

            ctx.call_agent = call
            return result

        cls.load_scenario = load
        return cleanup

    base.install_hooks = install
    sys.argv[1] = str(prepare_evaluator(sys.argv[1], Path(os.environ["KSAE_TELEMETRY_OUTPUT"])))
    # Retain evaluator criterion implementations to explain auxiliary benchmark warnings.
    import importlib

    audit_dir = Path(os.environ["KSAE_TELEMETRY_OUTPUT"]) / "runtime"
    for module_name, filename in (
        (
            "srunner.scenariomanager.scenarioatomics.atomic_criteria",
            "atomic_criteria_source.py",
        ),
        ("leaderboard.utils.statistics_manager", "statistics_manager_source.py"),
    ):
        module = importlib.import_module(module_name)
        (audit_dir / filename).write_bytes(Path(module.__file__).read_bytes())
    violation_runtime.main()


if __name__ == "__main__":
    main()
