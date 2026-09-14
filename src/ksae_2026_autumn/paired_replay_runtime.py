"""Reconstruct a candidate by route-start replay, then run E or F for three seconds."""

import copy
import hashlib
import json
import math
import os
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

from .dual_control import Prefix, Protocol, Settings, Stall, state_error
from .dual_runtime import GARAGE_ROUTE_SHA256, clear_parking_slots, prepare_evaluator
from .replay_candidate_input import (
    call_candidate,
    common_packet,
    fixed_backend,
    save_packet,
)
from .replay_checkpoint import capture, compare, restore_agent_rng
from .telemetry import write_json as write


def main():
    import random

    import carla
    import numpy as np
    import torch
    from leaderboard.scenarios import route_scenario, scenario_manager
    from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle

    from fallback_control.carla_adapter import CarlaFallback
    from fallback_control.types import Config
    from ksae_2026_autumn import telemetry_runtime as base
    from ksae_2026_autumn import violation_runtime
    from ksae_2026_autumn.telemetry import control_dict

    seed = int(os.environ["KSAE_REPLAY_SEED"])
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True)
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
    case_name = os.environ["KSAE_REPLAY_CASE"]
    mode = "takeover" if case_name.startswith("F") else "delayed"
    scenario_config = Path(os.environ["KSAE_DUAL_CONFIG"])
    settings = json.loads(scenario_config.read_text())
    protocol_settings = replace(Settings(**settings["protocol"]), horizon_ticks=60)
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
            "horizon_s": 3.0,
            "lead_departure_after_onset_s": 18.0,
            "controller_commit": metadata["research_commit"],
        }
        metadata["replay_experiment"] = {
            "case": case_name,
            "seed": seed,
            "horizon_s": 3.0,
            "method": "route_start_control_prefix_reconstruction",
            "full_simulator_snapshot": False,
            "recovery_enabled_in_horizon": False,
            "torch_deterministic_algorithms": True,
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
                Prefix(os.environ["KSAE_DUAL_REFERENCE_TICKS"]) if case_name != "E0" else None
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
                branch_id="replay_" + case_name,
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
            backend = None
            original_call = ctx.call_agent

            def call(callback, active_agent, input_data, timestamp):
                nonlocal first_frame, backend
                start = time.perf_counter()
                backend = fixed_backend()
                pre_snapshot = world.get_snapshot()
                if first_frame is None:
                    first_frame = int(pre_snapshot.frame)
                tick_before = int(pre_snapshot.frame) - first_frame
                onset_tick = (
                    reference.onset_tick
                    if reference
                    else None
                    if protocol.trigger is None
                    else protocol.trigger - first_frame
                )
                candidate_tick_before = (
                    None if onset_tick is None else onset_tick + protocol.c.takeover_after_ticks
                )
                if candidate_tick_before is not None and tick_before == candidate_tick_before:
                    # All branches begin with the same cold, unapplied fallback history.
                    progress = fb.route._progress
                    fb.controller.reset()
                    fb.route._progress = progress
                    fb.actuator.reset()
                    if reference:
                        capture(
                            ctx.directory / "before_restore",
                            active_agent,
                            fb,
                            channel,
                            pre_snapshot,
                            first_frame,
                            onset_tick,
                            input_data,
                        )
                        reference_folder = reference.path.parent
                        reference_sha = restore_agent_rng(
                            reference_folder, active_agent, candidate_tick_before
                        )
                    if reference:
                        input_data = common_packet(
                            reference_folder, candidate_tick_before, input_data
                        )
                    save_packet(ctx.directory, candidate_tick_before, input_data)
                    capture(
                        ctx.directory,
                        active_agent,
                        fb,
                        channel,
                        pre_snapshot,
                        first_frame,
                        onset_tick,
                        input_data,
                    )
                    if reference:
                        after = compare(reference_folder, ctx.directory)
                        write(
                            ctx.directory / "restoration.json",
                            {
                                "reference_checkpoint_sha256": reference_sha,
                                "candidate_tick": candidate_tick_before,
                                "restored_groups": ["agent", "rng"],
                                "before": compare(
                                    reference_folder, ctx.directory / "before_restore"
                                ),
                                "after": after,
                                "post_candidate_inputs": "live",
                            },
                        )
                        if not after["match"]:
                            raise RuntimeError(
                                "Candidate state mismatch after software restoration"
                            )
                is_candidate = (
                    candidate_tick_before is not None and tick_before == candidate_tick_before
                )
                if is_candidate:
                    call_candidate(
                        original_call,
                        callback,
                        active_agent,
                        input_data,
                        timestamp,
                        ctx.directory,
                        tick_before,
                        backend,
                    )
                else:
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
                candidate_tick_now = (
                    None if onset_tick is None else onset_tick + protocol.c.takeover_after_ticks
                )
                horizon_done = (
                    candidate_tick_now is not None
                    and tick >= candidate_tick_now + protocol.c.horizon_ticks
                )
                if horizon_done:
                    self._running = False
                row["replay"] = {
                    "case": case_name,
                    "route_tick": tick,
                    "candidate_tick": candidate_tick_now,
                    "horizon_ticks": 60,
                    "horizon_done": horizon_done,
                    "prefix_controls_replayed": reference is not None
                    and candidate_tick_now is not None
                    and tick < candidate_tick_now,
                    "post_candidate_live": candidate_tick_now is not None
                    and tick > candidate_tick_now,
                    "candidate_input_shared": is_candidate,
                }
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
                    if tick < candidate_tick:
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
