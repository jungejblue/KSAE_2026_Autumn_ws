"""Process-local observation hooks; original Garage files remain unchanged."""

import ast
import atexit
import functools
import inspect
import json
import os
import runpy
import sys
import textwrap
from pathlib import Path

from ksae_2026_autumn.telemetry import RouteTelemetry, write_json


def instrument_submit(function, callback):
    """Insert one observation after the original apply_control call.

    Source identity is checked by the launcher before this is used. The AST
    transformation cannot add a second model call or world tick.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(function)))
    expected = ast.dump(ast.parse("self.ego_vehicles[0].apply_control(ego_action)").body[0])

    class Insert(ast.NodeTransformer):
        count = 0

        def visit_Expr(self, node):
            if ast.dump(node) == expected:
                self.count += 1
                addition = ast.parse("_ksae_after_submit(self, ego_action)").body[0]
                return [node, ast.copy_location(addition, node)]
            return self.generic_visit(node)

    transform = Insert()
    tree = ast.fix_missing_locations(transform.visit(tree))
    if transform.count != 1:
        raise RuntimeError(f"Expected one apply_control call, found {transform.count}")
    namespace = dict(function.__globals__, _ksae_after_submit=callback)
    exec(compile(tree, inspect.getsourcefile(function), "exec"), namespace)
    return functools.wraps(function)(namespace[function.__name__])


def install_hooks(output):
    from leaderboard.scenarios import scenario_manager as module
    from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
    from srunner.scenariomanager.timer import GameTime

    metadata = json.loads((output / "run.json").read_text())
    sessions = []
    cls = module.ScenarioManager
    original_load, original_stop = cls.load_scenario, cls.stop_scenario

    @functools.wraps(original_load)
    def load(self, scenario, agent, route_index, rep_number):
        result = original_load(self, scenario, agent, route_index, rep_number)
        world = CarlaDataProvider.get_world()
        settings = world.get_settings()
        if not settings.synchronous_mode or abs(settings.fixed_delta_seconds - 0.05) > 1e-5:
            raise RuntimeError("Telemetry requires synchronous simulation with fixed delta 0.05")
        route_id = f"{scenario.config.name}_rep{rep_number}"
        if Path(route_id).name != route_id:
            raise ValueError("Invalid route identifier")
        info = {
            "run_id": metadata["run_id"],
            "route_id": route_id,
            "branch_id": "clean",
            "repetition": rep_number,
            "route_index": route_index,
            "town": str(scenario.config.town),
            "ego_id": int(self.ego_vehicles[0].id),
            "ego_type": self.ego_vehicles[0].type_id,
            "seed": metadata["seed"],
            "sensor_specs": agent.sensors(),
            "sensor_frame_policy": "exact_current_frame",
            "simulation": {
                key: getattr(settings, key)
                for key in (
                    "synchronous_mode",
                    "fixed_delta_seconds",
                    "substepping",
                    "max_substeps",
                    "max_substep_delta_time",
                )
            },
            "scenario_names": [item.name for item in scenario.config.scenario_configs],
            "units": {
                "position": "m",
                "velocity": "m/s",
                "acceleration": "m/s^2",
                "rotation": "radian",
                "angular_velocity": "radian/s",
            },
            "semantics": "State at frame k precedes command submitted at k; effects appear later.",
            "ground_truth_usage": "logging only; no additional data enters the TF++ policy",
            "carla_client_version": CarlaDataProvider.get_client().get_client_version(),
            "carla_server_version": CarlaDataProvider.get_client().get_server_version(),
        }
        context = RouteTelemetry(
            output / "routes" / route_id,
            info,
            metadata["logging_enabled"],
            world,
            self.ego_vehicles[0],
        )
        sessions.append(context)
        self._telemetry_context = agent._telemetry_context = context
        return result

    def after_submit(self, control):
        self._telemetry_context.submitted(control, GameTime.get_frame())

    @functools.wraps(original_stop)
    def stop(self):
        # Close before the original cleanup performs its sensor-destruction tick.
        context = getattr(self, "_telemetry_context", None)
        if context is not None:
            context.close("stop_scenario")
        return original_stop(self)

    def close_remaining():
        for context in sessions:
            context.close("process_exit")

    cls.load_scenario = load
    cls.stop_scenario = stop
    cls._tick_scenario = instrument_submit(cls._tick_scenario, after_submit)
    atexit.register(close_remaining)
    return close_remaining


def main():
    output = Path(os.environ["KSAE_TELEMETRY_OUTPUT"])
    evaluator = Path(sys.argv[1])
    metadata = json.loads((output / "run.json").read_text())

    def close():
        pass

    try:
        import carla
        import numpy
        import torch
        from leaderboard.scenarios import scenario_manager
        from srunner.scenariomanager import carla_data_provider

        metadata["runtime"] = {
            "python": sys.executable,
            "python_version": sys.version,
            "numpy": numpy.__version__,
            "torch": torch.__version__,
            "torch_cuda": torch.version.cuda,
            "cuda_available": torch.cuda.is_available(),
            "carla_module": carla.__file__,
            "scenario_manager": scenario_manager.__file__,
            "carla_data_provider": carla_data_provider.__file__,
        }
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable in the selected Python environment")
        metadata["runtime"]["gpu"] = torch.cuda.get_device_name(0)
        metadata["runtime"]["gpu_memory_bytes"] = torch.cuda.get_device_properties(0).total_memory
        write_json(output / "run.json", metadata)
        close = install_hooks(output)
        sys.argv = [str(evaluator), *sys.argv[2:]]
        runpy.run_path(str(evaluator), run_name="__main__")
    finally:
        close()


if __name__ == "__main__":
    main()
