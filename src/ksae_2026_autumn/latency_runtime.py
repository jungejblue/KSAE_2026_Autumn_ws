"""Apply E2E command delay after TF++ returns and before evaluator submission."""

import json
import time

from ksae_2026_autumn.action_delay import ActionDelay, DelayConfig
from ksae_2026_autumn.telemetry import write_json


def attach_delay(context, config, control_factory):
    """Keep raw E2E output in telemetry; return the selected buffered control."""
    channel = ActionDelay(config)
    context._action_delay = channel
    context.metadata["branch_id"] = f"delay_{config.delay_ms}ms"
    context.metadata["action_delay"] = config.to_dict()
    context.metadata["semantics"] = (
        "State at frame k precedes submission at k; e2e_control is newly generated, "
        "selected/submitted_control is the buffered command. RPC evidence, not physics ACK."
    )
    write_json(context.directory / "route.json", context.metadata)
    original_call = context.call_agent

    def call(callback, agent, input_data, timestamp):
        try:
            original_call(callback, agent, input_data, timestamp)
            row = context.pending
            start = time.perf_counter()
            current_source = row["action_source_frame"]
            delay = channel.step(row["frame"], row["e2e_control"], current_source)
            output = control_factory()
            for key, value in delay["output_control"].items():
                setattr(output, key, value)
            row["current_e2e_action_source_frame"] = current_source
            row["action_source_frame"] = delay["selected_source_frame"]
            row["action_observation_age_s"] = (
                (row["frame"] - delay["selected_source_frame"]) * 0.05
                if delay["selected_source_frame"] is not None
                else None
            )
            row["injected_delay_ticks"] = delay["actual_age_ticks"]
            row["action_delay"] = delay
            row["delay_wall_ms"] = (time.perf_counter() - start) * 1000
            return output
        except BaseException as error:
            context.errors.append(f"action delay: {type(error).__name__}: {error}")
            raise

    context.call_agent = call


def main():
    import carla
    from leaderboard.scenarios import scenario_manager

    from ksae_2026_autumn import telemetry_runtime as base
    from ksae_2026_autumn import violation_runtime

    original_install = base.install_hooks

    def install(output):
        cleanup = original_install(output)
        config = DelayConfig(**json.loads((output / "run.json").read_text())["action_delay"])
        cls = scenario_manager.ScenarioManager
        old_load = cls.load_scenario

        def load(self, scenario, agent, route_index, rep_number):
            result = old_load(self, scenario, agent, route_index, rep_number)
            attach_delay(self._telemetry_context, config, carla.VehicleControl)
            return result

        cls.load_scenario = load
        return cleanup

    base.install_hooks = install
    # Its load wrapper runs after ours, so detector identity inherits the correct branch.
    violation_runtime.main()


if __name__ == "__main__":
    main()
