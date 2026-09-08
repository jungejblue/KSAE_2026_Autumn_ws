"""TF++ adapter. The matching telemetry runtime attaches the route context."""

from sensor_agent import SensorAgent


def get_entry_point():
    return "LoggingSensorAgent"


class LoggingSensorAgent(SensorAgent):
    def run_step(self, input_data, timestamp, sensors=None):
        context = getattr(self, "_telemetry_context", None)
        if context is None:
            raise RuntimeError("Launch this agent using tools/run_telemetry.py")
        return context.call_agent(super().run_step, self, input_data, timestamp)
