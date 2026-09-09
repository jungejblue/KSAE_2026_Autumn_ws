"""CARLA 0.9.15 privileged research adapter; never calls world.tick/apply_control.

All controller coordinates are RH: (x_CARLA, -y_CARLA, -yaw_CARLA).
Use the full route passed to set_global_plan BEFORE Garage downsamples it.
"""

import math
import time

import numpy as np

from .actuation import LongitudinalActuator
from .controller import FallbackController
from .route import Route, wrap
from .steering import equivalent_from_inner, front_track, normalized_from_equivalent
from .types import Command, Config, EgoState, Geometry, Obstacle


def geometry_from_vehicle(vehicle):
    from ksae_2026_autumn.violation_geometry import validate_bbox

    profile = validate_bbox(vehicle)
    physics = vehicle.get_physics_control()
    if len(physics.wheels) != 4:
        raise ValueError("Exactly four wheels required")
    # Fixed vehicle-local wheel reference; never reinterpret UE world wheel coordinates.
    local = np.asarray(profile["local_m"], dtype=float)
    order = np.argsort(local[:, 0])
    rear_x = float(np.mean(local[order[:2], 0]))
    front_x = float(np.mean(local[order[2:], 0]))
    wb = front_x - rear_x
    if not 1.5 < wb < 4.5 or np.max(np.abs(local)) > 8:
        raise ValueError("Invalid fixed MKZ wheel profile")
    bb = vehicle.bounding_box
    # Enclose even a rotated bounding box in actor-local XY.
    vertices = bb.get_local_vertices()
    bx = np.array([p.x for p in vertices])
    by = np.array([-p.y for p in vertices])
    front, rear = float(bx.max() - rear_x), float(rear_x - bx.min())
    front_wheels = [physics.wheels[int(i)] for i in order[2:]]
    angle = math.radians(min(w.max_steer_angle for w in front_wheels))
    geo = Geometry(
        wb,
        rear_x,
        front,
        rear,
        float((by.max() - by.min()) / 2),
        angle,
        float((by.max() + by.min()) / 2),
        tuple((float(p[0] - rear_x), float(-p[1])) for p in local),
    )
    return geo, physics


def dense_route_from_plan(carla_map, full_world_plan):
    import carla

    if len(full_world_plan) < 3:
        raise ValueError("Full world route missing")
    points, widths, unsupported = [], [], []
    accum = 0.0
    previous_wp = None
    for transform, command in full_world_plan:
        loc = transform.location
        wp = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            raise ValueError("Route waypoint missing")
        center = wp.transform.location
        if math.hypot(center.x - loc.x, center.y - loc.y) > 1.5:
            raise ValueError("Route is not a dense lane-center route")
        p = np.array([center.x, -center.y])
        if points:
            gap = float(np.linalg.norm(p - points[-1]))
            if gap < 0.02:
                continue
            if gap > 3.0:
                raise ValueError(
                    "Sparse/discontinuous route: use original dense set_global_plan input"
                )
            accum += gap
        name = getattr(command, "name", str(command)).upper()
        lane_jump = (
            previous_wp is not None
            and wp.road_id == previous_wp.road_id
            and wp.lane_id != previous_wp.lane_id
        )
        if "CHANGELANE" in name or lane_jump:
            unsupported.append(accum)
        points.append(p)
        widths.append(float(wp.lane_width))
        previous_wp = wp
    return Route(points, widths, unsupported=unsupported)


class CarlaFallback:
    def __init__(self, ego_vehicle, full_world_plan, config=None, extra_static_obstacles=()):
        self.vehicle = ego_vehicle
        self.world = ego_vehicle.get_world()
        self.c = config or Config()
        self.actuator = LongitudinalActuator(self.c)
        settings = self.world.get_settings()
        if not settings.synchronous_mode or settings.fixed_delta_seconds is None:
            raise ValueError("Synchronous fixed-delta CARLA required")
        if abs(settings.fixed_delta_seconds - self.c.control_dt) > 1e-6:
            raise ValueError("Config.control_dt must equal CARLA fixed_delta_seconds")
        self.geometry, self.physics = geometry_from_vehicle(ego_vehicle)
        self.map = self.world.get_map()
        self.route = dense_route_from_plan(self.map, full_world_plan)
        self.controller = FallbackController(self.route, self.geometry, self.c)
        self.extra_static = tuple(extra_static_obstacles)
        self.last_frame = None
        self._invalid_episode = False
        self.last_time = None
        self.last_result = None
        self.last_diagnostics = {}
        self.state_diagnostics = {}
        self.traffic_lights = list(self.world.get_actors().filter("traffic.traffic_light*"))
        # Stop-line positions are cached; light states are read each tick.
        self.stop_lines = []
        for light in self.traffic_lights:
            for wp in light.get_stop_waypoints():
                loc = wp.transform.location
                xy = np.array([loc.x, -loc.y])
                distances = np.linalg.norm(self.route.xy - xy, axis=1)
                # Keep all matching route stations (including repeated traversals).
                ids = np.flatnonzero(distances < 1.5)
                for i in ids:
                    if abs(wrap(self.route.yaw[i] + math.radians(wp.transform.rotation.yaw))) < 0.5:
                        self.stop_lines.append((light, float(self.route.s[i])))
        self.stop_signs = list(self.world.get_actors().filter("traffic.stop*"))
        self.sign_wait = {}
        self.served_signs = set()

    def _effective_steer_bound(self, speed):
        curve = sorted((float(p.x), float(p.y)) for p in self.physics.steering_curve)
        # UE4 wheeled-vehicle steering curve x-axis is forward speed in km/h.
        factor = (
            float(np.interp(speed * 3.6, [p[0] for p in curve], [p[1] for p in curve]))
            if curve
            else 1.0
        )
        return self.geometry.max_steer * max(0.05, factor)

    def _state(self, actor_snapshot):
        import carla

        tf, vel = actor_snapshot.get_transform(), actor_snapshot.get_velocity()
        yaw = -math.radians(tf.rotation.yaw)
        # Low-speed planar bicycle model is not supported on steep slopes.
        if abs(tf.rotation.pitch) > 12 or abs(tf.rotation.roll) > 10:
            raise ValueError("unsupported_vehicle_attitude")
        x = tf.location.x + self.geometry.rear_axle_x * math.cos(yaw)
        y = -tf.location.y + self.geometry.rear_axle_x * math.sin(yaw)
        speed = vel.x * math.cos(yaw) - vel.y * math.sin(yaw)
        control = self.vehicle.get_control()
        if control.reverse or speed < -0.05:
            raise ValueError("reverse_not_supported")
        inner_bound = self._effective_steer_bound(max(0.0, speed))
        track = front_track(self.geometry)
        bound = equivalent_from_inner(inner_bound, self.geometry.wheelbase, track)
        # Read physical wheel angles; normalized VehicleControl is a request,
        # not a measurement of the current steering state.
        try:
            left = float(self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))
            right = float(self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel))
        except (AttributeError, RuntimeError) as exc:
            raise ValueError("wheel_steer_measurement_unavailable") from exc
        if not np.isfinite([left, right]).all() or max(abs(left), abs(right)) >= 85.0:
            raise ValueError("invalid_physical_wheel_angles")
        tl, tr = np.tan(np.radians([left, right]))
        tangent = 2.0 * tl * tr / (tl + tr) if tl * tr > 1e-8 else (tl + tr) / 2.0
        measured = -float(np.arctan(tangent))  # CARLA right-positive to RH left-positive
        mapped = equivalent_from_inner(
            -float(control.steer) * inner_bound, self.geometry.wheelbase, track
        )
        self.state_diagnostics = dict(
            physical_front_left_steer_deg=left,
            physical_front_right_steer_deg=right,
            physical_equivalent_steer_rad=measured,
            command_estimated_steer_rad=mapped,
            input_normalized_steer=float(control.steer),
            effective_steer_bound_rad=bound,
            inner_wheel_bound_rad=inner_bound,
            front_track_m=track,
            steering_mapping="mkz_inner_ackermann_inverse_v1",
            steering_mapping_residual_rad=measured - mapped,
            measured_yaw_rate_radps=-math.radians(actor_snapshot.get_angular_velocity().z),
            physical_steer_model_yaw_rate_radps=max(0.0, speed)
            / self.geometry.wheelbase
            * np.tan(measured),
            steering_state_source="physics_front_wheel_angles",
        )
        return EgoState(x, y, yaw, max(0.0, speed), measured), bound

    def _obstacles(self, snapshot, ego, ego_z):
        import carla

        obstacles = list(self.extra_static)
        for actor in self.world.get_actors():
            if actor.id == self.vehicle.id or not actor.type_id.startswith(
                ("vehicle.", "walker.pedestrian.", "static.prop.")
            ):
                continue
            snap = snapshot.find(actor.id)
            if snap is None:
                raise ValueError("actor_missing_from_snapshot")
            tf, vel = snap.get_transform(), snap.get_velocity()
            bb = actor.bounding_box
            center = tf.transform(carla.Location(bb.location.x, bb.location.y, bb.location.z))
            # Z overlap excludes bridges without ignoring elevated large obstacles.
            if abs(center.z - ego_z) > bb.extent.z + self.vehicle.bounding_box.extent.z + 1.0:
                continue
            radius = math.hypot(bb.extent.x, bb.extent.y)
            dist = math.hypot(center.x - ego.x, -center.y - ego.y)
            relative_speed = ego.speed + math.hypot(vel.x, vel.y)
            reach = (
                relative_speed * (self.c.times[-1] + self.c.reaction_time)
                + self.geometry.front
                + radius
                + 5
            )
            # The planner looks at least 40 m ahead, even at zero speed.
            # Shrinking this range while braking used to remove the very actor
            # that caused the stop, creating repeated acceleration/braking cycles.
            reach = max(reach, 40.0 + self.geometry.front + radius)
            if dist > reach:
                continue
            if dist > self.c.sensor_range:
                raise ValueError("required_obstacle_outside_configured_range")
            obstacles.append(
                Obstacle(actor.id, center.x, -center.y, vel.x, -vel.y, max(0.1, radius))
            )
        return obstacles

    def _traffic_stop(self, ego, s, sim_time):
        import carla

        stops = []
        for light, station in self.stop_lines:
            if s - 2 <= station <= s + 60 and light.get_state() in (
                carla.TrafficLightState.Red,
                carla.TrafficLightState.Yellow,
            ):
                stops.append(station - self.geometry.front - self.c.stop_buffer)
        for sign in self.stop_signs:
            if sign.id in self.served_signs:
                continue
            trigger = sign.get_transform().transform(
                carla.Location(
                    sign.trigger_volume.location.x,
                    sign.trigger_volume.location.y,
                    sign.trigger_volume.location.z,
                )
            )
            distances = np.linalg.norm(self.route.xy - [trigger.x, -trigger.y], axis=1)
            i = int(np.argmin(distances))
            station = float(self.route.s[i])
            if (
                distances[i]
                > max(sign.trigger_volume.extent.x, sign.trigger_volume.extent.y)
                + self.route.width[i] / 2
            ):
                continue
            if not s - 2 <= station <= s + 40:
                continue
            stop = station - self.geometry.front - self.c.stop_buffer
            if ego.speed < 0.15 and abs(s - stop) < 2.5:
                since = self.sign_wait.setdefault(sign.id, sim_time)
                if sim_time - since >= 2.0:
                    self.served_signs.add(sign.id)
                    continue
            else:
                self.sign_wait.pop(sign.id, None)
            stops.append(stop)
        return min(stops) if stops else None

    def run_step(self, snapshot=None):
        import carla

        start = time.perf_counter()
        snapshot = snapshot or self.world.get_snapshot()
        frame, now = int(snapshot.frame), float(snapshot.timestamp.elapsed_seconds)
        if self.last_frame == frame:
            return self.last_result
        cmd = Command(diagnostics={"reason": "adapter_not_ready"})
        self.state_diagnostics = {}
        try:
            if self.last_frame is not None and frame < self.last_frame:
                self._invalid_episode = True
            if self._invalid_episode:
                raise ValueError("episode_reset_requires_new_adapter")
            if self.last_frame is not None and frame != self.last_frame + 1:
                # No stale warm starts when controller was inactive for several ticks.
                self.controller.reset()
                self.actuator.reset()
            latest = self.world.get_snapshot()
            if (
                latest.frame != frame
                or abs(latest.timestamp.elapsed_seconds - now) > self.c.max_snapshot_age
            ):
                raise ValueError("stale_snapshot")
            if (
                self.last_time is not None
                and self.last_frame is not None
                and frame == self.last_frame + 1
            ):
                if abs(now - self.last_time - self.c.control_dt) > 1e-5:
                    raise ValueError("tick_interval_changed")
            snap = snapshot.find(self.vehicle.id)
            if snap is None:
                raise ValueError("ego_missing_from_snapshot")
            ego, steer_bound = self._state(snap)
            obstacles = self._obstacles(snapshot, ego, snap.get_transform().location.z)
            self.state_diagnostics["observed_obstacle_ids"] = [o.id for o in obstacles]
            state = self.route.project(ego, self.c.projection_distance)
            projection_diagnostics = dict(self.route.last_projection)
            stop_s = self._traffic_stop(ego, float(state[0]), now)
            limit = self.vehicle.get_speed_limit() / 3.6
            cmd = self.controller.step(
                ego, obstacles, limit if limit > 0 else None, stop_s, steer_bound
            )
            cmd.diagnostics.update(projection_diagnostics)
            cmd.diagnostics["requested_roadwheel_steer_rad"] = None if cmd.emergency else cmd.steer
            elapsed = time.perf_counter() - start
            # This is a post-return rejection, not a hard real-time preemption mechanism.
            if self.c.enforce_tick_deadline and elapsed > self.c.control_dt:
                cmd.emergency = True
                cmd.diagnostics["reason"] = "fallback_tick_deadline_missed"
            if self.world.get_snapshot().frame != frame:
                cmd.emergency = True
                cmd.diagnostics["reason"] = "frame_changed_during_compute"
            if cmd.emergency:
                self.actuator.reset()
                control = carla.VehicleControl(
                    throttle=0.0, brake=1.0, steer=float(self.vehicle.get_control().steer)
                )
            else:
                throttle, brake, actuation = self.actuator.step(
                    cmd.acceleration, cmd.target_speed, ego.speed,
                    urgent_brake=bool(cmd.diagnostics.get("prompt_braking_policy", False)),
                    stop_requested=bool(cmd.diagnostics.get("stop_intent", False)),
                    coast_requested=cmd.diagnostics.get("coast_requested"),
                )
                cmd.diagnostics["actuation"] = actuation
                normalized = normalized_from_equivalent(
                    cmd.steer,
                    self.state_diagnostics["inner_wheel_bound_rad"],
                    self.geometry.wheelbase,
                    self.state_diagnostics["front_track_m"],
                )
                cmd.diagnostics["requested_normalized_steer"] = normalized
                control = carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=normalized,
                )
        except (ValueError, RuntimeError) as exc:
            self.actuator.reset()
            control = carla.VehicleControl(
                throttle=0.0, brake=1.0, steer=float(self.vehicle.get_control().steer)
            )
            cmd = Command(diagnostics={"reason": str(exc)[:250]})
        cmd.diagnostics.update(
            {
                **self.state_diagnostics,
                "frame": frame,
                "simulation_time_s": now,
                "adapter_s": time.perf_counter() - start,
                "emergency": cmd.emergency,
                "information_source": "carla_ground_truth",
                "control": {
                    "throttle": control.throttle,
                    "brake": control.brake,
                    "steer": control.steer,
                },
            }
        )
        self.last_frame, self.last_time = frame, now
        self.last_diagnostics = cmd.diagnostics
        self.last_result = control
        return control
