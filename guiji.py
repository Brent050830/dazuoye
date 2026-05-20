import math
import time
from dataclasses import dataclass

import carla


# Scenario configuration
HOST = "localhost"
PORT = 2000
MAP_NAME = "Town04"
FIXED_DELTA_SECONDS = 0.05
SIM_SECONDS = 28.0

INITIAL_GAP = 48.0
LEAD_BRAKE_TIME = 6.0
EGO_TARGET_SPEED = 15.5
LEAD_TARGET_SPEED = 13.0

TTC_BRAKE_THRESHOLD = 4.5
TTC_AVOID_THRESHOLD = 3.6
SAFE_DISTANCE = 34.0
LANE_CLEAR_FRONT = 45.0
LANE_CLEAR_REAR = 18.0

LANE_CHANGE_LENGTH = 28.0
MPC_HORIZON_STEPS = 18
MPC_DT = 0.10
WHEEL_BASE = 2.85


def clamp(value, low, high):
    return max(low, min(high, value))


def vector_length(vector):
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def dot_2d(a, b):
    return a.x * b.x + a.y * b.y


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_rad(rotation):
    return math.radians(rotation.yaw)


def get_speed(vehicle):
    return vector_length(vehicle.get_velocity())


def speed_control(current_speed, target_speed):
    error = target_speed - current_speed
    if error >= 0.0:
        return clamp(0.18 + 0.06 * error, 0.0, 0.75), 0.0
    return 0.0, clamp(-0.12 * error, 0.0, 0.75)


def waypoint_steer(vehicle, carla_map, lookahead=12.0):
    waypoint = carla_map.get_waypoint(
        vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
    )
    next_waypoints = waypoint.next(lookahead)
    if not next_waypoints:
        return 0.0

    target = next_waypoints[0].transform.location
    transform = vehicle.get_transform()
    dx = target.x - transform.location.x
    dy = target.y - transform.location.y
    target_yaw = math.atan2(dy, dx)
    heading_error = normalize_angle(target_yaw - yaw_to_rad(transform.rotation))
    return clamp(1.8 * heading_error, -0.45, 0.45)


def vehicle_transform_from_waypoint(waypoint):
    transform = waypoint.transform
    transform.location.z += 0.45
    return transform


def same_direction_lane(source_wp, target_wp):
    if target_wp is None or target_wp.lane_type != carla.LaneType.Driving:
        return False
    yaw_error = abs(normalize_angle(yaw_to_rad(source_wp.transform.rotation) - yaw_to_rad(target_wp.transform.rotation)))
    return yaw_error < math.radians(30.0)


def find_fixed_scenario_waypoint(carla_map):
    """Pick a deterministic straight multi-lane start in Town04."""
    spawn_points = sorted(
        carla_map.get_spawn_points(),
        key=lambda t: (round(t.location.x, 1), round(t.location.y, 1), round(t.rotation.yaw, 1)),
    )

    candidates = []
    for index, transform in enumerate(spawn_points):
        waypoint = carla_map.get_waypoint(
            transform.location, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if waypoint.is_junction:
            continue

        future = waypoint.next(70.0)
        if not future or future[0].is_junction:
            continue

        yaw_now = yaw_to_rad(waypoint.transform.rotation)
        yaw_future = yaw_to_rad(future[0].transform.rotation)
        if abs(normalize_angle(yaw_future - yaw_now)) > math.radians(8.0):
            continue

        left_ok = same_direction_lane(waypoint, waypoint.get_left_lane())
        right_ok = same_direction_lane(waypoint, waypoint.get_right_lane())
        if left_ok or right_ok:
            candidates.append((index, waypoint, left_ok, right_ok))

    if not candidates:
        raise RuntimeError("No straight multi-lane spawn point found for the emergency avoidance scenario.")

    selected = candidates[len(candidates) // 2]
    print(
        "Fixed scenario start: sorted_spawn_index={}, left_lane={}, right_lane={}".format(
            selected[0], selected[2], selected[3]
        )
    )
    return selected[1]


@dataclass
class FrontVehicleReading:
    distance: float
    closing_speed: float
    ttc: float
    lateral_offset: float
    is_front_vehicle: bool


class VirtualGroundTruthSensor:
    """Ground-truth sensor used first; later it can be replaced by radar/lidar perception."""

    def __init__(self, world, carla_map, ego_vehicle, lead_vehicle):
        self.world = world
        self.carla_map = carla_map
        self.ego = ego_vehicle
        self.lead = lead_vehicle

    def front_vehicle(self):
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        lead_loc = self.lead.get_location()
        forward = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        relative = lead_loc - ego_loc

        longitudinal = dot_2d(relative, forward)
        lateral = dot_2d(relative, right)
        lane_width = self.carla_map.get_waypoint(ego_loc).lane_width

        ego_speed_along = dot_2d(self.ego.get_velocity(), forward)
        lead_speed_along = dot_2d(self.lead.get_velocity(), forward)
        closing_speed = ego_speed_along - lead_speed_along
        ttc = longitudinal / closing_speed if closing_speed > 0.1 and longitudinal > 0.0 else float("inf")

        is_front = longitudinal > 0.0 and abs(lateral) < lane_width * 0.65
        return FrontVehicleReading(longitudinal, closing_speed, ttc, lateral, is_front)

    def lane_clear(self, side):
        ego_wp = self.carla_map.get_waypoint(
            self.ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )
        target_wp = ego_wp.get_left_lane() if side == "left" else ego_wp.get_right_lane()
        if not same_direction_lane(ego_wp, target_wp):
            return False

        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        forward = ego_tf.get_forward_vector()

        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.id == self.ego.id:
                continue

            actor_wp = self.carla_map.get_waypoint(
                actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
            )
            if actor_wp.road_id != target_wp.road_id or actor_wp.lane_id != target_wp.lane_id:
                continue

            relative = actor.get_location() - ego_loc
            longitudinal = dot_2d(relative, forward)
            if -LANE_CLEAR_REAR <= longitudinal <= LANE_CLEAR_FRONT:
                return False

        return True


class QuinticLaneChangeTrajectory:
    """Lateral offset d(s) = D * (10t^3 - 15t^4 + 6t^5), t=s/L."""

    def __init__(self, start_transform, lateral_offset, length):
        self.origin = start_transform.location
        self.start_yaw = yaw_to_rad(start_transform.rotation)
        self.forward = start_transform.get_forward_vector()
        self.right = start_transform.get_right_vector()
        self.lateral_offset = lateral_offset
        self.length = length

    def to_local(self, location):
        relative = location - self.origin
        return dot_2d(relative, self.forward), dot_2d(relative, self.right)

    def lateral_at(self, s):
        if s <= 0.0:
            return 0.0
        if s >= self.length:
            return self.lateral_offset
        tau = s / self.length
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        return self.lateral_offset * blend

    def lateral_slope_at(self, s):
        if s <= 0.0 or s >= self.length:
            return 0.0
        tau = s / self.length
        blend_dot = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
        return self.lateral_offset * blend_dot / self.length


class SamplingMPCTracker:
    """Small receding-horizon MPC using a kinematic bicycle prediction model."""

    def __init__(self):
        self.previous_steer = 0.0

    def control(self, ego_vehicle, trajectory, target_speed):
        transform = ego_vehicle.get_transform()
        s0, d0 = trajectory.to_local(transform.location)
        yaw0 = normalize_angle(yaw_to_rad(transform.rotation) - trajectory.start_yaw)
        v0 = get_speed(ego_vehicle)

        steer_candidates = [
            clamp(self.previous_steer + delta, -0.65, 0.65)
            for delta in (-0.45, -0.32, -0.20, -0.10, 0.0, 0.10, 0.20, 0.32, 0.45)
        ]
        accel_candidates = (-4.0, -2.0, -1.0, 0.0, 1.0)

        best_cost = float("inf")
        best_action = (0.0, -3.0)

        for steer in steer_candidates:
            for accel in accel_candidates:
                s = s0
                d = d0
                yaw = yaw0
                speed = v0
                cost = 0.0

                for step in range(MPC_HORIZON_STEPS):
                    speed = max(0.0, speed + accel * MPC_DT)
                    s += speed * math.cos(yaw) * MPC_DT
                    d += speed * math.sin(yaw) * MPC_DT
                    yaw = normalize_angle(yaw + speed / WHEEL_BASE * math.tan(steer) * MPC_DT)

                    ref_d = trajectory.lateral_at(s)
                    ref_yaw = math.atan(trajectory.lateral_slope_at(s))
                    lateral_error = d - ref_d
                    yaw_error = normalize_angle(yaw - ref_yaw)
                    speed_error = speed - target_speed

                    cost += 6.0 * lateral_error**2
                    cost += 1.7 * yaw_error**2
                    cost += 0.07 * speed_error**2
                    cost += 0.08 * steer**2
                    cost += 0.01 * accel**2
                    cost += 0.02 * step * abs(steer - self.previous_steer)

                if cost < best_cost:
                    best_cost = cost
                    best_action = (steer, accel)

        steer, accel = best_action
        self.previous_steer = steer

        if accel >= 0.0:
            throttle = clamp(0.25 + 0.18 * accel, 0.0, 0.65)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-accel / 7.5, 0.0, 1.0)

        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)


class CollisionMonitor:
    def __init__(self, world, vehicle, actor_list):
        self.history = []
        blueprint = world.get_blueprint_library().find("sensor.other.collision")
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(self._on_collision)
        actor_list.append(self.sensor)

    def _on_collision(self, event):
        self.history.append(event)
        print("Collision detected with actor id {}".format(event.other_actor.id))


def setup_world(client):
    world = client.load_world(MAP_NAME)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)
    return world


def restore_world(world, original_settings):
    world.apply_settings(original_settings)


def spawn_scenario(world):
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()

    ego_bp = blueprint_library.find("vehicle.tesla.model3")
    lead_bp = blueprint_library.find("vehicle.lincoln.mkz_2020")
    ego_bp.set_attribute("role_name", "ego")
    lead_bp.set_attribute("role_name", "lead")

    ego_wp = find_fixed_scenario_waypoint(carla_map)
    lead_waypoints = ego_wp.next(INITIAL_GAP)
    if not lead_waypoints:
        raise RuntimeError("Could not place the lead vehicle ahead of the ego vehicle.")

    ego_vehicle = world.spawn_actor(ego_bp, vehicle_transform_from_waypoint(ego_wp))
    lead_vehicle = world.spawn_actor(lead_bp, vehicle_transform_from_waypoint(lead_waypoints[0]))

    return ego_vehicle, lead_vehicle


def set_spectator(world, ego_vehicle):
    ego_tf = ego_vehicle.get_transform()
    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            ego_tf.location + carla.Location(z=45.0),
            carla.Rotation(pitch=-75.0, yaw=ego_tf.rotation.yaw),
        )
    )


def choose_avoidance_side(sensor):
    if sensor.lane_clear("left"):
        return "left"
    if sensor.lane_clear("right"):
        return "right"
    return None


def main():
    actor_list = []
    world = None
    original_settings = None

    try:
        client = carla.Client(HOST, PORT)
        client.set_timeout(30.0)

        world = client.get_world()
        original_settings = world.get_settings()
        world = setup_world(client)
        carla_map = world.get_map()

        ego_vehicle, lead_vehicle = spawn_scenario(world)
        actor_list.extend([ego_vehicle, lead_vehicle])
        collision_monitor = CollisionMonitor(world, ego_vehicle, actor_list)
        sensor = VirtualGroundTruthSensor(world, carla_map, ego_vehicle, lead_vehicle)
        mpc = SamplingMPCTracker()

        world.tick()
        set_spectator(world, ego_vehicle)

        state = "FOLLOW"
        trajectory = None
        avoidance_side = None
        start_time = time.time()
        frame = 0

        print("Scenario started: map={}, ego=Tesla Model3, lead=Lincoln MKZ 2020".format(MAP_NAME))
        print("Lead car will brake hard at {:.1f}s.".format(LEAD_BRAKE_TIME))

        while frame * FIXED_DELTA_SECONDS < SIM_SECONDS:
            sim_time = frame * FIXED_DELTA_SECONDS

            if sim_time < LEAD_BRAKE_TIME:
                lead_throttle, lead_brake = speed_control(get_speed(lead_vehicle), LEAD_TARGET_SPEED)
                lead_steer = waypoint_steer(lead_vehicle, carla_map)
                lead_vehicle.apply_control(
                    carla.VehicleControl(throttle=lead_throttle, brake=lead_brake, steer=lead_steer)
                )
            else:
                lead_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))

            front = sensor.front_vehicle()
            ego_speed = get_speed(ego_vehicle)

            emergency_needed = (
                front.is_front_vehicle
                and front.distance < SAFE_DISTANCE
                and front.ttc < TTC_AVOID_THRESHOLD
            )
            brake_needed = (
                front.is_front_vehicle
                and front.ttc < TTC_BRAKE_THRESHOLD
            )

            if state == "FOLLOW" and emergency_needed:
                avoidance_side = choose_avoidance_side(sensor)
                if avoidance_side is not None:
                    lane_width = carla_map.get_waypoint(ego_vehicle.get_location()).lane_width
                    lateral_offset = -lane_width if avoidance_side == "left" else lane_width
                    trajectory = QuinticLaneChangeTrajectory(
                        ego_vehicle.get_transform(), lateral_offset, LANE_CHANGE_LENGTH
                    )
                    state = "AVOID"
                    print(
                        "Avoidance started at {:.2f}s: side={}, distance={:.1f}m, TTC={:.2f}s".format(
                            sim_time, avoidance_side, front.distance, front.ttc
                        )
                    )
                else:
                    state = "EMERGENCY_BRAKE"
                    print(
                        "Emergency brake only at {:.2f}s: no adjacent clear lane, TTC={:.2f}s".format(
                            sim_time, front.ttc
                        )
                    )

            if state == "AVOID" and trajectory is not None:
                target_speed = min(EGO_TARGET_SPEED, max(8.0, ego_speed))
                ego_control = mpc.control(ego_vehicle, trajectory, target_speed)
                if brake_needed:
                    ego_control.brake = max(ego_control.brake, 0.20)
                    ego_control.throttle = 0.0

                progress, lateral = trajectory.to_local(ego_vehicle.get_location())
                if progress > LANE_CHANGE_LENGTH + 8.0 and abs(lateral - trajectory.lateral_offset) < 0.65:
                    state = "LANE_KEEP"
                    print("Avoidance completed at {:.2f}s.".format(sim_time))

            elif state == "EMERGENCY_BRAKE":
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            else:
                if brake_needed:
                    target_speed = min(EGO_TARGET_SPEED, max(0.0, ego_speed - 5.0))
                else:
                    target_speed = EGO_TARGET_SPEED
                throttle, brake = speed_control(ego_speed, target_speed)
                ego_control = carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=waypoint_steer(ego_vehicle, carla_map),
                )

            ego_vehicle.apply_control(ego_control)

            if frame % int(1.0 / FIXED_DELTA_SECONDS) == 0:
                print(
                    "t={:05.2f}s state={:<15} dist={:05.1f}m ttc={:05.2f}s "
                    "ego={:04.1f}m/s lead={:04.1f}m/s steer={:+.2f} brake={:.2f}".format(
                        sim_time,
                        state,
                        front.distance,
                        front.ttc if math.isfinite(front.ttc) else 99.99,
                        ego_speed,
                        get_speed(lead_vehicle),
                        ego_control.steer,
                        ego_control.brake,
                    )
                )

            world.tick()
            set_spectator(world, ego_vehicle)
            frame += 1

        elapsed = time.time() - start_time
        print(
            "Scenario finished in {:.1f}s wall time. Collisions: {}".format(
                elapsed, len(collision_monitor.history)
            )
        )

    finally:
        if world is not None and original_settings is not None:
            restore_world(world, original_settings)
        for actor in actor_list:
            if actor is not None:
                actor.destroy()
        print("Cleanup finished.")


if __name__ == "__main__":
    main()
