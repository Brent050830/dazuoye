import math

import carla

from config import (
    BACKGROUND_BICYCLE_END_FORWARD_OFFSET,
    BACKGROUND_BICYCLE_FORWARD_OFFSETS,
    BACKGROUND_BICYCLE_RIGHT_OFFSETS,
    BACKGROUND_BICYCLE_SPEED_MAX,
    BACKGROUND_BICYCLE_SPEED_MIN,
<<<<<<< Updated upstream
=======
    BACKGROUND_NONMOTOR_TYPES,
>>>>>>> Stashed changes
    BACKGROUND_VEHICLE_ROUTE_INDICES,
    BACKGROUND_VEHICLE_SPEED_MAX,
    BACKGROUND_VEHICLE_SPEED_MIN,
    INITIAL_GAP,
<<<<<<< Updated upstream
=======
    KEY_OVERTAKE_VEHICLE_BLUEPRINT,
    KEY_OVERTAKE_VEHICLE_ROUTE_INDEX,
    KEY_OVERTAKE_VEHICLE_SPEED,
    NONMOTOR_START_CLUSTER_NEAR_JUNCTION,
    NONMOTOR_TRIGGER_DISTANCE,
    NONMOTOR_VISUAL_PHYSICS_EXPERIMENT,
    NONMOTOR_WAIT_AT_INTERSECTION,
>>>>>>> Stashed changes
    RIGHT_OBJECT_CLEAR_ROUTE_STEPS,
    RIGHT_OBJECT_CROSSING_SPEED,
    RIGHT_OBJECT_EXIT_ROAD,
    RIGHT_OBJECT_R344_ANCHOR_BACK_STEPS,
    RIGHT_OBJECT_R344_END_FORWARD_OFFSET,
    RIGHT_OBJECT_R344_RIGHT_OFFSET,
    RIGHT_OBJECT_R344_START_FORWARD_OFFSET,
    RIGHT_OBJECT_TRIGGER_ROAD,
    RIGHT_OBJECT_TRIGGER_ROUTE_STEPS,
<<<<<<< Updated upstream
)
from route import find_route_transition_index
from utils import clamp, get_town10_start_waypoint, vehicle_transform_from_waypoint
=======
    SLOW_RIGHT_LANE_VEHICLE_DISTANCE,
    SLOW_RIGHT_LANE_VEHICLE_ROLE_NAME,
    SLOW_RIGHT_LANE_VEHICLE_SPEED,
    VISUAL_TRAFFIC_ROLE_PREFIX,
    VISUAL_TRAFFIC_ROUTE_INDICES,
    VISUAL_TRAFFIC_SPEED_MAX,
    VISUAL_TRAFFIC_SPEED_MIN,
)
from route import find_route_transition_index
from utils import clamp, get_town10_start_waypoint, same_direction_lane, vehicle_transform_from_waypoint


def set_actor_target_velocity_safe(actor, velocity):
    try:
        actor.set_target_velocity(velocity)
    except (AttributeError, RuntimeError):
        pass
>>>>>>> Stashed changes


def spawn_scenario(world):
    """生成场景车辆：自车（Tesla Model3）和前车（Lincoln MKZ）"""
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()

    ego_bp = blueprint_library.find("vehicle.tesla.model3")
    lead_bp = blueprint_library.find("vehicle.lincoln.mkz_2020")
    ego_bp.set_attribute("role_name", "ego")
    lead_bp.set_attribute("role_name", "lead")

    ego_wp = get_town10_start_waypoint(carla_map)  # 找到确定性起始路点
    lead_waypoints = ego_wp.next(INITIAL_GAP)         # 前车在自车前方INITIAL_GAP米处
    if not lead_waypoints:
        raise RuntimeError("无法在自车前方放置前车。")

    ego_vehicle = world.spawn_actor(ego_bp, vehicle_transform_from_waypoint(ego_wp))
    lead_vehicle = world.spawn_actor(lead_bp, vehicle_transform_from_waypoint(lead_waypoints[0]))

    return ego_vehicle, lead_vehicle, ego_wp




class RightSideBicycleCrossing:
    """R344 -> R20 右转路口的右侧非机动车横穿目标。"""

<<<<<<< Updated upstream
    def __init__(self, actor, start_location, end_location, trigger_index, clear_index, speed):
=======
    def __init__(
        self,
        actor,
        start_location,
        end_location,
        trigger_index,
        clear_index,
        speed,
        trigger_location=None,
        trigger_distance=None,
    ):
>>>>>>> Stashed changes
        self.actor = actor
        self.name = actor.attributes.get("role_name", "right_side_bicycle")
        self.start_location = start_location
        self.end_location = end_location
        self.trigger_index = trigger_index
        self.clear_index = clear_index
        self.speed = speed
<<<<<<< Updated upstream
=======
        self.trigger_location = trigger_location
        self.trigger_distance = trigger_distance
>>>>>>> Stashed changes
        self.progress = 0.0
        self.is_active = False
        self.is_finished = False

        dx = end_location.x - start_location.x
        dy = end_location.y - start_location.y
        self.length = max(math.sqrt(dx * dx + dy * dy), 0.1)
        self.yaw = math.degrees(math.atan2(dy, dx))
        self.velocity = carla.Vector3D(dx / self.length * speed, dy / self.length * speed, 0.0)
        self._set_location(start_location)

    def _set_location(self, location):
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=self.yaw)))

<<<<<<< Updated upstream
    def update(self, route_index, dt):
        if self.is_finished:
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
            return

        if not self.is_active and route_index >= self.trigger_index:
            self.is_active = True
            print(
                "{} started: trigger_index={}, clear_index={}.".format(
                    self.name,
                    self.trigger_index, self.clear_index
=======
    def _should_start(self, route_index, ego_vehicle):
        if self.trigger_location is not None and self.trigger_distance is not None and ego_vehicle is not None:
            return ego_vehicle.get_location().distance(self.trigger_location) <= self.trigger_distance
        return route_index >= self.trigger_index

    def update(self, route_index, dt, ego_vehicle=None):
        if self.is_finished:
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
            set_actor_target_velocity_safe(self.actor, self.velocity)
            return

        if not self.is_active and self._should_start(route_index, ego_vehicle):
            self.is_active = True
            print(
                "{} started: trigger_index={}, clear_index={}, trigger_distance={}.".format(
                    self.name,
                    self.trigger_index,
                    self.clear_index,
                    self.trigger_distance,
>>>>>>> Stashed changes
                )
            )

        if not self.is_active:
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
<<<<<<< Updated upstream
=======
            set_actor_target_velocity_safe(self.actor, self.velocity)
>>>>>>> Stashed changes
            self._set_location(self.start_location)
            return

        self.progress = min(self.length, self.progress + self.speed * dt)
        ratio = self.progress / self.length
        current_location = carla.Location(
            x=self.start_location.x + (self.end_location.x - self.start_location.x) * ratio,
            y=self.start_location.y + (self.end_location.y - self.start_location.y) * ratio,
            z=self.start_location.z + (self.end_location.z - self.start_location.z) * ratio,
        )
        self.velocity = carla.Vector3D(
            (self.end_location.x - self.start_location.x) / self.length * self.speed,
            (self.end_location.y - self.start_location.y) / self.length * self.speed,
            0.0,
        )
        self._set_location(current_location)
<<<<<<< Updated upstream
=======
        set_actor_target_velocity_safe(self.actor, self.velocity)
>>>>>>> Stashed changes

        if self.progress >= self.length:
            self.is_finished = True
            self.is_active = False
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
<<<<<<< Updated upstream
=======
            set_actor_target_velocity_safe(self.actor, self.velocity)
>>>>>>> Stashed changes
            print("{} finished crossing.".format(self.name))

    def is_conflict_window(self, route_index):
        return self.trigger_index <= route_index <= self.clear_index and not self.is_finished


class BackgroundRouteVehicle:
<<<<<<< Updated upstream
    """沿自车同一条固定路线行驶的背景车辆，仅改变初始位置和速度。"""

    def __init__(self, actor, target_speed, start_index, loop_route):
        self.actor = actor
=======
    """沿自车同一条固定路线行驶的剧本车辆。"""

    def __init__(self, actor, target_speed, start_index, loop_route):
        self.actor = actor
        self.name = actor.attributes.get("role_name", "route_vehicle")
>>>>>>> Stashed changes
        self.target_speed = target_speed
        self.loop_route = loop_route
        self.route_span = max(loop_route.step_distance, (len(loop_route.points) - 1) * loop_route.step_distance)
        self.progress = clamp(start_index, 0, len(loop_route.points) - 2) * loop_route.step_distance
        self.z_offset = actor.get_location().z - loop_route.points[int(self.progress / loop_route.step_distance)].z
        self.update(0.0)

    def update(self, dt):
        self.progress = (self.progress + self.target_speed * dt) % self.route_span
        route_position = self.progress / self.loop_route.step_distance
        low_index = min(int(route_position), len(self.loop_route.points) - 2)
        high_index = low_index + 1
        ratio = route_position - low_index

        start = self.loop_route.points[low_index]
        end = self.loop_route.points[high_index]
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z

        location = carla.Location(
            x=start.x + dx * ratio,
            y=start.y + dy * ratio,
            z=start.z + dz * ratio + self.z_offset,
        )
        yaw = math.degrees(math.atan2(dy, dx))
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=yaw)))

        segment_length = math.sqrt(dx * dx + dy * dy)
        if segment_length > 0.001:
            velocity = carla.Vector3D(
                x=dx / segment_length * self.target_speed,
                y=dy / segment_length * self.target_speed,
                z=0.0,
            )
            self.actor.set_target_velocity(velocity)


<<<<<<< Updated upstream
=======
class RightLaneSlowVehicleController:
    """沿最右侧同向车道慢速行驶的交通车。"""

    def __init__(self, actor, lane_waypoints, target_speed, step_distance):
        if len(lane_waypoints) < 2:
            raise RuntimeError("RightLaneSlowVehicleController requires at least two waypoints.")
        self.actor = actor
        self.lane_waypoints = lane_waypoints
        self.target_speed = target_speed
        self.step_distance = step_distance
        self.route_span = max(step_distance, (len(lane_waypoints) - 1) * step_distance)
        self.progress = 0.0
        self.z_offset = actor.get_location().z - lane_waypoints[0].transform.location.z
        self.update(0.0)

    def update(self, dt):
        self.progress = min(self.route_span, self.progress + self.target_speed * dt)
        route_position = self.progress / self.step_distance
        low_index = min(int(route_position), len(self.lane_waypoints) - 2)
        high_index = low_index + 1
        ratio = route_position - low_index

        start = self.lane_waypoints[low_index].transform.location
        end = self.lane_waypoints[high_index].transform.location
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z

        location = carla.Location(
            x=start.x + dx * ratio,
            y=start.y + dy * ratio,
            z=start.z + dz * ratio + self.z_offset,
        )
        yaw = math.degrees(math.atan2(dy, dx))
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=yaw)))

        segment_length = math.sqrt(dx * dx + dy * dy)
        if segment_length > 0.001:
            velocity = carla.Vector3D(
                x=dx / segment_length * self.target_speed,
                y=dy / segment_length * self.target_speed,
                z=0.0,
            )
            self.actor.set_target_velocity(velocity)


class VisualLaneVehicleController:
    """沿非主路线车道行驶的纯视觉交通车。"""

    def __init__(self, actor, lane_waypoints, target_speed, step_distance):
        self.actor = actor
        self.lane_waypoints = lane_waypoints
        self.target_speed = target_speed
        self.step_distance = step_distance
        self.route_span = max(step_distance, (len(lane_waypoints) - 1) * step_distance)
        self.progress = 0.0
        self.z_offset = actor.get_location().z - lane_waypoints[0].transform.location.z
        self.update(0.0)

    def update(self, dt):
        self.progress = (self.progress + self.target_speed * dt) % self.route_span
        route_position = self.progress / self.step_distance
        low_index = min(int(route_position), len(self.lane_waypoints) - 2)
        high_index = low_index + 1
        ratio = route_position - low_index

        start = self.lane_waypoints[low_index].transform.location
        end = self.lane_waypoints[high_index].transform.location
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        location = carla.Location(
            x=start.x + dx * ratio,
            y=start.y + dy * ratio,
            z=start.z + dz * ratio + self.z_offset,
        )
        yaw = math.degrees(math.atan2(dy, dx))
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=yaw)))

        segment_length = math.sqrt(dx * dx + dy * dy)
        if segment_length > 0.001:
            velocity = carla.Vector3D(
                x=dx / segment_length * self.target_speed,
                y=dy / segment_length * self.target_speed,
                z=0.0,
            )
            self.actor.set_target_velocity(velocity)


def get_rightmost_same_direction_lane(waypoint):
    """寻找给定路点右侧最外层的同向 Driving 车道。"""
    current = waypoint
    while True:
        right_lane = current.get_right_lane()
        if not same_direction_lane(current, right_lane):
            return current
        current = right_lane


def build_right_lane_waypoint_path(start_waypoint, step_distance, max_steps=80):
    """沿同向车道向前构建慢车行驶路点序列。"""
    waypoints = [start_waypoint]
    waypoint = start_waypoint
    for _ in range(max_steps):
        next_waypoints = waypoint.next(step_distance)
        if not next_waypoints:
            break
        same_direction_next = [
            candidate for candidate in next_waypoints
            if same_direction_lane(waypoint, candidate)
        ]
        if not same_direction_next:
            break
        waypoint = same_direction_next[0]
        waypoints.append(waypoint)
    return waypoints


>>>>>>> Stashed changes


def find_nonmotor_blueprint(blueprint_library):
    """优先选择自行车蓝图；不可用时使用摩托车或普通车辆替代。"""
    preferred_ids = (
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
        "vehicle.yamaha.yzf",
        "vehicle.kawasaki.ninja",
    )
    for blueprint_id in preferred_ids:
        try:
            return blueprint_library.find(blueprint_id)
        except (IndexError, RuntimeError):
            continue

    vehicles = blueprint_library.filter("vehicle.*")
    if not vehicles:
        raise RuntimeError("No vehicle blueprint is available for the right-side object scenario.")
    return vehicles[0]


<<<<<<< Updated upstream
=======
def find_pedestrian_blueprint(blueprint_library):
    walkers = blueprint_library.filter("walker.pedestrian.*")
    if not walkers:
        raise RuntimeError("No pedestrian blueprint is available for the walking crossing scenario.")
    return walkers[0]


def opposite_direction_lane(source_wp):
    for lane_getter in (source_wp.get_left_lane, source_wp.get_right_lane):
        candidate = lane_getter()
        if candidate is None or candidate.lane_type != carla.LaneType.Driving:
            continue
        yaw_error = abs(math.radians(candidate.transform.rotation.yaw - source_wp.transform.rotation.yaw))
        yaw_error = abs((yaw_error + math.pi) % (2.0 * math.pi) - math.pi)
        if yaw_error > math.radians(150.0):
            return candidate
    return None


>>>>>>> Stashed changes
def get_r344_nonmotor_anchor(loop_route, transition_index):
    """获取 R344 右转入口附近的非机动车直行锚点。"""
    anchor_index = max(0, transition_index - RIGHT_OBJECT_R344_ANCHOR_BACK_STEPS)
    anchor_wp = loop_route.waypoints[anchor_index]
    if anchor_wp.road_id != RIGHT_OBJECT_TRIGGER_ROAD:
        return None, anchor_index
    return anchor_wp, anchor_index


def r344_nonmotor_location(anchor_wp, forward_offset, right_offset):
    """在 R344 连接段右侧按前向/右向偏移生成非机动车位置。"""
    anchor_location = anchor_wp.transform.location
    anchor_forward = anchor_wp.transform.get_forward_vector()
    anchor_right = anchor_wp.transform.get_right_vector()
    return carla.Location(
        x=anchor_location.x + anchor_forward.x * forward_offset + anchor_right.x * right_offset,
        y=anchor_location.y + anchor_forward.y * forward_offset + anchor_right.y * right_offset,
        z=anchor_location.z + anchor_forward.z * forward_offset + anchor_right.z * right_offset + 0.65,
    )


def spawn_actor_with_z_retry(world, blueprint, transform, z_retry=0.5):
    """先按原始位置生成 actor，失败后抬高一次重试。"""
    actor = world.try_spawn_actor(blueprint, transform)
    if actor is not None:
        return actor
    retry_transform = carla.Transform(
        carla.Location(
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z + z_retry,
        ),
        transform.rotation,
    )
    return world.try_spawn_actor(blueprint, retry_transform)


<<<<<<< Updated upstream
=======
def is_spawn_transform_clear(actor_list, transform, min_distance=10.0):
    """避免新生成车辆与已存在 actor 开局重叠。"""
    for actor in actor_list:
        if actor is None or not actor.is_alive:
            continue
        if actor.get_location().distance(transform.location) < min_distance:
            return False
    return True


>>>>>>> Stashed changes
def spawn_right_side_bicycle_crossing(world, loop_route, actor_list):
    """在 R344 -> R20 右转处生成从右侧通过的非机动车目标。"""
    transition_index = find_route_transition_index(
        loop_route, RIGHT_OBJECT_TRIGGER_ROAD, RIGHT_OBJECT_EXIT_ROAD
    )
    if transition_index is None:
        print("Right-side bicycle skipped: R344 -> R20 transition not found on route.")
        return None

    anchor_wp, anchor_index = get_r344_nonmotor_anchor(loop_route, transition_index)
    if anchor_wp is None:
        print(
            "Right-side bicycle skipped: R344 crossing anchor mismatch, index={}, road={}.".format(
                anchor_index, loop_route.waypoints[anchor_index].road_id
            )
        )
        return None

<<<<<<< Updated upstream
    start_location = r344_nonmotor_location(
        anchor_wp, RIGHT_OBJECT_R344_START_FORWARD_OFFSET, RIGHT_OBJECT_R344_RIGHT_OFFSET
=======
    start_forward_offset = -2.0 if NONMOTOR_START_CLUSTER_NEAR_JUNCTION else RIGHT_OBJECT_R344_START_FORWARD_OFFSET
    start_location = r344_nonmotor_location(
        anchor_wp, start_forward_offset, RIGHT_OBJECT_R344_RIGHT_OFFSET
>>>>>>> Stashed changes
    )
    end_location = r344_nonmotor_location(
        anchor_wp, RIGHT_OBJECT_R344_END_FORWARD_OFFSET, RIGHT_OBJECT_R344_RIGHT_OFFSET
    )
    yaw = math.degrees(math.atan2(end_location.y - start_location.y, end_location.x - start_location.x))
    start_transform = carla.Transform(start_location, carla.Rotation(yaw=yaw))

    blueprint = find_nonmotor_blueprint(world.get_blueprint_library())
    try:
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "right_side_bicycle")
    except AttributeError:
        pass
    actor = spawn_actor_with_z_retry(world, blueprint, start_transform)
    if actor is None:
        print("Right-side bicycle skipped: failed to spawn actor near R344 -> R20.")
        return None

<<<<<<< Updated upstream
    actor.set_simulate_physics(False)
    actor_list.append(actor)
=======
    actor.set_simulate_physics(NONMOTOR_VISUAL_PHYSICS_EXPERIMENT)
    actor_list.append(actor)
    print(
        "Right-side bicycle note: CARLA bicycle blueprints may have limited pedal/wheel animation; "
        "kinematic mode is kept by default for stable scenario playback."
    )
>>>>>>> Stashed changes

    scenario = RightSideBicycleCrossing(
        actor,
        start_transform.location,
        end_location,
        max(0, transition_index - RIGHT_OBJECT_TRIGGER_ROUTE_STEPS),
        min(len(loop_route.waypoints) - 1, transition_index + RIGHT_OBJECT_CLEAR_ROUTE_STEPS),
        RIGHT_OBJECT_CROSSING_SPEED,
<<<<<<< Updated upstream
=======
        trigger_location=anchor_wp.transform.location if NONMOTOR_WAIT_AT_INTERSECTION else None,
        trigger_distance=NONMOTOR_TRIGGER_DISTANCE if NONMOTOR_WAIT_AT_INTERSECTION else None,
>>>>>>> Stashed changes
    )
    print(
        "Right-side bicycle ready: blueprint={}, transition_index={}, anchor_index={}, anchor_road={}, path=straight_along_r344, start=({:.1f}, {:.1f}), end=({:.1f}, {:.1f}).".format(
            blueprint.id,
            transition_index,
            anchor_index,
            anchor_wp.road_id,
            start_location.x,
            start_location.y,
            end_location.x,
            end_location.y,
        )
    )
    return scenario


<<<<<<< Updated upstream
=======
def spawn_slow_right_lane_vehicle(world, carla_map, loop_route, actor_list):
    """在自车既定路线前方约 75m 生成一辆最右侧同向慢车。"""
    route_index = min(
        int(SLOW_RIGHT_LANE_VEHICLE_DISTANCE / loop_route.step_distance),
        len(loop_route.waypoints) - 1,
    )
    route_waypoint = loop_route.waypoints[route_index]
    lane_waypoint = get_rightmost_same_direction_lane(route_waypoint)
    lane_path = build_right_lane_waypoint_path(lane_waypoint, loop_route.step_distance)
    if len(lane_path) < 2:
        print("Slow right-lane vehicle skipped: no usable lane path at route_index={}.".format(route_index))
        return None

    blueprint_library = world.get_blueprint_library()
    blueprint = blueprint_library.find("vehicle.audi.tt")
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", SLOW_RIGHT_LANE_VEHICLE_ROLE_NAME)

    transform = vehicle_transform_from_waypoint(lane_path[0])
    if not is_spawn_transform_clear(actor_list, transform, min_distance=12.0):
        print("Slow right-lane vehicle skipped: spawn point is too close to existing actors.")
        return None

    actor = spawn_actor_with_z_retry(world, blueprint, transform)
    if actor is None:
        print("Slow right-lane vehicle skipped: failed to spawn actor.")
        return None

    actor.set_simulate_physics(False)
    actor_list.append(actor)
    controller = RightLaneSlowVehicleController(
        actor,
        lane_path,
        SLOW_RIGHT_LANE_VEHICLE_SPEED,
        loop_route.step_distance,
    )
    print(
        "Slow right-lane vehicle ready: role={}, route_index={}, road={}, lane={}, speed={:.1f}m/s.".format(
            SLOW_RIGHT_LANE_VEHICLE_ROLE_NAME,
            route_index,
            lane_waypoint.road_id,
            lane_waypoint.lane_id,
            SLOW_RIGHT_LANE_VEHICLE_SPEED,
        )
    )
    return controller


>>>>>>> Stashed changes
def spawn_background_route_vehicles(world, loop_route, actor_list, rng):
    """在自车固定路线不同路段生成 5 辆较慢的背景车辆。"""
    blueprint_library = world.get_blueprint_library()
    preferred_ids = (
        "vehicle.audi.tt",
        "vehicle.dodge.charger_2020",
        "vehicle.mercedes.coupe",
        "vehicle.mini.cooper_s",
        "vehicle.nissan.patrol",
    )
    vehicles = []
    for index, blueprint_id in zip(BACKGROUND_VEHICLE_ROUTE_INDICES, preferred_ids):
        blueprint = blueprint_library.find(blueprint_id)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "background_vehicle_{}".format(len(vehicles) + 1))

        waypoint_index = min(index, len(loop_route.waypoints) - 1)
        transform = vehicle_transform_from_waypoint(loop_route.waypoints[waypoint_index])
<<<<<<< Updated upstream
=======
        if not is_spawn_transform_clear(actor_list, transform, min_distance=12.0):
            print("Background vehicle skipped: index={} is too close to existing actors.".format(waypoint_index))
            continue
>>>>>>> Stashed changes
        actor = spawn_actor_with_z_retry(world, blueprint, transform)
        if actor is None:
            print("Background vehicle skipped: index={}, blueprint={}.".format(waypoint_index, blueprint_id))
            continue

        actor.set_simulate_physics(False)
        target_speed = rng.uniform(BACKGROUND_VEHICLE_SPEED_MIN, BACKGROUND_VEHICLE_SPEED_MAX)
        actor_list.append(actor)
        vehicles.append(BackgroundRouteVehicle(actor, target_speed, waypoint_index, loop_route))
        print(
            "Background vehicle ready: role={}, route_index={}, target_speed={:.1f}m/s.".format(
                actor.attributes.get("role_name", "--"),
                waypoint_index,
                target_speed,
            )
        )
    return vehicles


<<<<<<< Updated upstream
=======
def spawn_visual_traffic_vehicles(world, loop_route, actor_list, rng):
    """生成不接入感知的对向/旁路视觉交通流。"""
    blueprint_library = world.get_blueprint_library()
    preferred_ids = (
        "vehicle.lincoln.mkz_2020",
        "vehicle.bmw.grandtourer",
        "vehicle.chevrolet.impala",
        "vehicle.toyota.prius",
        "vehicle.carlamotors.carlacola",
    )
    vehicles = []
    for idx, route_index in enumerate(VISUAL_TRAFFIC_ROUTE_INDICES, 1):
        route_index = min(route_index, len(loop_route.waypoints) - 1)
        lane_wp = opposite_direction_lane(loop_route.waypoints[route_index])
        if lane_wp is None:
            print("Visual traffic skipped: no opposite lane at route_index={}.".format(route_index))
            continue

        lane_path = build_right_lane_waypoint_path(lane_wp, loop_route.step_distance, max_steps=90)
        if len(lane_path) < 2:
            print("Visual traffic skipped: short lane path at route_index={}.".format(route_index))
            continue

        blueprint = blueprint_library.find(preferred_ids[(idx - 1) % len(preferred_ids)])
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "{}_{}".format(VISUAL_TRAFFIC_ROLE_PREFIX, idx))

        transform = vehicle_transform_from_waypoint(lane_path[0])
        if not is_spawn_transform_clear(actor_list, transform, min_distance=10.0):
            print("Visual traffic skipped: spawn point too close at route_index={}.".format(route_index))
            continue

        actor = spawn_actor_with_z_retry(world, blueprint, transform)
        if actor is None:
            print("Visual traffic skipped: failed to spawn at route_index={}.".format(route_index))
            continue

        actor.set_simulate_physics(False)
        actor_list.append(actor)
        target_speed = rng.uniform(VISUAL_TRAFFIC_SPEED_MIN, VISUAL_TRAFFIC_SPEED_MAX)
        vehicles.append(VisualLaneVehicleController(actor, lane_path, target_speed, loop_route.step_distance))
        print(
            "Visual traffic ready: role={}, route_index={}, road={}, lane={}, speed={:.1f}m/s.".format(
                actor.attributes.get("role_name", "--"),
                route_index,
                lane_wp.road_id,
                lane_wp.lane_id,
                target_speed,
            )
        )
    return vehicles


def spawn_key_overtake_vehicle(world, loop_route, actor_list):
    """生成第一段横向超车目标：低速同车道车辆。"""
    blueprint_library = world.get_blueprint_library()
    blueprint = blueprint_library.find(KEY_OVERTAKE_VEHICLE_BLUEPRINT)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", "slow_overtake_vehicle")

    waypoint_index = min(KEY_OVERTAKE_VEHICLE_ROUTE_INDEX, len(loop_route.waypoints) - 1)
    transform = vehicle_transform_from_waypoint(loop_route.waypoints[waypoint_index])
    if not is_spawn_transform_clear(actor_list, transform, min_distance=14.0):
        print("Key overtake vehicle skipped: index={} is too close to existing actors.".format(waypoint_index))
        return None
    actor = spawn_actor_with_z_retry(world, blueprint, transform)
    if actor is None:
        print(
            "Key overtake vehicle skipped: index={}, blueprint={}.".format(
                waypoint_index, KEY_OVERTAKE_VEHICLE_BLUEPRINT
            )
        )
        return None

    actor.set_simulate_physics(False)
    actor_list.append(actor)
    controller = BackgroundRouteVehicle(actor, KEY_OVERTAKE_VEHICLE_SPEED, waypoint_index, loop_route)
    print(
        "Key overtake vehicle ready: role={}, route_index={}, target_speed={:.1f}m/s.".format(
            actor.attributes.get("role_name", "--"),
            waypoint_index,
            KEY_OVERTAKE_VEHICLE_SPEED,
        )
    )
    return controller


>>>>>>> Stashed changes
def spawn_background_r344_bicycles(world, loop_route, actor_list, rng):
    """在 R344 右侧非机动车区域生成额外背景自行车。"""
    transition_index = find_route_transition_index(
        loop_route, RIGHT_OBJECT_TRIGGER_ROAD, RIGHT_OBJECT_EXIT_ROAD
    )
    if transition_index is None:
        print("Background bicycles skipped: R344 -> R20 transition not found on route.")
        return []

    anchor_wp, anchor_index = get_r344_nonmotor_anchor(loop_route, transition_index)
    if anchor_wp is None:
        print("Background bicycles skipped: R344 nonmotor anchor not found.")
        return []

<<<<<<< Updated upstream
    blueprint = find_nonmotor_blueprint(world.get_blueprint_library())
=======
    blueprint_library = world.get_blueprint_library()
    bicycle_blueprint = find_nonmotor_blueprint(blueprint_library)
    pedestrian_blueprint = find_pedestrian_blueprint(blueprint_library)
>>>>>>> Stashed changes
    bicycles = []
    for idx, (forward_offset, right_offset) in enumerate(
        zip(BACKGROUND_BICYCLE_FORWARD_OFFSETS, BACKGROUND_BICYCLE_RIGHT_OFFSETS),
        1,
    ):
<<<<<<< Updated upstream
        start_location = r344_nonmotor_location(anchor_wp, forward_offset, right_offset)
        end_location = r344_nonmotor_location(anchor_wp, BACKGROUND_BICYCLE_END_FORWARD_OFFSET, right_offset)
        yaw = math.degrees(math.atan2(end_location.y - start_location.y, end_location.x - start_location.x))
        transform = carla.Transform(start_location, carla.Rotation(yaw=yaw))

        bicycle_bp = blueprint
        try:
            if bicycle_bp.has_attribute("role_name"):
                bicycle_bp.set_attribute("role_name", "background_bicycle_{}".format(idx))
=======
        nonmotor_type = BACKGROUND_NONMOTOR_TYPES[min(idx - 1, len(BACKGROUND_NONMOTOR_TYPES) - 1)]
        if NONMOTOR_START_CLUSTER_NEAR_JUNCTION:
            forward_offset = -4.0 - 2.0 * (idx - 1)
        start_location = r344_nonmotor_location(anchor_wp, forward_offset, right_offset)
        end_location = r344_nonmotor_location(anchor_wp, BACKGROUND_BICYCLE_END_FORWARD_OFFSET, right_offset)
        if nonmotor_type == "pedestrian":
            start_location.z -= 0.55
            end_location.z -= 0.55
        yaw = math.degrees(math.atan2(end_location.y - start_location.y, end_location.x - start_location.x))
        transform = carla.Transform(start_location, carla.Rotation(yaw=yaw))

        bicycle_bp = pedestrian_blueprint if nonmotor_type == "pedestrian" else bicycle_blueprint
        try:
            if bicycle_bp.has_attribute("role_name"):
                bicycle_bp.set_attribute("role_name", "background_{}_{}".format(nonmotor_type, idx))
>>>>>>> Stashed changes
        except AttributeError:
            pass

        actor = spawn_actor_with_z_retry(world, bicycle_bp, transform)
        if actor is None:
<<<<<<< Updated upstream
            print("Background bicycle skipped: index={}, start_offset={}.".format(idx, forward_offset))
            continue

        actor.set_simulate_physics(False)
=======
            print("Background {} skipped: index={}, start_offset={}.".format(nonmotor_type, idx, forward_offset))
            continue

        actor.set_simulate_physics(NONMOTOR_VISUAL_PHYSICS_EXPERIMENT)
>>>>>>> Stashed changes
        actor_list.append(actor)
        speed = rng.uniform(BACKGROUND_BICYCLE_SPEED_MIN, BACKGROUND_BICYCLE_SPEED_MAX)
        scenario = RightSideBicycleCrossing(
            actor,
            transform.location,
            end_location,
            max(0, transition_index - RIGHT_OBJECT_TRIGGER_ROUTE_STEPS - 10 + idx * 3),
            min(len(loop_route.waypoints) - 1, transition_index + RIGHT_OBJECT_CLEAR_ROUTE_STEPS + 8),
            speed,
<<<<<<< Updated upstream
        )
        bicycles.append(scenario)
        print(
            "Background bicycle ready: role={}, anchor_index={}, right_offset={:.1f}, speed={:.1f}m/s.".format(
=======
            trigger_location=anchor_wp.transform.location if NONMOTOR_WAIT_AT_INTERSECTION else None,
            trigger_distance=NONMOTOR_TRIGGER_DISTANCE if NONMOTOR_WAIT_AT_INTERSECTION else None,
        )
        bicycles.append(scenario)
        print(
            "Background {} ready: role={}, anchor_index={}, right_offset={:.1f}, speed={:.1f}m/s.".format(
                nonmotor_type,
>>>>>>> Stashed changes
                actor.attributes.get("role_name", "--"),
                anchor_index,
                right_offset,
                speed,
            )
        )
    return bicycles


