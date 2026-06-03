import math

import carla

from config import (
    BACKGROUND_BICYCLE_END_FORWARD_OFFSET,
    BACKGROUND_BICYCLE_FORWARD_OFFSETS,
    BACKGROUND_BICYCLE_RIGHT_OFFSETS,
    BACKGROUND_BICYCLE_SPEED_MAX,
    BACKGROUND_BICYCLE_SPEED_MIN,
    BACKGROUND_VEHICLE_ROUTE_INDICES,
    BACKGROUND_VEHICLE_SPEED_MAX,
    BACKGROUND_VEHICLE_SPEED_MIN,
    INITIAL_GAP,
    RIGHT_OBJECT_CLEAR_ROUTE_STEPS,
    RIGHT_OBJECT_CROSSING_SPEED,
    RIGHT_OBJECT_EXIT_ROAD,
    RIGHT_OBJECT_R344_ANCHOR_BACK_STEPS,
    RIGHT_OBJECT_R344_END_FORWARD_OFFSET,
    RIGHT_OBJECT_R344_RIGHT_OFFSET,
    RIGHT_OBJECT_R344_START_FORWARD_OFFSET,
    RIGHT_OBJECT_TRIGGER_ROAD,
    RIGHT_OBJECT_TRIGGER_ROUTE_STEPS,
)
from route import find_route_transition_index
from utils import clamp, get_town10_start_waypoint, vehicle_transform_from_waypoint


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

    ego_vehicle = world.spawn_actor(ego_bp, vehicle_transform_from_waypoint(ego_wp)) # 在自车起始路点生成自车
    lead_vehicle = world.spawn_actor(lead_bp, vehicle_transform_from_waypoint(lead_waypoints[0]))

    return ego_vehicle, lead_vehicle, ego_wp # 返回生成的自车、前车和自车的起始路点




class RightSideBicycleCrossing:
    """R344 -> R20 右转路口的右侧非机动车横穿目标。"""

    def __init__(self, actor, start_location, end_location, trigger_index, clear_index, speed):
        self.actor = actor
        self.name = actor.attributes.get("role_name", "right_side_bicycle")
        self.start_location = start_location
        self.end_location = end_location
        self.trigger_index = trigger_index
        self.clear_index = clear_index
        self.speed = speed
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
                )
            )

        if not self.is_active:
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
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

        if self.progress >= self.length:
            self.is_finished = True
            self.is_active = False
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
            print("{} finished crossing.".format(self.name))

    def is_conflict_window(self, route_index):
        return self.trigger_index <= route_index <= self.clear_index and not self.is_finished


class BackgroundRouteVehicle:
    """沿自车同一条固定路线行驶的背景车辆，仅改变初始位置和速度。"""

    def __init__(self, actor, target_speed, start_index, loop_route):
        self.actor = actor
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

    start_location = r344_nonmotor_location(
        anchor_wp, RIGHT_OBJECT_R344_START_FORWARD_OFFSET, RIGHT_OBJECT_R344_RIGHT_OFFSET
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

    actor.set_simulate_physics(False)
    actor_list.append(actor)

    scenario = RightSideBicycleCrossing(
        actor,
        start_transform.location,
        end_location,
        max(0, transition_index - RIGHT_OBJECT_TRIGGER_ROUTE_STEPS),
        min(len(loop_route.waypoints) - 1, transition_index + RIGHT_OBJECT_CLEAR_ROUTE_STEPS),
        RIGHT_OBJECT_CROSSING_SPEED,
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

    blueprint = find_nonmotor_blueprint(world.get_blueprint_library())
    bicycles = []
    for idx, (forward_offset, right_offset) in enumerate(
        zip(BACKGROUND_BICYCLE_FORWARD_OFFSETS, BACKGROUND_BICYCLE_RIGHT_OFFSETS),
        1,
    ):
        start_location = r344_nonmotor_location(anchor_wp, forward_offset, right_offset)
        end_location = r344_nonmotor_location(anchor_wp, BACKGROUND_BICYCLE_END_FORWARD_OFFSET, right_offset)
        yaw = math.degrees(math.atan2(end_location.y - start_location.y, end_location.x - start_location.x))
        transform = carla.Transform(start_location, carla.Rotation(yaw=yaw))

        bicycle_bp = blueprint
        try:
            if bicycle_bp.has_attribute("role_name"):
                bicycle_bp.set_attribute("role_name", "background_bicycle_{}".format(idx))
        except AttributeError:
            pass

        actor = spawn_actor_with_z_retry(world, bicycle_bp, transform)
        if actor is None:
            print("Background bicycle skipped: index={}, start_offset={}.".format(idx, forward_offset))
            continue

        actor.set_simulate_physics(False)
        actor_list.append(actor)
        speed = rng.uniform(BACKGROUND_BICYCLE_SPEED_MIN, BACKGROUND_BICYCLE_SPEED_MAX)
        scenario = RightSideBicycleCrossing(
            actor,
            transform.location,
            end_location,
            max(0, transition_index - RIGHT_OBJECT_TRIGGER_ROUTE_STEPS - 10 + idx * 3),
            min(len(loop_route.waypoints) - 1, transition_index + RIGHT_OBJECT_CLEAR_ROUTE_STEPS + 8),
            speed,
        )
        bicycles.append(scenario)
        print(
            "Background bicycle ready: role={}, anchor_index={}, right_offset={:.1f}, speed={:.1f}m/s.".format(
                actor.attributes.get("role_name", "--"),
                anchor_index,
                right_offset,
                speed,
            )
        )
    return bicycles


