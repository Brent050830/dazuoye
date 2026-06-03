"""通用数学、车辆状态和基础道路辅助函数。"""

import math

import carla

from config import TOWN10_START_SPAWN_INDEX


def clamp(value, low, high):
    """将 value 限制在 [low, high] 范围内。"""
    return max(low, min(high, value))


def vector_length(vector):
    """计算三维向量的欧几里得长度。"""
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def dot_2d(a, b):
    """计算两个向量在水平面（XY）上的点积。"""
    return a.x * b.x + a.y * b.y


def normalize_angle(angle):
    """将任意角度归一化到 (-π, π] 区间。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_rad(rotation):
    """将 CARLA Rotation 的偏航角（度）转换为弧度。"""
    return math.radians(rotation.yaw)


def get_speed(vehicle):
    """获取车辆当前速度的标量值（m/s）。"""
    return vector_length(vehicle.get_velocity())


def speed_control(current_speed, target_speed):
    """简单比例速度控制器，返回 (油门, 制动) 元组。"""
    error = target_speed - current_speed
    if error >= 0.0:
        return clamp(0.18 + 0.06 * error, 0.0, 0.75), 0.0
    return 0.0, clamp(-0.12 * error, 0.0, 0.75)


def waypoint_steer(vehicle, carla_map, lookahead=12.0):
    """基于前视路点的纯追踪转向控制，返回归一化转向量。"""
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
    """从路点生成车辆生成位置，Z 轴抬高 0.45m 避免穿地。"""
    transform = waypoint.transform
    transform.location.z += 0.45
    return transform


def same_direction_lane(source_wp, target_wp):
    """判断目标路点是否为与源路点同向的行驶车道。"""
    if target_wp is None or target_wp.lane_type != carla.LaneType.Driving:
        return False
    yaw_error = abs(
        normalize_angle(yaw_to_rad(source_wp.transform.rotation) - yaw_to_rad(target_wp.transform.rotation))
    )
    return yaw_error < math.radians(30.0)


def get_town10_start_waypoint(carla_map):
    """获取 Town10 固定起点，确保每次仿真从同一位置开始。"""
    spawn_points = sorted(
        carla_map.get_spawn_points(),
        key=lambda transform: (
            round(transform.location.x, 1),
            round(transform.location.y, 1),
            round(transform.rotation.yaw, 1),
        ),
    )

    if TOWN10_START_SPAWN_INDEX >= len(spawn_points):
        raise RuntimeError("Town10 fixed spawn index is out of range.")

    transform = spawn_points[TOWN10_START_SPAWN_INDEX]
    waypoint = carla_map.get_waypoint(
        transform.location, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    print(
        "Town10 fixed loop start: sorted_spawn_index={}, location=({:.1f}, {:.1f}), road={}, lane={}".format(
            TOWN10_START_SPAWN_INDEX,
            waypoint.transform.location.x,
            waypoint.transform.location.y,
            waypoint.road_id,
            waypoint.lane_id,
        )
    )
    return waypoint
