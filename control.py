import math
from dataclasses import dataclass

import carla

from config import MPC_DT, MPC_HORIZON_STEPS, WHEEL_BASE # MPC控制器的时间步长、预测时域步数和车辆轴距
from utils import clamp, dot_2d, get_speed, normalize_angle, smooth_reference_for, yaw_to_rad # 一些数学工具函数：clamp用于限制数值范围，dot_2d计算二维向量点积，get_speed获取车辆速度，normalize_angle将角度归一化到[-pi, pi]，yaw_to_rad将carla的旋转转换为弧度表示的航向角


# ===================== 换道轨迹规划与 MPC 轨迹跟踪控制器 =====================

@dataclass
class AvoidancePathCandidate:
    """一条候选避障路径及其约束/代价诊断信息。"""

    trajectory: object # 换道轨迹对象，包含计算轨迹坐标和参考航向的方法
    length: float # 换道长度，表示从起点到终点沿全局路径的纵向距离
    start_offset: float # 起始侧向偏移量，表示换道开始时相对于全局路径的横向位置
    target_offset: float # 目标侧向偏移量，表示换道结束时相对于全局路径的横向位置
    lateral_shift: float # 侧向位移，表示车辆在换道过程中的横向移动距离
    lateral_accel: float # 侧向加速度，表示车辆在换道过程中的横向加速度
    safety_cost: float # 安全代价，表示换道过程中的安全风险
    comfort_cost: float # 舒适代价，表示换道过程中的舒适性影响
    tracking_cost: float # 跟踪代价，表示车辆跟踪轨迹的性能
    total_cost: float # 总代价，表示候选路径的综合评价
    is_valid: bool # 标记路径是否有效
    reject_reason: str = ""
    start_route_s: float = 0.0
    end_route_s: float = 0.0


class RouteOffsetLaneChangeTrajectory:
    """基础路线上的局部偏移候选段，段尾保持目标偏移，不在本段内强制回到基础路线。"""

    def __init__(self, loop_route, start_index, lateral_offset, length, start_offset=0.0, end_offset=None):
        self.loop_route = loop_route
        self.start_index = max(0, min(start_index, len(loop_route.points) - 1))
        self.start_offset = start_offset
        self.lateral_offset = lateral_offset # 目标侧向偏移；本段内只从起点平滑过渡到该偏移，不再回收到基础路线
        self.end_offset = end_offset if end_offset is not None else lateral_offset
        self.length = length
        self.step_distance = loop_route.step_distance
        self.route_reference = smooth_reference_for(loop_route) # 获取全局路径的平滑参考线对象，提供在全局路径上计算坐标和右向量的方法
        self.start_route_s = self.start_index * self.step_distance # 换道起点在全局路径上的纵向位置，基于起始索引和步距计算得到，表示换道开始时在全局路径上的位置，供后续计算使用
        self.end_route_s = self.start_route_s + self.length # 换道终点在全局路径上的纵向位置，基于起点位置和换道长度计算得到，表示换道结束时在全局路径上的位置，供后续计算使用
        self.is_route_relative = True

    def _route_pose_at(self, s):
        """计算全局路径上纵向位置 s 处的坐标和右向单位向量"""
        route_s = self.start_route_s + s
        location = self.route_reference.location_at_route_s(route_s)
        right = self.route_reference.right_at_route_s(route_s)
        return location, right

    def avoidance_delta_at(self, s):
        """计算纵向位置 s 处的侧向偏移量：从当前偏移平滑过渡到目标偏移并保持。"""
        if s <= 0.0:
            return self.start_offset # 如果 s 小于等于 0，直接返回起始侧向偏移量，表示换道开始时的横向位置
        if s >= self.length:
            return self.end_offset # 如果 s 大于等于换道长度，直接返回结束侧向偏移量，表示换道结束时的横向位置

        tau = s / max(self.length, 0.001) # 计算当前 s 在换道长度中的归一化位置 tau，范围在 [0, 1] 之间，表示换道过程中的进度
        return self._blend_offset(self.start_offset, self.end_offset, tau)

    def _blend_offset(self, start, end, tau):
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5 # 五次多项式的值，表示侧向偏移随纵向位置的变化比例
        return start + (end - start) * blend

    def lateral_slope_at(self, s):
        """计算纵向位置 s 处的轨迹横向斜率（用于计算参考航向角）"""
        if s <= 0.0 or s >= self.length:
            return 0.0
        tau = (s) / max(self.length, 0.001)
        offset_delta = self.end_offset - self.start_offset # 计算侧向偏移的总变化量，表示从换道开始到结束的横向移动距离
        blend_dot = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4 # 五次多项式的导数值，表示侧向偏移随纵向位置变化的斜率，影响参考航向角的计算
        return offset_delta * blend_dot / max(self.length, 0.001) # 计算横向斜率，表示轨迹在当前 s 位置的横向变化率，供参考航向角计算使用

    def location_at(self, s):
        """计算纵向位置 s 处的全局坐标，基于全局路径坐标加上侧向偏移"""
        route_location, route_right = self._route_pose_at(s) # 获取全局路径上 s 位置的坐标和右向单位向量
        lateral = self.avoidance_delta_at(s) # 计算当前 s 位置的侧向偏移量
        return route_location + carla.Location( # 在全局路径坐标的基础上添加侧向偏移，形成换道轨迹的坐标
            x=route_right.x * lateral,
            y=route_right.y * lateral,
            z=0.0,
        )

    def reference_yaw_at(self, s):
        """计算纵向位置 s 处的参考航向角，基于前后位置的坐标差计算切线方向"""
        ds = max(0.5, self.step_distance * 0.25)
        before = self.location_at(max(0.0, s - ds))
        after = self.location_at(s + ds)
        return math.atan2(after.y - before.y, after.x - before.x)

    def to_local(self, location): 
        """将全局坐标转换为以起点为原点的局部纵横坐标 (s, d)，其中 s 是沿全局路径的进度，d 是相对于全局路径的横向偏移"""
        projection = self.route_reference.project(
            location,
            self.start_route_s,
            search_back=12.0,
            search_ahead=self.length + 32.0,
        )
        progress = max(0.0, projection["route_s"] - self.start_route_s)
        lateral = projection["lateral"] # 通过当前位置到平滑参考线的投影计算横向偏移量 d
        return progress, lateral # 返回局部坐标 (s, d)，其中 s 是沿全局路径的进度，d 是相对于全局路径的横向偏移

    def replacement_points(self, step=2.0):
        points = []
        sample_count = max(1, int(math.ceil(self.length / max(step, 0.5))))
        for index in range(sample_count + 1):
            s = min(self.length, index * self.length / sample_count)
            points.append(self.location_at(s))
        return points


def smoothed_route_right_at(loop_route, index):
    """按路线中心线差分计算指定索引处的平滑右向量。"""
    return smooth_reference_for(loop_route).right_at_route_s(index * loop_route.step_distance)


def select_best_route_offset_trajectory(loop_route, ego_vehicle, front, base_length, obstacle_actors=None):
    """生成左右多偏移避障替换段，按约束和车辆冲突筛选最优候选。"""
    route_index = loop_route.last_index
    start_transform = ego_vehicle.get_transform()
    route_reference = smooth_reference_for(loop_route)
    route_location = route_reference.location_at_route_s(route_index * loop_route.step_distance)
    route_right = smoothed_route_right_at(loop_route, route_index)
    start_offset = dot_2d(start_transform.location - route_location, route_right)
    lane_width = max(loop_route.waypoints[route_index].lane_width, 2.5)
    ego_speed = max(get_speed(ego_vehicle), 4.0)
    front_distance = getattr(front, "distance", float("inf"))
    front_ttc = getattr(front, "ttc", float("inf"))
    front_target_speed = max(0.0, getattr(front, "target_speed_along", 0.0))
    front_actor_id = getattr(front, "actor_id", None)

    length_values = _candidate_lengths(base_length)
    target_values = _candidate_target_offsets(start_offset, lane_width)

    candidates = []
    for length in length_values: # 对于每个候选换道长度，生成多条候选轨迹，每条轨迹对应一个候选目标侧向偏移，并计算每条轨迹的约束满足情况和代价，最后从有效的候选中选取总代价最低的一条作为最终的换道轨迹
        for target_offset in target_values:
            trajectory = RouteOffsetLaneChangeTrajectory( # 创建一条基于全局路径的换道轨迹，输入参数包括固定路线、起始索引、自车初始变换、目标侧向偏移、换道长度和起始侧向偏移
                loop_route, route_index, target_offset, length, start_offset, end_offset=target_offset
            )
            candidates.append(
                _score_avoidance_candidate( # 计算每条候选避障路径的约束满足情况和代价，输入轨迹对象、换道长度、起始偏移、目标偏移、目标中心偏移、车道宽度、自车速度、前车距离和前车TTC，输出一个包含轨迹和相关信息的AvoidancePathCandidate对象
                    trajectory,
                    length,
                    start_offset,
                    target_offset,
                    lane_width,
                    ego_speed,
                    front_distance,
                    front_ttc,
                    front_target_speed,
                    obstacle_actors or [],
                    ego_vehicle,
                    front_actor_id,
                )
            )

    valid_candidates = [candidate for candidate in candidates if candidate.is_valid]
    if not valid_candidates:
        return None, candidates
    return min(valid_candidates, key=lambda candidate: candidate.total_cost), candidates


def select_return_to_base_trajectory(loop_route, ego_vehicle, base_length, obstacle_actors=None):
    """生成从当前横向偏移回到基础路线的候选段，安全后才允许采用。"""
    route_index = loop_route.last_index
    start_transform = ego_vehicle.get_transform()
    route_reference = smooth_reference_for(loop_route)
    route_location = route_reference.location_at_route_s(route_index * loop_route.step_distance)
    route_right = smoothed_route_right_at(loop_route, route_index)
    start_offset = dot_2d(start_transform.location - route_location, route_right)
    if abs(start_offset) < 0.20:
        return None, []

    lane_width = max(loop_route.waypoints[route_index].lane_width, 2.5)
    ego_speed = max(get_speed(ego_vehicle), 4.0)
    candidates = []
    for length in _candidate_lengths(base_length):
        trajectory = RouteOffsetLaneChangeTrajectory(
            loop_route,
            route_index,
            0.0,
            length,
            start_offset,
            end_offset=0.0,
        )
        candidates.append(
            _score_avoidance_candidate(
                trajectory,
                length,
                start_offset,
                0.0,
                lane_width,
                ego_speed,
                float("inf"),
                float("inf"),
                0.0,
                obstacle_actors or [],
                ego_vehicle,
                None,
            )
        )

    valid_candidates = [candidate for candidate in candidates if candidate.is_valid]
    if not valid_candidates:
        return None, candidates
    return min(valid_candidates, key=lambda candidate: candidate.total_cost), candidates


def _candidate_lengths(base_length):
    """围绕基础避障长度生成候选纵向长度，避免只固定一条路径。"""
    values = []
    for scale in (0.85, 1.0, 1.15, 1.30):
        length = clamp(base_length * scale, 14.0, 56.0)
        if all(abs(length - existing) > 0.1 for existing in values):
            values.append(length)
    return values


def _candidate_target_offsets(start_offset, lane_width):
    """围绕当前路线左右生成候选峰值横向偏移。"""
    offsets = (-1.20, -0.90, -0.60, -0.35, -0.20, 0.0, 0.20, 0.35, 0.60, 0.90, 1.20)
    values = []
    for scale in offsets:
        target = start_offset + scale * lane_width
        if all(abs(target - existing) > 0.05 for existing in values):
            values.append(target)
    return values


def _score_avoidance_candidate( # 计算每条候选避障路径的约束满足情况和代价，输入轨迹对象、换道长度、起始偏移、目标偏移、目标中心偏移、车道宽度、自车速度、前车距离和前车TTC，输出一个包含轨迹和相关信息的AvoidancePathCandidate对象
    trajectory,
    length,
    start_offset,
    target_offset,
    lane_width,
    ego_speed,
    front_distance,
    front_ttc,
    front_target_speed,
    obstacle_actors,
    ego_vehicle,
    front_actor_id,
):
    lateral_shift = target_offset - start_offset # 计算侧向位移，表示车辆在换道过程中的横向移动距离，即目标偏移与起始偏移之间的差值
    maneuver_time = length / ego_speed # 估算换道所需时间，基于换道长度和自车速度计算得到，表示完成换道所需的时间
    lateral_accel = 10.0 * math.sqrt(3.0) * abs(lateral_shift) / (3.0 * max(maneuver_time * maneuver_time, 0.01))
    max_lateral_accel = 3.8
    reject_reason = ""

    if length < 14.0:
        reject_reason = "length too short"
    elif lateral_accel > max_lateral_accel:
        reject_reason = "lateral acceleration too high"
    predicted_front_motion = front_target_speed * maneuver_time
    front_clear_distance = front_distance + predicted_front_motion

    if reject_reason == "":
        reject_reason = _candidate_collision_reason(
            trajectory,
            length,
            ego_speed,
            obstacle_actors,
            ego_vehicle,
            front_actor_id,
        )

    center_error = abs(target_offset) / lane_width
    safety_cost = _safety_cost(length, maneuver_time, front_clear_distance, front_ttc)
    comfort_cost = (lateral_accel / max_lateral_accel) ** 2 + 0.20 * abs(lateral_shift) / lane_width
    tracking_cost = _tracking_cost(trajectory, length)
    total_cost = 4.0 * safety_cost + 2.0 * comfort_cost + tracking_cost + 0.6 * center_error

    return AvoidancePathCandidate( # 创建一个AvoidancePathCandidate对象，包含轨迹对象、换道长度、起始偏移、目标偏移、侧向位移、侧向加速度、安全代价、舒适代价、跟踪代价、总代价、有效性标志和拒绝原因等信息，供后续选择和分析使用
        trajectory=trajectory,
        length=length,
        start_offset=start_offset,
        target_offset=target_offset,
        lateral_shift=lateral_shift,
        lateral_accel=lateral_accel,
        safety_cost=safety_cost,
        comfort_cost=comfort_cost,
        tracking_cost=tracking_cost,
        total_cost=total_cost,
        is_valid=(reject_reason == ""),
        reject_reason=reject_reason,
        start_route_s=trajectory.start_route_s,
        end_route_s=trajectory.end_route_s,
    )


def _safety_cost(length, maneuver_time, front_distance, front_ttc):
    """安全代价：TTC 越紧迫越偏向较短、较快完成横向避障的路径。"""
    distance_cost = 0.0
    if math.isfinite(front_distance): # 如果前车距离是有限的，计算距离代价，距离越近代价越高，鼓励选择较短的换道路径
        distance_cost = max(0.0, length * 0.75 - front_distance + 4.0) ** 2 / 25.0
    ttc_cost = 0.0
    if math.isfinite(front_ttc): # 如果前车TTC是有限的，计算TTC代价，TTC越紧迫（越小）代价越高，鼓励选择较快完成换道的路径
        ttc_cost = max(0.0, maneuver_time - front_ttc + 0.4) ** 2 # 如果换道所需时间超过前车TTC，说明在换道过程中可能会与前车发生冲突，代价会显著增加，鼓励选择更快完成换道的路径以避开前车
    return distance_cost + ttc_cost


def _actor_half_extents(actor, default_length=2.4, default_width=1.0):
    """返回车辆包围盒半长、半宽；无包围盒时使用保守默认值。"""
    bbox = getattr(actor, "bounding_box", None)
    extent = getattr(bbox, "extent", None)
    if extent is None:
        return default_length, default_width
    return max(0.1, float(extent.x)), max(0.1, float(extent.y))


def _candidate_collision_reason(trajectory, length, ego_speed, obstacle_actors, ego_vehicle, front_actor_id):
    """用当前候选路径与所有车辆的简化时空包络筛掉明显冲突路径。"""
    if not obstacle_actors:
        return ""

    ego_actor_id = getattr(ego_vehicle, "id", None)
    ego_half_length, ego_half_width = _actor_half_extents(ego_vehicle)
    sample_step = 1.0
    sample_count = max(1, int(math.ceil(length / sample_step)))

    for actor in obstacle_actors:
        if actor is None or not actor.is_alive or actor.id == ego_actor_id:
            continue

        actor_half_length, actor_half_width = _actor_half_extents(actor)
        is_front_actor = front_actor_id is not None and actor.id == front_actor_id
        longitudinal_buffer = ego_half_length + actor_half_length + (3.0 if is_front_actor else 2.0)
        lateral_buffer = ego_half_width + actor_half_width + (1.0 if is_front_actor else 0.6)

        actor_loc = actor.get_location()
        projection = trajectory.route_reference.project(
            actor_loc,
            trajectory.start_route_s,
            search_back=15.0,
            search_ahead=length + longitudinal_buffer + 30.0,
        )
        actor_route_s = projection["route_s"]
        actor_lateral = projection["lateral"]
        tangent = carla.Vector3D(x=projection["right"].y, y=-projection["right"].x, z=0.0)
        actor_speed_along = dot_2d(actor.get_velocity(), tangent)
        if getattr(actor, "attributes", {}).get("role_name", "") == "lead":
            actor_speed_along = 0.0

        for index in range(sample_count + 1):
            local_s = min(length, index * length / sample_count)
            route_s = trajectory.start_route_s + local_s
            time_to_sample = local_s / max(ego_speed, 0.1)
            predicted_actor_s = actor_route_s + actor_speed_along * time_to_sample
            longitudinal_gap = predicted_actor_s - route_s
            if abs(longitudinal_gap) > longitudinal_buffer:
                continue
            lateral_gap = actor_lateral - trajectory.avoidance_delta_at(local_s)
            if abs(lateral_gap) <= lateral_buffer:
                return "candidate conflicts with front vehicle" if is_front_actor else "candidate conflicts with vehicle"
    return ""


def _tracking_cost(trajectory, length):
    """跟踪难度代价：参考航向变化和五次曲线斜率越大，MPC 越难稳定跟踪。"""
    samples = 6
    yaws = []
    max_slope = 0.0
    for index in range(samples + 1):
        s = length * index / samples
        yaws.append(trajectory.reference_yaw_at(s))
        max_slope = max(max_slope, abs(trajectory.lateral_slope_at(s)))
    yaw_variation = 0.0
    for before, after in zip(yaws, yaws[1:]):
        yaw_variation += abs(normalize_angle(after - before))
    return 0.35 * yaw_variation + 0.30 * max_slope


class SamplingMPCTracker:
    """基于采样的后退时域MPC控制器，使用运动学自行车模型进行滚动优化。
    通过枚举转向角和加速度候选值，选取预测代价最小的控制动作。
    """

    def __init__(self):
        self.previous_steer = 0.0  # 上一帧的转向量，用于连续性惩罚

    def control(self, ego_vehicle, trajectory, target_speed): # 计算控制指令的主函数，输入自车对象、要跟踪的轨迹和目标速度，输出carla.VehicleControl对象
        """计算当前帧的最优控制指令
        返回：carla.VehicleControl（油门、制动、转向）
        """
        if not getattr(trajectory, "is_route_relative", False):
            raise ValueError("SamplingMPCTracker now only supports route-relative trajectories.")

        transform = ego_vehicle.get_transform()
        progress0, _ = trajectory.to_local(transform.location) # 将自车当前位置转换到轨迹的局部坐标系下，得到自车在轨迹上的弧长 s 和横向 d，供后续预测使用
        x0 = transform.location.x
        y0 = transform.location.y
        yaw0 = yaw_to_rad(transform.rotation)
        v0 = get_speed(ego_vehicle)

        steer_candidates = [
            clamp(self.previous_steer + delta, -0.45, 0.45)
            for delta in (-0.45, -0.32, -0.20, -0.10, 0.0, 0.10, 0.20, 0.32, 0.45)
        ]
        if v0 < max(3.0, target_speed * 0.5):
            accel_candidates = (0.0, 1.0)
        else:
            accel_candidates = (-4.0, -2.0, -1.0, 0.0, 1.0)

        best_cost = float("inf")
        best_action = (0.0, -3.0)

        for steer in steer_candidates:
            """对于每个候选转向角，遍历所有候选加速度，进行前向积分预测，并计算代价函数，选取代价最小的动作"""
            for accel in accel_candidates:
                """初始化预测状态为当前状态"""
                x = x0
                y = y0
                yaw = yaw0
                speed = v0
                progress = progress0
                cost = 0.0

                for step in range(MPC_HORIZON_STEPS):
                    """基于运动学自行车模型进行前向积分预测，计算每个时间步的状态，并根据轨迹计算误差和代价"""
                    speed = max(0.0, speed + accel * MPC_DT)
                    distance_step = speed * MPC_DT
                    x += distance_step * math.cos(yaw)
                    y += distance_step * math.sin(yaw)
                    progress += distance_step
                    yaw = normalize_angle(yaw + speed / WHEEL_BASE * math.tan(steer) * MPC_DT)

                    ref_location = trajectory.location_at(progress) # 根据预测的进度从轨迹上获取参考位置和参考航向，计算预测状态与轨迹参考状态之间的误差，包括位置误差、航向误差和速度误差
                    ref_yaw = trajectory.reference_yaw_at(progress) # 根据预测的进度从轨迹上获取参考位置和参考航向，计算预测状态与轨迹参考状态之间的误差，包括位置误差、航向误差和速度误差
                    dx = x - ref_location.x
                    dy = y - ref_location.y
                    position_error = math.sqrt(dx * dx + dy * dy)
                    yaw_error = normalize_angle(yaw - ref_yaw)
                    speed_error = speed - target_speed # 计算位置误差、航向误差和速度误差，分别表示预测状态与轨迹参考状态之间的偏差

                    cost += 6.0 * position_error**2
                    cost += 1.7 * yaw_error**2
                    cost += 0.07 * speed_error**2 # 代价函数中包含位置误差、航向误差和速度误差的平方项，分别乘以权重系数，鼓励控制动作能够使车辆更好地跟踪轨迹，同时保持接近目标速度
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


