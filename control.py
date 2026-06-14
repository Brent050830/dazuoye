import math
import time
from dataclasses import dataclass

import carla

try:
    from scipy.optimize import minimize as scipy_minimize
except Exception:
    scipy_minimize = None

from config import FRONT_CONFLICT_LATERAL_MARGIN, MPC_DT, MPC_HORIZON_STEPS, WHEEL_BASE # MPC控制器的时间步长、预测时域步数和车辆轴距
from utils import clamp, dot_2d, get_speed, normalize_angle, smooth_reference_for, yaw_to_rad # 一些数学工具函数：clamp用于限制数值范围，dot_2d计算二维向量点积，get_speed获取车辆速度，normalize_angle将角度归一化到[-pi, pi]，yaw_to_rad将carla的旋转转换为弧度表示的航向角

MPC_DELTA_MAX_RAD = math.radians(32.0)
MPC_DELTA_RESPONSE_TAU = 0.22
MPC_DELTA_RATE_LIMIT_RAD = math.radians(70.0)
MPC_DYNAMIC_MIN_VX = 2.0
MPC_DYNAMIC_VX_SAFE = 2.0
MPC_VEHICLE_MASS = 1650.0
MPC_YAW_INERTIA = 2850.0
MPC_FRONT_AXLE = 1.25
MPC_REAR_AXLE = max(0.1, WHEEL_BASE - MPC_FRONT_AXLE)
MPC_FRONT_CORNERING_STIFFNESS = 55000.0
MPC_REAR_CORNERING_STIFFNESS = 65000.0
MPC_BETA_SOFT_LIMIT = math.radians(6.0)
MPC_BETA_HARD_LIMIT = math.radians(18.0)
MPC_YAW_RATE_SOFT_LIMIT = math.radians(55.0)
MPC_YAW_RATE_HARD_LIMIT = math.radians(120.0)
MPC_TIRE_SLIP_HARD_LIMIT = math.radians(20.0)
LTV_MPC_HORIZON_STEPS = min(10, MPC_HORIZON_STEPS)
LTV_MPC_MAX_ITER = 24
LTV_MPC_SOLVE_TIMEOUT_SECONDS = 0.80
LTV_MPC_STEER_LIMIT = 0.45


# ===================== 换道轨迹规划与 MPC 轨迹跟踪控制器 =====================

@dataclass
class AvoidancePathCandidate:
    """一条候选避障路径及其约束/代价诊断信息。"""

    trajectory: object # 换道轨迹对象，包含计算轨迹坐标和参考航向的方法
    length: float # 换道长度，表示从起点到终点沿全局路径的纵向距离
    transition_ratio: float # 横向偏移完成比例，表示在 length 的多少比例处完成侧向动作并开始保持目标偏移
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

    def __init__(
        self,
        loop_route,
        start_index,
        lateral_offset,
        length,
        start_offset=0.0,
        end_offset=None,
        transition_ratio=1.0,
    ):
        self.loop_route = loop_route
        self.start_index = max(0, min(start_index, len(loop_route.points) - 1))
        self.start_offset = start_offset
        self.lateral_offset = lateral_offset # 目标侧向偏移；本段内只从起点平滑过渡到该偏移，不再回收到基础路线
        self.end_offset = end_offset if end_offset is not None else lateral_offset
        self.length = length
        self.transition_ratio = clamp(transition_ratio, 0.05, 1.0)
        self.transition_length = max(0.001, self.length * self.transition_ratio)
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
        if s >= self.transition_length:
            return self.end_offset # 如果 s 大于等于横向过渡长度，直接保持结束侧向偏移量

        tau = s / self.transition_length # 计算当前 s 在横向过渡长度中的归一化位置 tau，范围在 [0, 1] 之间
        return self._blend_offset(self.start_offset, self.end_offset, tau)

    def _blend_offset(self, start, end, tau):
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5 # 五次多项式的值，表示侧向偏移随纵向位置的变化比例
        return start + (end - start) * blend

    def lateral_slope_at(self, s):
        """计算纵向位置 s 处的轨迹横向斜率（用于计算参考航向角）"""
        if s <= 0.0 or s >= self.transition_length:
            return 0.0
        tau = s / self.transition_length
        offset_delta = self.end_offset - self.start_offset # 计算侧向偏移的总变化量，表示从换道开始到结束的横向移动距离
        blend_dot = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4 # 五次多项式的导数值，表示侧向偏移随纵向位置变化的斜率，影响参考航向角的计算
        return offset_delta * blend_dot / self.transition_length # 计算横向斜率，表示轨迹在当前 s 位置的横向变化率，供参考航向角计算使用

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

    min_avoidance_length = front_distance if math.isfinite(front_distance) else 14.0
    length_values = _candidate_lengths(base_length, min_length=min_avoidance_length)
    target_values = _candidate_target_offsets(start_offset, lane_width)
    transition_values = _avoidance_transition_ratios()

    candidates = []
    for length in length_values: # 对于每个候选换道长度，生成多条候选轨迹，每条轨迹对应一个候选目标侧向偏移，并计算每条轨迹的约束满足情况和代价，最后从有效的候选中选取总代价最低的一条作为最终的换道轨迹
        for target_offset in target_values:
            ratios = (1.0,) if abs(target_offset - start_offset) < 0.05 else transition_values
            for transition_ratio in ratios:
                trajectory = RouteOffsetLaneChangeTrajectory( # 创建一条基于全局路径的换道轨迹，输入参数包括固定路线、起始索引、自车初始变换、目标侧向偏移、换道长度和起始侧向偏移
                    loop_route,
                    route_index,
                    target_offset,
                    length,
                    start_offset,
                    end_offset=target_offset,
                    transition_ratio=transition_ratio,
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
    return _select_preferred_avoidance_candidate(valid_candidates), candidates


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
        for transition_ratio in _return_transition_ratios():
            trajectory = RouteOffsetLaneChangeTrajectory(
                loop_route,
                route_index,
                0.0,
                length,
                start_offset,
                end_offset=0.0,
                transition_ratio=transition_ratio,
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


def _select_preferred_avoidance_candidate(valid_candidates):
    """普通避障优先选择向右偏移的安全候选；右侧无安全候选时再选左侧。"""
    right_candidates = [
        candidate for candidate in valid_candidates
        if candidate.target_offset > candidate.start_offset + 0.05
    ]
    if right_candidates:
        return min(right_candidates, key=lambda candidate: candidate.total_cost)

    left_candidates = [
        candidate for candidate in valid_candidates
        if candidate.target_offset < candidate.start_offset - 0.05
    ]
    if left_candidates:
        return min(left_candidates, key=lambda candidate: candidate.total_cost)

    return min(valid_candidates, key=lambda candidate: candidate.total_cost)


def _candidate_lengths(base_length, min_length=14.0):
    """围绕基础避障长度生成候选纵向长度，避免只固定一条路径。"""
    values = []
    lower_bound = clamp(min_length, 14.0, 56.0)
    for scale in (0.50, 0.60, 0.70, 0.80, 0.90, 1.0, 1.15, 1.30):
        length = clamp(base_length * scale, lower_bound, 56.0)
        if all(abs(length - existing) > 0.1 for existing in values):
            values.append(length)
    return values


def _avoidance_transition_ratios():
    return (0.75,0.8, 0.85,0.9, 1.0)


def _return_transition_ratios():
    return (0.85, 1.0)


def _candidate_target_offsets(start_offset, lane_width):
    """围绕当前路线左右生成候选峰值横向偏移。"""
    offsets = (
        -1.20, -1.10, -1.05, -1.00, -0.90, -0.70, -0.55, -0.45, -0.35, -0.25, -0.15,
        0.0,
        0.15, 0.25, 0.35, 0.45, 0.55, 0.70, 0.90, 1.00, 1.05, 1.10, 1.20,
    )
    values = []
    for scale in offsets: # 对于每个偏移量，计算对应的目标侧向偏移，并确保与现有候选值之间有足够的差距，避免生成过于相似的候选路径，增加换道策略的多样性和适应性
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
    transition_length = getattr(trajectory, "transition_length", length)
    transition_ratio = getattr(trajectory, "transition_ratio", 1.0)
    maneuver_time = transition_length / ego_speed # 横向动作实际完成时间，必须按过渡长度而不是整段长度计算
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
        transition_ratio=transition_ratio,
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
    check_length = length + 2.0
    sample_count = max(1, int(math.ceil(check_length / sample_step)))

    for actor in obstacle_actors:
        if actor is None or not actor.is_alive or actor.id == ego_actor_id: # 如果障碍物列表中的某个actor无效（None或已销毁）或者是自车本身，直接跳过该actor，不进行碰撞检查，避免无效数据导致错误的碰撞判断
            continue

        actor_half_length, actor_half_width = _actor_half_extents(actor) # 获取当前actor的半长和半宽，用于后续计算安全缓冲区，确保在换道过程中与该actor保持足够的距离，减少碰撞风险
        is_front_actor = front_actor_id is not None and actor.id == front_actor_id # 判断当前actor是否是前车，通过比较actor的ID与前车ID来确定，如果是前车，后续计算中会使用更大的安全缓冲区，以反映与前车潜在的更高风险
        longitudinal_buffer = ego_half_length + actor_half_length + (3.0 if is_front_actor else 2.0) # 计算纵向安全缓冲区，基于自车和actor的半长以及一个额外的安全距离（前车更大），确保在换道过程中与其他车辆保持足够的纵向距离，减少碰撞风险
        lateral_margin = FRONT_CONFLICT_LATERAL_MARGIN if is_front_actor else 0.25
        lateral_buffer = ego_half_width + actor_half_width + lateral_margin

        actor_loc = actor.get_location()
        projection = trajectory.route_reference.project(
            actor_loc,
            trajectory.start_route_s,
            search_back=15.0,
            search_ahead=check_length + longitudinal_buffer + 2.0,
        )
        actor_route_s = projection["route_s"]
        actor_lateral = projection["lateral"]
        tangent = carla.Vector3D(x=projection["right"].y, y=-projection["right"].x, z=0.0)
        actor_speed_along = dot_2d(actor.get_velocity(), tangent)
        if getattr(actor, "attributes", {}).get("role_name", "") == "lead":
            actor_speed_along = 0.0

        for index in range(sample_count + 1): # 遍历采样点
            local_s = min(check_length, index * check_length / sample_count)
            route_s = trajectory.start_route_s + local_s # 计算当前采样点在全局路径上的纵向位置，基于换道起点位置和当前采样点的局部纵向位置计算得到，表示当前采样点在全局路径上的位置，供后续预测使用
            time_to_sample = local_s / max(ego_speed, 0.1)
            predicted_actor_s = actor_route_s + actor_speed_along * time_to_sample # 预测actor在当前采样点时间点的纵向位置，基于actor当前在全局路径上的位置和沿切线方向的速度预测得到，表示在换道过程中与该actor可能发生交互的时间点，供后续碰撞检查使用
            longitudinal_gap = predicted_actor_s - route_s
            if abs(longitudinal_gap) > longitudinal_buffer:
                continue
            lateral_gap = actor_lateral - trajectory.avoidance_delta_at(local_s)
            if abs(lateral_gap) <= lateral_buffer:
                role_name = getattr(actor, "attributes", {}).get("role_name", actor.type_id)
                return "{}: actor={}, local_s={:.1f}, longitudinal_gap={:.1f}, lateral_gap={:.2f}, lateral_buffer={:.2f}".format(
                    "candidate conflicts with front vehicle" if is_front_actor else "candidate conflicts with vehicle",
                    role_name,
                    local_s,
                    longitudinal_gap,
                    lateral_gap,
                    lateral_buffer,
                )
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
    """Sampling MPC tracker with a linear-tire dynamic bicycle prediction model."""

    def __init__(self):
        self.previous_steer = 0.0
        self.previous_accel = 0.0
        self.previous_delta = 0.0

    def control(self, ego_vehicle, trajectory, target_speed): # 计算控制指令的主函数，输入自车对象、要跟踪的轨迹和目标速度，输出carla.VehicleControl对象
        """计算当前帧的最优控制指令
        返回：carla.VehicleControl（油门、制动、转向）
        """
        if not getattr(trajectory, "is_route_relative", False):
            raise ValueError("SamplingMPCTracker now only supports route-relative trajectories.")

        transform = ego_vehicle.get_transform()
        progress0, _ = trajectory.to_local(transform.location) # 将自车当前位置转换到轨迹的局部坐标系下，得到自车在轨迹上的弧长 s 和横向 d，供后续预测使用
        x0 = transform.location.x # 获取自车当前位置的全局坐标 x 和 y，以及航向角 yaw 和当前速度 v0，作为MPC预测的初始状态
        y0 = transform.location.y
        yaw0 = yaw_to_rad(transform.rotation)
        velocity = ego_vehicle.get_velocity()
        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)
        vx0 = velocity.x * cos_yaw + velocity.y * sin_yaw
        vy0 = -velocity.x * sin_yaw + velocity.y * cos_yaw
        speed0 = get_speed(ego_vehicle)
        if vx0 < 0.1 and speed0 > 0.1:
            vx0 = speed0
        vx0 = max(0.0, vx0)
        if vx0 < MPC_DYNAMIC_MIN_VX:
            vy0 = 0.0
        # CARLA angular velocity is reported in degrees/s; the model uses rad/s.
        yaw_rate0 = math.radians(ego_vehicle.get_angular_velocity().z)

        steer_candidates = [
            clamp(self.previous_steer + delta, -0.45, 0.45)
            for delta in (-0.45, -0.32, -0.20, -0.10, 0.0, 0.10, 0.20, 0.32, 0.45)
        ]
        if speed0 < max(3.0, target_speed * 0.5):
            accel_candidates = (0.0, 1.0)
        else:
            accel_candidates = (-4.0, -2.0, -1.0, 0.0, 1.0)

        best_cost = float("inf")
        best_action = (0.0, -3.0, self.previous_delta)
        previous_steer = self.previous_steer
        previous_accel = self.previous_accel

        for steer in steer_candidates:
            for accel in accel_candidates:
                x = x0
                y = y0
                yaw = yaw0
                vx = vx0
                vy = vy0
                yaw_rate = yaw_rate0
                steer_angle = self.previous_delta
                progress = progress0
                first_step_delta = steer_angle
                reject_candidate = False
                cost = 0.0
                steer_delta = steer - previous_steer
                accel_delta = accel - previous_accel
                delta_cmd = clamp(steer, -1.0, 1.0) * MPC_DELTA_MAX_RAD

                cost += 0.35 * steer_delta**2
                cost += 0.025 * accel_delta**2

                for step in range(MPC_HORIZON_STEPS):
                    steer_rate = clamp(
                        (delta_cmd - steer_angle) / MPC_DELTA_RESPONSE_TAU,
                        -MPC_DELTA_RATE_LIMIT_RAD,
                        MPC_DELTA_RATE_LIMIT_RAD,
                    )
                    steer_angle = clamp(
                        steer_angle + steer_rate * MPC_DT,
                        -MPC_DELTA_MAX_RAD,
                        MPC_DELTA_MAX_RAD,
                    )
                    if step == 0:
                        first_step_delta = steer_angle

                    if vx < MPC_DYNAMIC_MIN_VX:
                        vx = max(0.0, vx + accel * MPC_DT)
                        vy = 0.0
                        yaw_rate = vx / WHEEL_BASE * math.tan(steer_angle)
                        x += vx * math.cos(yaw) * MPC_DT
                        y += vx * math.sin(yaw) * MPC_DT
                        yaw = normalize_angle(yaw + yaw_rate * MPC_DT)
                        alpha_f = 0.0
                        alpha_r = 0.0
                    else:
                        vx_safe = max(abs(vx), MPC_DYNAMIC_VX_SAFE)
                        alpha_f = steer_angle - math.atan2(vy + MPC_FRONT_AXLE * yaw_rate, vx_safe)
                        alpha_r = -math.atan2(vy - MPC_REAR_AXLE * yaw_rate, vx_safe)
                        force_yf = MPC_FRONT_CORNERING_STIFFNESS * alpha_f
                        force_yr = MPC_REAR_CORNERING_STIFFNESS * alpha_r

                        x_dot = vx * math.cos(yaw) - vy * math.sin(yaw)
                        y_dot = vx * math.sin(yaw) + vy * math.cos(yaw)
                        vy_dot = (force_yf + force_yr) / MPC_VEHICLE_MASS - vx * yaw_rate
                        yaw_rate_dot = (MPC_FRONT_AXLE * force_yf - MPC_REAR_AXLE * force_yr) / MPC_YAW_INERTIA

                        x += x_dot * MPC_DT
                        y += y_dot * MPC_DT
                        yaw = normalize_angle(yaw + yaw_rate * MPC_DT)
                        vx = max(0.0, vx + accel * MPC_DT)
                        vy += vy_dot * MPC_DT
                        yaw_rate += yaw_rate_dot * MPC_DT

                    ref_yaw_for_progress = trajectory.reference_yaw_at(progress)
                    progress_yaw_error = normalize_angle(yaw - ref_yaw_for_progress)
                    progress += max(0.0, vx * math.cos(progress_yaw_error)) * MPC_DT

                    ref_location = trajectory.location_at(progress)
                    ref_yaw = trajectory.reference_yaw_at(progress)
                    dx = x - ref_location.x
                    dy = y - ref_location.y
                    tangent_x = math.cos(ref_yaw)
                    tangent_y = math.sin(ref_yaw)
                    longitudinal_error = dx * tangent_x + dy * tangent_y
                    lateral_error = -dx * tangent_y + dy * tangent_x
                    yaw_error = normalize_angle(yaw - ref_yaw)
                    speed_error = vx - target_speed
                    vx_safe_for_beta = max(abs(vx), MPC_DYNAMIC_VX_SAFE)
                    beta = math.atan2(vy, vx_safe_for_beta)

                    cost += 9.0 * lateral_error**2
                    cost += 0.45 * longitudinal_error**2
                    cost += 1.7 * yaw_error**2
                    cost += 0.07 * speed_error**2
                    cost += 0.08 * steer**2
                    cost += 0.01 * accel**2
                    cost += 0.004 * step * steer_delta**2
                    cost += 1.6 * beta**2
                    cost += 0.10 * yaw_rate**2

                    beta_excess = max(0.0, abs(beta) - MPC_BETA_SOFT_LIMIT)
                    yaw_rate_excess = max(0.0, abs(yaw_rate) - MPC_YAW_RATE_SOFT_LIMIT)
                    cost += 90.0 * beta_excess**2
                    cost += 8.0 * yaw_rate_excess**2

                    if (
                        abs(beta) > MPC_BETA_HARD_LIMIT
                        or abs(yaw_rate) > MPC_YAW_RATE_HARD_LIMIT
                        or abs(alpha_f) > MPC_TIRE_SLIP_HARD_LIMIT
                        or abs(alpha_r) > MPC_TIRE_SLIP_HARD_LIMIT
                    ):
                        reject_candidate = True
                        break

                    if step == MPC_HORIZON_STEPS - 1:
                        cost += 18.0 * lateral_error**2
                        cost += 5.0 * yaw_error**2
                        cost += 0.04 * speed_error**2

                if not reject_candidate and cost < best_cost:
                    best_cost = cost
                    best_action = (steer, accel, first_step_delta)

        steer, accel, first_step_delta = best_action
        self.previous_steer = steer
        self.previous_accel = accel
        self.previous_delta = first_step_delta

        if accel >= 0.0:
            throttle = clamp(0.25 + 0.18 * accel, 0.0, 0.65)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-accel / 7.5, 0.0, 1.0)

        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)


class LTVMPCTracker:
    """First-version LTV-MPC tracker.

    The implementation keeps the current CARLA-facing interface and uses
    scipy.optimize.minimize to optimize a short control sequence. The dynamic
    rollout is kept in the objective for now; it can later be replaced by an
    explicit A_k/B_k QP form without changing the caller.
    """

    def __init__(self):
        self.fallback_tracker = SamplingMPCTracker()
        self.previous_delta_cmd = 0.0
        self.previous_accel = 0.0
        self.previous_delta = 0.0
        self.previous_solution = None
        self._warned_fallback = False

    def control(self, ego_vehicle, trajectory, target_speed):
        if scipy_minimize is None:
            return self._fallback(ego_vehicle, trajectory, target_speed, "scipy unavailable")
        if not getattr(trajectory, "is_route_relative", False):
            return self._fallback(ego_vehicle, trajectory, target_speed, "trajectory is not route-relative")

        initial_state = self._initial_state(ego_vehicle, trajectory)
        if initial_state is None:
            return self._fallback(ego_vehicle, trajectory, target_speed, "invalid initial state")

        initial_guess = self._initial_guess()
        bounds = []
        for _ in range(LTV_MPC_HORIZON_STEPS):
            bounds.append((-LTV_MPC_STEER_LIMIT * MPC_DELTA_MAX_RAD, LTV_MPC_STEER_LIMIT * MPC_DELTA_MAX_RAD))
            bounds.append((-4.0, 1.0))

        start_time = time.perf_counter()
        try:
            result = scipy_minimize(
                self._objective,
                initial_guess,
                args=(initial_state, trajectory, target_speed),
                method="L-BFGS-B",
                bounds=bounds,
                options={"maxiter": LTV_MPC_MAX_ITER, "ftol": 1e-2, "maxls": 8, "disp": False},
            )
        except Exception as exc:
            return self._fallback(ego_vehicle, trajectory, target_speed, "solver exception: {}".format(exc))

        solve_time = time.perf_counter() - start_time
        if solve_time > LTV_MPC_SOLVE_TIMEOUT_SECONDS:
            return self._fallback(ego_vehicle, trajectory, target_speed, "solver timeout {:.3f}s".format(solve_time))
        if not getattr(result, "success", False):
            return self._fallback(ego_vehicle, trajectory, target_speed, "solver failed")
        if result.x is None or len(result.x) < 2:
            return self._fallback(ego_vehicle, trajectory, target_speed, "empty solution")

        delta_cmd = float(result.x[0])
        accel = float(result.x[1])
        if not (math.isfinite(delta_cmd) and math.isfinite(accel)):
            return self._fallback(ego_vehicle, trajectory, target_speed, "non-finite solution")

        delta_cmd = clamp(delta_cmd, -LTV_MPC_STEER_LIMIT * MPC_DELTA_MAX_RAD, LTV_MPC_STEER_LIMIT * MPC_DELTA_MAX_RAD)
        accel = clamp(accel, -4.0, 1.0)
        steer = clamp(delta_cmd / MPC_DELTA_MAX_RAD, -1.0, 1.0)

        if self._solution_unstable(initial_state, trajectory, target_speed, result.x):
            return self._fallback(ego_vehicle, trajectory, target_speed, "unstable solution")

        self.previous_solution = list(result.x)
        self.previous_delta_cmd = delta_cmd
        self.previous_accel = accel
        self.previous_delta = self._advance_delta(self.previous_delta, delta_cmd)
        self.fallback_tracker.previous_steer = steer
        self.fallback_tracker.previous_accel = accel
        self.fallback_tracker.previous_delta = self.previous_delta

        if accel >= 0.0:
            throttle = clamp(0.25 + 0.18 * accel, 0.0, 0.65)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-accel / 7.5, 0.0, 1.0)
        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)

    def _initial_state(self, ego_vehicle, trajectory):
        transform = ego_vehicle.get_transform()
        progress0, _ = trajectory.to_local(transform.location)
        yaw0 = yaw_to_rad(transform.rotation)
        velocity = ego_vehicle.get_velocity()
        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)
        vx0 = velocity.x * cos_yaw + velocity.y * sin_yaw
        vy0 = -velocity.x * sin_yaw + velocity.y * cos_yaw
        speed0 = get_speed(ego_vehicle)
        if vx0 < 0.1 and speed0 > 0.1:
            vx0 = speed0
        vx0 = max(0.0, vx0)
        if vx0 < MPC_DYNAMIC_MIN_VX:
            vy0 = 0.0
        yaw_rate0 = math.radians(ego_vehicle.get_angular_velocity().z)
        values = [transform.location.x, transform.location.y, yaw0, vx0, vy0, yaw_rate0, self.previous_delta, progress0]
        if any(not math.isfinite(value) for value in values):
            return None
        return values

    def _initial_guess(self):
        if self.previous_solution and len(self.previous_solution) == 2 * LTV_MPC_HORIZON_STEPS:
            shifted = list(self.previous_solution[2:])
            shifted.extend(self.previous_solution[-2:])
            return shifted
        guess = []
        for _ in range(LTV_MPC_HORIZON_STEPS):
            guess.append(self.previous_delta_cmd)
            guess.append(self.previous_accel)
        return guess

    def _objective(self, controls, initial_state, trajectory, target_speed):
        x, y, yaw, vx, vy, yaw_rate, steer_angle, progress = initial_state
        previous_delta_cmd = self.previous_delta_cmd
        previous_accel = self.previous_accel
        cost = 0.0

        for step in range(LTV_MPC_HORIZON_STEPS):
            delta_cmd = float(controls[2 * step])
            accel = float(controls[2 * step + 1])
            delta_cmd = clamp(delta_cmd, -LTV_MPC_STEER_LIMIT * MPC_DELTA_MAX_RAD, LTV_MPC_STEER_LIMIT * MPC_DELTA_MAX_RAD)
            accel = clamp(accel, -4.0, 1.0)

            delta_change = delta_cmd - previous_delta_cmd
            accel_change = accel - previous_accel
            rate_excess = max(0.0, abs(delta_change) - MPC_DELTA_RATE_LIMIT_RAD * MPC_DT)
            cost += 4.0 * delta_change**2 + 0.035 * accel_change**2 + 800.0 * rate_excess**2
            previous_delta_cmd = delta_cmd
            previous_accel = accel

            x, y, yaw, vx, vy, yaw_rate, steer_angle, alpha_f, alpha_r = self._dynamic_step(
                x, y, yaw, vx, vy, yaw_rate, steer_angle, delta_cmd, accel
            )
            if not all(math.isfinite(value) for value in (x, y, yaw, vx, vy, yaw_rate, steer_angle, alpha_f, alpha_r)):
                return 1e9

            ref_yaw_for_progress = trajectory.reference_yaw_at(progress)
            progress_yaw_error = normalize_angle(yaw - ref_yaw_for_progress)
            progress += max(0.0, vx * math.cos(progress_yaw_error)) * MPC_DT

            ref_location = trajectory.location_at(progress)
            ref_yaw = trajectory.reference_yaw_at(progress)
            lateral_error, longitudinal_error, yaw_error = self._tracking_errors(x, y, yaw, ref_location, ref_yaw)
            speed_error = vx - target_speed
            beta = math.atan2(vy, max(abs(vx), MPC_DYNAMIC_VX_SAFE))

            cost += 10.0 * lateral_error**2
            cost += 0.25 * longitudinal_error**2
            cost += 2.2 * yaw_error**2
            cost += 0.06 * speed_error**2
            cost += 0.45 * delta_cmd**2
            cost += 0.012 * accel**2
            cost += 1.8 * beta**2
            cost += 0.12 * yaw_rate**2

            beta_excess = max(0.0, abs(beta) - MPC_BETA_SOFT_LIMIT)
            yaw_rate_excess = max(0.0, abs(yaw_rate) - MPC_YAW_RATE_SOFT_LIMIT)
            cost += 110.0 * beta_excess**2
            cost += 10.0 * yaw_rate_excess**2

            if abs(beta) > MPC_BETA_HARD_LIMIT or abs(yaw_rate) > MPC_YAW_RATE_HARD_LIMIT:
                return 1e8 + cost
            if abs(alpha_f) > MPC_TIRE_SLIP_HARD_LIMIT or abs(alpha_r) > MPC_TIRE_SLIP_HARD_LIMIT:
                return 1e8 + cost

            if step == LTV_MPC_HORIZON_STEPS - 1:
                cost += 22.0 * lateral_error**2
                cost += 7.0 * yaw_error**2
                cost += 0.04 * speed_error**2

        return cost

    def _dynamic_step(self, x, y, yaw, vx, vy, yaw_rate, steer_angle, delta_cmd, accel):
        steer_angle = self._advance_delta(steer_angle, delta_cmd)
        if vx < MPC_DYNAMIC_MIN_VX:
            vx = max(0.0, vx + accel * MPC_DT)
            vy = 0.0
            yaw_rate = vx / WHEEL_BASE * math.tan(steer_angle)
            x += vx * math.cos(yaw) * MPC_DT
            y += vx * math.sin(yaw) * MPC_DT
            yaw = normalize_angle(yaw + yaw_rate * MPC_DT)
            return x, y, yaw, vx, vy, yaw_rate, steer_angle, 0.0, 0.0

        vx_safe = max(abs(vx), MPC_DYNAMIC_VX_SAFE)
        alpha_f = steer_angle - math.atan2(vy + MPC_FRONT_AXLE * yaw_rate, vx_safe)
        alpha_r = -math.atan2(vy - MPC_REAR_AXLE * yaw_rate, vx_safe)
        force_yf = MPC_FRONT_CORNERING_STIFFNESS * alpha_f
        force_yr = MPC_REAR_CORNERING_STIFFNESS * alpha_r

        x_dot = vx * math.cos(yaw) - vy * math.sin(yaw)
        y_dot = vx * math.sin(yaw) + vy * math.cos(yaw)
        vy_dot = (force_yf + force_yr) / MPC_VEHICLE_MASS - vx * yaw_rate
        yaw_rate_dot = (MPC_FRONT_AXLE * force_yf - MPC_REAR_AXLE * force_yr) / MPC_YAW_INERTIA

        x += x_dot * MPC_DT
        y += y_dot * MPC_DT
        yaw = normalize_angle(yaw + yaw_rate * MPC_DT)
        vx = max(0.0, vx + accel * MPC_DT)
        vy += vy_dot * MPC_DT
        yaw_rate += yaw_rate_dot * MPC_DT
        return x, y, yaw, vx, vy, yaw_rate, steer_angle, alpha_f, alpha_r

    def _advance_delta(self, current_delta, delta_cmd):
        steer_rate = clamp(
            (delta_cmd - current_delta) / MPC_DELTA_RESPONSE_TAU,
            -MPC_DELTA_RATE_LIMIT_RAD,
            MPC_DELTA_RATE_LIMIT_RAD,
        )
        return clamp(current_delta + steer_rate * MPC_DT, -MPC_DELTA_MAX_RAD, MPC_DELTA_MAX_RAD)

    def _tracking_errors(self, x, y, yaw, ref_location, ref_yaw):
        dx = x - ref_location.x
        dy = y - ref_location.y
        tangent_x = math.cos(ref_yaw)
        tangent_y = math.sin(ref_yaw)
        longitudinal_error = dx * tangent_x + dy * tangent_y
        lateral_error = -dx * tangent_y + dy * tangent_x
        yaw_error = normalize_angle(yaw - ref_yaw)
        return lateral_error, longitudinal_error, yaw_error

    def _solution_unstable(self, initial_state, trajectory, target_speed, controls):
        return self._objective(controls, initial_state, trajectory, target_speed) >= 1e8

    def _fallback(self, ego_vehicle, trajectory, target_speed, reason):
        if not self._warned_fallback:
            print("LTV-MPC fallback to SamplingMPCTracker: {}".format(reason))
            self._warned_fallback = True
        control = self.fallback_tracker.control(ego_vehicle, trajectory, target_speed)
        self.previous_delta_cmd = clamp(control.steer, -1.0, 1.0) * MPC_DELTA_MAX_RAD
        self.previous_accel = 0.0
        self.previous_delta = getattr(self.fallback_tracker, "previous_delta", self.previous_delta)
        self.previous_solution = None
        return control


