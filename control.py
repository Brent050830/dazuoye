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


class RouteOffsetLaneChangeTrajectory:
    """基于全局路径的换道轨迹：在给定的全局路径上生成一个带有侧向偏移的轨迹。
通过在全局路径上进行插值，计算出每个纵向位置 s 对应的全局坐标，并在此基础上添加侧向偏移，形成换道轨迹。
    """

    def __init__(self, loop_route, start_index, start_transform, lateral_offset, length, start_offset=0.0):
        """初始化换道轨迹"""
        self.loop_route = loop_route
        self.start_index = max(0, min(start_index, len(loop_route.points) - 1))
        self.start_offset = start_offset # 起始侧向偏移量，表示换道开始时相对于全局路径的横向位置
        self.lateral_offset = lateral_offset # 目标侧向偏移量，表示换道结束时相对于全局路径的横向位置
        self.length = length # 换道长度，表示从起点到终点沿全局路径的纵向距离
        self.step_distance = loop_route.step_distance
        self.route_reference = smooth_reference_for(loop_route)
        self.start_route_s = self.start_index * self.step_distance
        self.is_route_relative = True

    def _route_pose_at(self, s):
        """计算全局路径上纵向位置 s 处的坐标和右向单位向量"""
        route_s = self.start_route_s + s
        location = self.route_reference.location_at_route_s(route_s)
        right = self.route_reference.right_at_route_s(route_s)
        return location, right

    def avoidance_delta_at(self, s):
        """计算纵向位置 s 处的侧向偏移量，使用五次多项式进行平滑过渡"""
        if s <= 0.0: # 如果 s 小于等于0，说明在起点之前，侧向偏移为起始偏移
            return self.start_offset
        if s >= self.length: # 如果 s 大于等于换道长度，说明在终点之后，侧向偏移为目标偏移
            return self.lateral_offset
        tau = s / self.length
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5 # 五次多项式的值，表示侧向偏移随纵向位置的变化比例
        return self.start_offset + (self.lateral_offset - self.start_offset) * blend # 通过线性插值计算当前侧向偏移，起始偏移和目标偏移之间根据五次多项式的值进行平滑过渡

    def lateral_at(self, s):
        """计算纵向位置 s 处的目标横向偏移量"""
        return self.avoidance_delta_at(s)

    def lateral_slope_at(self, s):
        """计算纵向位置 s 处的轨迹横向斜率（用于计算参考航向角）"""
        if s <= 0.0 or s >= self.length:
            return 0.0
        tau = s / self.length
        blend_dot = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
        return (self.lateral_offset - self.start_offset) * blend_dot / self.length

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
        return progress, lateral


def smoothed_route_right_at(loop_route, index):
    """按路线中心线差分计算指定索引处的平滑右向量。"""
    return smooth_reference_for(loop_route).right_at_route_s(index * loop_route.step_distance)


def select_best_route_offset_trajectory(loop_route, ego_vehicle, target_wp, front, base_length):
    """生成多条路线相对避障候选轨迹，按约束筛选并返回总代价最低的一条。"""
    if target_wp is None:
        return None, []

    route_index = loop_route.last_index
    start_transform = ego_vehicle.get_transform()
    route_reference = smooth_reference_for(loop_route)
    route_location = route_reference.location_at_route_s(route_index * loop_route.step_distance)
    route_right = smoothed_route_right_at(loop_route, route_index)
    start_offset = dot_2d(start_transform.location - route_location, route_right)
    target_center_offset = dot_2d(target_wp.transform.location - route_location, route_right)
    lane_width = max(target_wp.lane_width, 2.5)
    ego_speed = max(get_speed(ego_vehicle), 4.0)
    front_distance = getattr(front, "distance", float("inf"))
    front_ttc = getattr(front, "ttc", float("inf"))
    front_target_speed = max(0.0, getattr(front, "target_speed_along", 0.0))

    length_values = _candidate_lengths(base_length)
    target_values = _candidate_target_offsets(start_offset, target_center_offset, lane_width)

    candidates = []
    for length in length_values: # 对于每个候选换道长度，生成多条候选轨迹，每条轨迹对应一个候选目标侧向偏移，并计算每条轨迹的约束满足情况和代价，最后从有效的候选中选取总代价最低的一条作为最终的换道轨迹
        for target_offset in target_values:
            trajectory = RouteOffsetLaneChangeTrajectory(
                loop_route, route_index, start_transform, target_offset, length, start_offset
            )
            candidates.append(
                _score_avoidance_candidate(
                    trajectory,
                    length,
                    start_offset,
                    target_offset,
                    target_center_offset,
                    lane_width,
                    ego_speed,
                    front_distance,
                    front_ttc,
                    front_target_speed,
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


def _candidate_target_offsets(start_offset, target_center_offset, lane_width):
    """围绕目标邻道中心生成候选横向终点，并限制在邻道中心附近。"""
    side = 1.0 if target_center_offset >= start_offset else -1.0
    nudges = (-0.20 * lane_width * side, 0.0, 0.15 * lane_width * side)
    values = []
    for nudge in nudges:
        target = target_center_offset + nudge
        if abs(target - target_center_offset) <= lane_width * 0.35:
            values.append(target)
    return values


def _score_avoidance_candidate( # 计算每条候选避障路径的约束满足情况和代价，输入轨迹对象、换道长度、起始偏移、目标偏移、目标中心偏移、车道宽度、自车速度、前车距离和前车TTC，输出一个包含轨迹和相关信息的AvoidancePathCandidate对象
    trajectory,
    length,
    start_offset,
    target_offset,
    target_center_offset,
    lane_width,
    ego_speed,
    front_distance,
    front_ttc,
    front_target_speed,
):
    lateral_shift = target_offset - start_offset # 计算侧向位移，表示车辆在换道过程中的横向移动距离，即目标偏移与起始偏移之间的差值
    maneuver_time = length / ego_speed # 估算换道所需时间，基于换道长度和自车速度计算得到，表示完成换道所需的时间
    lateral_accel = 10.0 * math.sqrt(3.0) * abs(lateral_shift) / (3.0 * max(maneuver_time * maneuver_time, 0.01))
    max_lateral_accel = 3.8
    reject_reason = ""

    if abs(target_offset - target_center_offset) > lane_width * 0.35:
        reject_reason = "target outside lane bound"
    elif length < 14.0:
        reject_reason = "length too short"
    elif lateral_accel > max_lateral_accel:
        reject_reason = "lateral acceleration too high"
    predicted_front_motion = front_target_speed * maneuver_time
    front_clear_distance = front_distance + predicted_front_motion

    if reject_reason == "" and math.isfinite(front_distance) and length > front_clear_distance + 6.0:
        reject_reason = "path too long before front vehicle"

    center_error = abs(target_offset - target_center_offset) / lane_width
    safety_cost = _safety_cost(length, maneuver_time, front_clear_distance, front_ttc)
    comfort_cost = (lateral_accel / max_lateral_accel) ** 2 + 0.20 * abs(lateral_shift) / lane_width
    tracking_cost = _tracking_cost(trajectory, length)
    total_cost = 4.0 * safety_cost + 2.0 * comfort_cost + tracking_cost + 0.6 * center_error

    return AvoidancePathCandidate(
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
        progress0, _ = trajectory.to_local(transform.location)
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

                    ref_location = trajectory.location_at(progress)
                    ref_yaw = trajectory.reference_yaw_at(progress)
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


