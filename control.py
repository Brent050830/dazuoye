import math

import carla

from config import MPC_DT, MPC_HORIZON_STEPS, WHEEL_BASE # MPC控制器的时间步长、预测时域步数和车辆轴距
from utils import clamp, dot_2d, get_speed, normalize_angle, yaw_to_rad # 一些数学工具函数：clamp用于限制数值范围，dot_2d计算二维向量点积，get_speed获取车辆速度，normalize_angle将角度归一化到[-pi, pi]，yaw_to_rad将carla的旋转转换为弧度表示的航向角


# ===================== 换道轨迹规划 =====================

class QuinticLaneChangeTrajectory:
    """五次多项式换道轨迹：d(s) = D * (10t³ - 15t⁴ + 6t⁵)，t=s/L。
    保证起止点的位移、速度、加速度均为零，轨迹平滑。
    """

    def __init__(self, start_transform, lateral_offset, length):
        """初始化换道轨迹
        参数：
            start_transform: 换道起点的车辆坐标变换
            lateral_offset:  目标侧向偏移量（负值为左换道），现在为车道宽度的正负值
            length:          换道纵向总长度（米）,现在是个定值
        """
        self.origin = start_transform.location # 换道起点的全局坐标
        self.start_yaw = yaw_to_rad(start_transform.rotation) # 换道起点的航向角（弧度）
        self.forward = start_transform.get_forward_vector() # 换道起点的前向单位向量
        self.right = start_transform.get_right_vector() # 换道起点的右向单位向量
        self.lateral_offset = lateral_offset # 目标侧向偏移量
        self.length = length # 换道纵向总长度

    def to_local(self, location):
        """将全局坐标转换为以起点为原点的局部纵横坐标 (s, d)"""
        relative = location - self.origin # 计算相对于起点的坐标差向量
        return dot_2d(relative, self.forward), dot_2d(relative, self.right) # 通过点积计算纵向位置 s 和横向位置 d

    def lateral_at(self, s):
        """计算纵向位置 s 处的目标横向偏移量"""
        if s <= 0.0: # 如果 s 小于等于0，说明在起点之前，横向偏移为0
            return 0.0
        if s >= self.length: # 如果 s 大于等于换道长度，说明在终点之后，横向偏移为目标值
            return self.lateral_offset
        tau = s / self.length # 归一化的纵向位置，范围 [0, 1]
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5 # 五次多项式的值，表示横向偏移随纵向位置的变化比例
        return self.lateral_offset * blend

    def lateral_slope_at(self, s):
        """计算纵向位置 s 处的轨迹横向斜率（用于计算参考航向角）"""
        if s <= 0.0 or s >= self.length:
            return 0.0
        tau = s / self.length # 归一化的纵向位置，范围 [0, 1]
        blend_dot = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4 # 五次多项式的导数，表示横向偏移随纵向位置的变化率
        return self.lateral_offset * blend_dot / self.length


# ===================== MPC 轨迹跟踪控制器 =====================

class RouteOffsetLaneChangeTrajectory:
    """基于全局路径的换道轨迹：在给定的全局路径上生成一个带有侧向偏移的轨迹。
通过在全局路径上进行插值，计算出每个纵向位置 s 对应的全局坐标，并在此基础上添加侧向偏移，形成换道轨迹。
    """

    def __init__(self, loop_route, start_index, start_transform, lateral_offset, length, start_offset=0.0):
        """初始化换道轨迹"""
        self.loop_route = loop_route
        self.start_index = max(0, min(start_index, len(loop_route.points) - 1))
        self.start_offset = start_offset
        self.lateral_offset = lateral_offset
        self.length = length
        self.step_distance = loop_route.step_distance
        self.right = start_transform.get_right_vector()
        self.start_yaw = yaw_to_rad(start_transform.rotation)
        self.is_route_relative = True

    def _clamp_index(self, index):
        """将索引限制在全局路径点的范围内"""
        return max(0, min(index, len(self.loop_route.points) - 1))

    def _route_pose_at(self, s):
        """计算全局路径上纵向位置 s 处的坐标和右向单位向量"""
        raw_index = self.start_index + s / self.step_distance
        lower = self._clamp_index(int(math.floor(raw_index)))
        upper = self._clamp_index(lower + 1)
        blend = clamp(raw_index - lower, 0.0, 1.0)
        p0 = self.loop_route.points[lower]
        p1 = self.loop_route.points[upper]
        location = carla.Location(
            x=p0.x + (p1.x - p0.x) * blend,
            y=p0.y + (p1.y - p0.y) * blend,
            z=p0.z + (p1.z - p0.z) * blend,
        )
        right0 = self.loop_route.waypoints[lower].transform.get_right_vector()
        right1 = self.loop_route.waypoints[upper].transform.get_right_vector()
        right = carla.Vector3D(
            x=right0.x + (right1.x - right0.x) * blend,
            y=right0.y + (right1.y - right0.y) * blend,
            z=0.0,
        )
        right_length = max(math.sqrt(right.x * right.x + right.y * right.y), 0.001)
        right.x /= right_length
        right.y /= right_length
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
        search_steps = int((self.length + 24.0) / self.step_distance) + 8
        start = self._clamp_index(self.start_index - 3)
        end = self._clamp_index(self.start_index + search_steps)
        nearest = min(
            range(start, end + 1),
            key=lambda index: self.loop_route.points[index].distance(location),
        )
        progress = max(0.0, (nearest - self.start_index) * self.step_distance)
        route_location = self.loop_route.points[nearest]
        route_right = self.loop_route.waypoints[nearest].transform.get_right_vector()
        lateral = dot_2d(location - route_location, route_right) # 通过计算当前位置与全局路径上最近点的坐标差向量与路径右向单位向量的点积，得到当前位置相对于全局路径的横向偏移量 d
        return progress, lateral


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
        transform = ego_vehicle.get_transform() # 获取自车的当前变换信息
        s0, d0 = trajectory.to_local(transform.location)  # 当前局部纵横坐标
        yaw0 = normalize_angle(yaw_to_rad(transform.rotation) - trajectory.start_yaw)  # 相对航向角
        v0 = get_speed(ego_vehicle)  # 当前速度

        # 候选转向角集合（共9个离散值）
        steer_candidates = [
            clamp(self.previous_steer + delta, -0.65, 0.65)
            for delta in (-0.45, -0.32, -0.20, -0.10, 0.0, 0.10, 0.20, 0.32, 0.45)
        ]
        # 候选加速度集合（共5个离散值，单位 m/s²）
        accel_candidates = (-4.0, -2.0, -1.0, 0.0, 1.0)

        if getattr(trajectory, "is_route_relative", False):
            return self._control_route_relative(ego_vehicle, trajectory, target_speed)

        best_cost = float("inf")
        best_action = (0.0, -3.0)  # 默认保守制动动作

        for steer in steer_candidates:
            """对于每个候选转向角，遍历所有候选加速度，进行前向积分预测，并计算代价函数，选取代价最小的动作"""
            for accel in accel_candidates:
                """初始化预测状态为当前状态"""
                s = s0
                d = d0
                yaw = yaw0
                speed = v0
                cost = 0.0

                # 沿预测时域逐步积分代价
                for step in range(MPC_HORIZON_STEPS):
                    speed = max(0.0, speed + accel * MPC_DT)
                    s += speed * math.cos(yaw) * MPC_DT
                    d += speed * math.sin(yaw) * MPC_DT
                    yaw = normalize_angle(yaw + speed / WHEEL_BASE * math.tan(steer) * MPC_DT)

                    ref_d = trajectory.lateral_at(s)          # 参考横向偏移
                    ref_yaw = math.atan(trajectory.lateral_slope_at(s))  # 参考航向角
                    lateral_error = d - ref_d                  # 横向跟踪误差
                    yaw_error = normalize_angle(yaw - ref_yaw) # 航向误差
                    speed_error = speed - target_speed         # 速度误差

                    cost += 6.0 * lateral_error**2    # 横向误差惩罚
                    cost += 1.7 * yaw_error**2        # 航向误差惩罚
                    cost += 0.07 * speed_error**2     # 速度误差惩罚
                    cost += 0.08 * steer**2           # 转向幅度惩罚
                    cost += 0.01 * accel**2           # 加速度幅度惩罚
                    cost += 0.02 * step * abs(steer - self.previous_steer)  # 转向连续性惩罚

                if cost < best_cost:
                    best_cost = cost
                    best_action = (steer, accel)

        steer, accel = best_action
        self.previous_steer = steer  # 保存本帧转向量供下帧使用

        # 将加速度映射为油门/制动量
        if accel >= 0.0:
            throttle = clamp(0.25 + 0.18 * accel, 0.0, 0.65)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-accel / 7.5, 0.0, 1.0)

        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)

    def _control_route_relative(self, ego_vehicle, trajectory, target_speed):
        """针对基于全局路径的换道轨迹的控制计算，输入自车对象、换道轨迹和目标速度，输出carla.VehicleControl对象"""
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
                    speed_error = speed - target_speed

                    cost += 6.0 * position_error**2
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


