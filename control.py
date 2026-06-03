import math

import carla

from config import MPC_DT, MPC_HORIZON_STEPS, WHEEL_BASE
from utils import clamp, dot_2d, get_speed, normalize_angle, yaw_to_rad


# ===================== 换道轨迹规划 =====================

class QuinticLaneChangeTrajectory:
    """五次多项式换道轨迹：d(s) = D * (10t³ - 15t⁴ + 6t⁵)，t=s/L。
    保证起止点的位移、速度、加速度均为零，轨迹平滑。
    """

    def __init__(self, start_transform, lateral_offset, length):
        """初始化换道轨迹
        参数：
            start_transform: 换道起点的车辆坐标变换
            lateral_offset:  目标侧向偏移量（负值为左换道）
            length:          换道纵向总长度（米）
        """
        self.origin = start_transform.location
        self.start_yaw = yaw_to_rad(start_transform.rotation)
        self.forward = start_transform.get_forward_vector()
        self.right = start_transform.get_right_vector()
        self.lateral_offset = lateral_offset
        self.length = length

    def to_local(self, location):
        """将全局坐标转换为以起点为原点的局部纵横坐标 (s, d)"""
        relative = location - self.origin
        return dot_2d(relative, self.forward), dot_2d(relative, self.right)

    def lateral_at(self, s):
        """计算纵向位置 s 处的目标横向偏移量"""
        if s <= 0.0:
            return 0.0
        if s >= self.length:
            return self.lateral_offset
        tau = s / self.length
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        return self.lateral_offset * blend

    def lateral_slope_at(self, s):
        """计算纵向位置 s 处的轨迹横向斜率（用于计算参考航向角）"""
        if s <= 0.0 or s >= self.length:
            return 0.0
        tau = s / self.length
        blend_dot = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
        return self.lateral_offset * blend_dot / self.length


# ===================== MPC 轨迹跟踪控制器 =====================

class SamplingMPCTracker:
    """基于采样的后退时域MPC控制器，使用运动学自行车模型进行滚动优化。
    通过枚举转向角和加速度候选值，选取预测代价最小的控制动作。
    """

    def __init__(self):
        self.previous_steer = 0.0  # 上一帧的转向量，用于连续性惩罚

    def control(self, ego_vehicle, trajectory, target_speed):
        """计算当前帧的最优控制指令
        返回：carla.VehicleControl（油门、制动、转向）
        """
        transform = ego_vehicle.get_transform()
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

        best_cost = float("inf")
        best_action = (0.0, -3.0)  # 默认保守制动动作

        for steer in steer_candidates:
            for accel in accel_candidates:
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


