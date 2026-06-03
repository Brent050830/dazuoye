import math
import random
import time

import carla

from actors import ( # 场景中涉及的各种演员生成函数，包括自车、前车、背景车辆、背景自行车和右侧过街自行车
    spawn_background_r344_bicycles,
    spawn_background_route_vehicles,
    spawn_right_side_bicycle_crossing,
    spawn_scenario,
)

from config import ( # 仿真参数配置，包括服务器连接、仿真时间、车辆目标速度、换道长度、安全距离、碰撞和避让的 TTC 阈值等
    CLIENT_TIMEOUT,
    EGO_TARGET_SPEED,
    FIXED_DELTA_SECONDS,
    HOST,
    LANE_CHANGE_LENGTH,
    LEAD_BRAKE_TIME,
    LEAD_TARGET_SPEED,
    MAP_NAME,
    PORT,
    RIGHT_OBJECT_DETECT_DISTANCE,
    RIGHT_OBJECT_STOP_DISTANCE,
    RIGHT_OBJECT_TTC_THRESHOLD,
    RIGHT_OBJECT_YIELD_SPEED,
    ROUTE_COMPLETION_HOLD_SECONDS,
    SAFE_DISTANCE,
    SIM_SECONDS,
    TRAFFIC_RANDOM_SEED,
    TTC_AVOID_THRESHOLD,
    TTC_BRAKE_THRESHOLD,
)
from control import QuinticLaneChangeTrajectory, SamplingMPCTracker # 换道轨迹规划和 MPC 控制器实现
from display import CollisionMonitor, PygameDemoDisplay # 碰撞监测和仿真显示实现
from perception import VirtualGroundTruthSensor # 虚拟传感器实现，提供前车和右侧过街物体的距离、TTC 等信息
from route import LoopRoute # 固定路线实现，提供路线点、航向和转弯信息
from utils import get_speed, speed_control, waypoint_steer # 工具函数，包括获取速度、速度控制和航向控制等

# ===================== 仿真世界初始化 =====================

def setup_world(client):
    """加载指定地图并启用同步模式，返回配置好的世界对象"""
    world = client.get_world()
    current_map = world.get_map().name
    if MAP_NAME not in current_map:
        print("正在加载地图 {}...原地图: {}".format(MAP_NAME, current_map))
        for attempt in range(3):
            try:
                world = client.load_world(MAP_NAME)
                break
            except RuntimeError as exc:
                if attempt == 2:
                    raise
                print("地图加载失败（第{}次）: {}，正在重试...".format(attempt + 1, exc))
                time.sleep(2.0)
    else:
        print("使用已加载地图 {}。".format(current_map))

    # 启用同步模式并设置固定步长
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)
    return world


def restore_world(world, original_settings):
    """仿真结束后恢复世界为异步模式，不影响其他程序使用CARLA"""
    world.apply_settings(original_settings)


def set_spectator(world, ego_vehicle):
    """将观察者视角定位到自车正上方，仿真中提供上帝视角"""
    ego_tf = ego_vehicle.get_transform()
    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            ego_tf.location + carla.Location(z=45.0),
            carla.Rotation(pitch=-75.0, yaw=ego_tf.rotation.yaw),
        )
    )


def choose_avoidance_side(sensor):
    """根据邻道净空状况选择避障换道方向：优先左转，其次右转，无道则返回 None"""
    if sensor.lane_clear("left"): # 调用传感器的 lane_clear 方法检查左侧邻道是否在前后安全范围内无车，如果左侧邻道清空，则选择左换道避障
        return "left"
    if sensor.lane_clear("right"):
        return "right"
    return None


def main():
    """主函数：设置仿真环境，生成演员，执行主循环进行控制，并在结束后清理资源"""
    actor_list = [] # 用于跟踪所有生成的演员，以便在仿真结束后统一销毁
    camera_display = None # 用于显示仿真画面，如果创建失败则保持为 None
    world = None # 仿真世界对象，初始化为 None，在 setup_world 成功后赋值，最后在 finally 块中恢复设置
    original_settings = None # 用于保存仿真世界的原始设置，以便在仿真结束后恢复，避免影响其他程序使用 CARLA

    try:
        client = carla.Client(HOST, PORT)
        client.set_timeout(CLIENT_TIMEOUT)

        world = client.get_world() # 获取当前世界对象
        original_settings = world.get_settings()
        world = setup_world(client) # 加载指定地图并启用同步模式，返回配置好的世界对象
        carla_map = world.get_map()

        ego_vehicle, lead_vehicle, ego_start_wp = spawn_scenario(world) # 生成自车和前车，并返回自车的起始路点
        actor_list.extend([ego_vehicle, lead_vehicle]) # 将生成的自车和前车添加到演员列表中，以便后续管理和清理
        collision_monitor = CollisionMonitor(world, ego_vehicle, actor_list)
        mpc = SamplingMPCTracker() # 创建 MPC 控制器实例，用于后续的轨迹跟踪控制
        camera_display = PygameDemoDisplay(world, ego_vehicle, actor_list) # 创建仿真显示实例，提供实时画面显示和信息渲染，如果创建失败则保持为 None

        world.tick() # 仿真世界进行一次更新，确保所有演员都已生成并准备就绪
        set_spectator(world, ego_vehicle) # 将观察者视角定位到自车正上方，提供上帝视角观察仿真过程
        loop_route = LoopRoute(ego_start_wp)
        traffic_rng = random.Random(TRAFFIC_RANDOM_SEED)
        background_vehicles = spawn_background_route_vehicles(world, loop_route, actor_list, traffic_rng)
        background_bicycles = spawn_background_r344_bicycles(world, loop_route, actor_list, traffic_rng)
        right_object_scenario = spawn_right_side_bicycle_crossing(world, loop_route, actor_list)
        right_object_scenarios = [scenario for scenario in [right_object_scenario] + background_bicycles if scenario]
        sensor = VirtualGroundTruthSensor(
            world,
            carla_map,
            ego_vehicle,
            lead_vehicle,
            front_extra_vehicles=[controller.actor for controller in background_vehicles],
            right_object_scenarios=right_object_scenarios,
        )

        state = "FOLLOW"
        trajectory = None
        start_time = time.time()
        frame = 0
        route_completion_time = None
        right_object_yield_done = False

        print("Scenario started: map={}, ego=Tesla Model3, lead=Lincoln MKZ 2020".format(MAP_NAME))
        print("Lead car will brake hard at {:.1f}s.".format(LEAD_BRAKE_TIME))
        print(
            "Background traffic: route_vehicles={}, r344_bicycles={}.".format(
                len(background_vehicles),
                len(background_bicycles),
            )
        )
        print(
            "Loop route: {:.1f}m, {} waypoints.".format(
                loop_route.length, len(loop_route.points)
            )
        )
        print(
            "Route turn check: right_turn_count={}, turn_events={}".format(
                loop_route.right_turn_count,
                [
                    "{}:{:.1f}deg@{}-{}".format(
                        event["direction"],
                        event["degrees"],
                        event["start_index"],
                        event["end_index"],
                    )
                    for event in loop_route.turn_events
                ],
            )
        )
        print(
            "Route lane check: right_lane_before_turn={}, prepare_index={}, close_to_index={}.".format(
                loop_route.right_lane_before_turn,
                loop_route.right_lane_prepare_index,
                loop_route.close_to_index,
            )
        )

        while frame * FIXED_DELTA_SECONDS < SIM_SECONDS: # 主循环，持续运行直到达到最大仿真时间
            if camera_display is not None and not camera_display.process_events():
                print("Animation window closed by user.")
                break

            sim_time = frame * FIXED_DELTA_SECONDS

            if sim_time < LEAD_BRAKE_TIME:
                lead_throttle, lead_brake = speed_control(get_speed(lead_vehicle), LEAD_TARGET_SPEED)
                lead_steer = waypoint_steer(lead_vehicle, carla_map)
                lead_vehicle.apply_control(
                    carla.VehicleControl(throttle=lead_throttle, brake=lead_brake, steer=lead_steer)
                )
            else:
                lead_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))

            for background_vehicle in background_vehicles:
                background_vehicle.update(FIXED_DELTA_SECONDS)

            for right_object in right_object_scenarios:
                right_object.update(loop_route.last_index, FIXED_DELTA_SECONDS)

            front = sensor.front_vehicle()
            right_object = sensor.right_side_object(loop_route.last_index)
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
            right_object_risk = (
                right_object.is_conflict_object
                and (
                    right_object.ttc < RIGHT_OBJECT_TTC_THRESHOLD
                    or right_object.distance < RIGHT_OBJECT_DETECT_DISTANCE
                )
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

            if state in ("FOLLOW", "LANE_KEEP") and right_object_risk and not right_object_yield_done:
                state = "RIGHT_OBJECT_YIELD"
                print(
                    "Right object yield started at {:.2f}s: distance={:.1f}m, TTC={:.2f}s, route_index={}.".format(
                        sim_time,
                        right_object.distance,
                        right_object.ttc if math.isfinite(right_object.ttc) else 99.99,
                        loop_route.last_index,
                    )
                )

            if route_completion_time is not None:
                state = "ROUTE_HOLD"
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            elif state == "AVOID" and trajectory is not None:
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

            elif state == "RIGHT_OBJECT_YIELD":
                target_speed = RIGHT_OBJECT_YIELD_SPEED
                throttle, brake = speed_control(ego_speed, target_speed)
                if right_object.distance < RIGHT_OBJECT_STOP_DISTANCE:
                    throttle = 0.0
                    brake = max(brake, 0.85)
                ego_control = carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=loop_route.steer(ego_vehicle),
                )
                if not right_object_risk:
                    state = "LANE_KEEP"
                    right_object_yield_done = True
                    print("Right object yield completed at {:.2f}s.".format(sim_time))

            else:
                if brake_needed:
                    target_speed = min(EGO_TARGET_SPEED, max(0.0, ego_speed - 5.0))
                else:
                    target_speed = EGO_TARGET_SPEED
                throttle, brake = speed_control(ego_speed, target_speed)
                ego_control = carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=loop_route.steer(ego_vehicle),
                )

            ego_vehicle.apply_control(ego_control)

            if frame % int(1.0 / FIXED_DELTA_SECONDS) == 0:
                print(
                    "t={:05.2f}s state={:<18} dist={:05.1f}m ttc={:05.2f}s "
                    "right={:05.1f}m r_ttc={:05.2f}s ego={:04.1f}m/s lead={:04.1f}m/s steer={:+.2f} brake={:.2f}".format(
                        sim_time,
                        state,
                        front.distance,
                        front.ttc if math.isfinite(front.ttc) else 99.99,
                        right_object.distance if math.isfinite(right_object.distance) else 99.9,
                        right_object.ttc if math.isfinite(right_object.ttc) else 99.99,
                        ego_speed,
                        get_speed(lead_vehicle),
                        ego_control.steer,
                        ego_control.brake,
                    )
                )

            world.tick()
            set_spectator(world, ego_vehicle)
            lap_completed = loop_route.update(ego_vehicle)
            if camera_display is not None:
                camera_display.render({
                    "sim_time": sim_time,
                    "state": state,
                    "scenario": "front_brake_and_right_object",
                    "ego_speed": ego_speed,
                    "lead_speed": get_speed(lead_vehicle),
                    "front_distance": front.distance,
                    "front_ttc": front.ttc,
                    "right_object_distance": right_object.distance,
                    "right_object_ttc": right_object.ttc,
                    "steer": ego_control.steer,
                    "throttle": ego_control.throttle,
                    "brake": ego_control.brake,
                    "collision_count": len(collision_monitor.history),
                    "lap_distance": loop_route.progress_distance,
                    "lap_target_distance": loop_route.length,
                })
            frame += 1

            # 碰撞发生后立即提前终止仿真
            if collision_monitor.history:
                print("检测到碰撞，提前终止仿真。")
                break

            if lap_completed and route_completion_time is None:
                route_completion_time = sim_time
                print(
                    "完成 Town10 固定路线一圈，行驶距离 {:.1f}m，继续运行 {:.1f}s 后结束。".format(
                        loop_route.progress_distance,
                        ROUTE_COMPLETION_HOLD_SECONDS,
                    )
                )

            if (
                route_completion_time is not None
                and sim_time - route_completion_time >= ROUTE_COMPLETION_HOLD_SECONDS
            ):
                print("到达路线终点后已继续运行 {:.1f}s，结束仿真。".format(ROUTE_COMPLETION_HOLD_SECONDS))
                break

        elapsed = time.time() - start_time
        print(
            "Scenario finished in {:.1f}s wall time. Collisions: {}".format(
                elapsed, len(collision_monitor.history)
            )
        )

    finally:
        if world is not None and original_settings is not None:
            restore_world(world, original_settings)
        for actor in reversed(actor_list):
            if actor is not None:
                try:
                    actor.destroy()
                except RuntimeError as exc:
                    print("Cleanup warning: failed to destroy actor {}: {}".format(actor.id, exc))
        if camera_display is not None:
            try:
                camera_display.close()
            except RuntimeError as exc:
                print("Cleanup warning: failed to close pygame display: {}".format(exc))
        print("Cleanup finished.")


if __name__ == "__main__":
    main()
