# CARLA 前车急停与后车紧急避障程序框架

> 维护约定：后续如果需要更新程序说明、框架说明、参数说明或维护记录，只更新本文档，不再新建新的说明文件。

## 1. 程序目标

本程序基于 CARLA 搭建一个自动驾驶紧急避障演示场景：

- 前车正常行驶一段时间后突然急停。
- 后车作为自车，先正常跟车。
- 当检测到前车急停导致 TTC 过低时，自车进行纵向制动。
- 如果相邻车道安全，自车生成五次多项式换道轨迹，并通过 MPC 跟踪轨迹完成横向避障。
- 使用 pygame 打开摄像头演示窗口，实时显示后车视角和关键状态信息。

主程序文件：

```text
dazuoye/guiji.py
```

## 2. 运行方式

先启动 CARLA 服务端，等待 `CarlaUE4.exe` 的仿真窗口完全打开。

然后在 PowerShell 中运行：

```powershell
& E:/Anaconda_envs/envs/carla_env/python.exe d:/17871/CARLA_0.9.15/WindowsNoEditor/PythonAPI/examples/dazuoye/guiji.py
```

如果 pygame 窗口打开后需要退出，可以按：

```text
Esc 或 Q
```

也可以直接关闭 pygame 窗口。

## 3. 整体框架

程序可以分为 7 个部分：

```text
参数配置
  ↓
工具函数
  ↓
场景与车辆生成
  ↓
虚拟传感器感知
  ↓
决策状态机
  ↓
五次多项式轨迹生成 + MPC 跟踪控制
  ↓
pygame 可视化 + 资源清理
```

## 4. 参数配置区

位置：`guiji.py` 文件开头。

主要参数：

```python
HOST = "localhost"
PORT = 2000
MAP_NAME = "Town04"
CLIENT_TIMEOUT = 120.0
FIXED_DELTA_SECONDS = 0.05
SIM_SECONDS = 28.0
```

含义：

- `HOST` / `PORT`：CARLA 服务端连接地址。
- `MAP_NAME`：使用地图，当前为 `Town04`。
- `CLIENT_TIMEOUT`：客户端等待 CARLA 响应的超时时间。
- `FIXED_DELTA_SECONDS`：同步仿真步长，当前为 0.05 秒，即 20Hz。
- `SIM_SECONDS`：单次演示最长运行时间。

车辆和场景参数：

```python
INITIAL_GAP = 48.0
LEAD_BRAKE_TIME = 6.0
EGO_TARGET_SPEED = 15.5
LEAD_TARGET_SPEED = 13.0
```

含义：

- `INITIAL_GAP`：后车与前车初始距离。
- `LEAD_BRAKE_TIME`：前车开始急刹的时间。
- `EGO_TARGET_SPEED`：后车目标速度。
- `LEAD_TARGET_SPEED`：前车急刹前的目标速度。

安全决策参数：

```python
TTC_BRAKE_THRESHOLD = 4.5
TTC_AVOID_THRESHOLD = 3.6
SAFE_DISTANCE = 34.0
LANE_CLEAR_FRONT = 45.0
LANE_CLEAR_REAR = 18.0
```

含义：

- `TTC_BRAKE_THRESHOLD`：低于该 TTC 时开始辅助制动。
- `TTC_AVOID_THRESHOLD`：低于该 TTC 时触发紧急换道避障。
- `SAFE_DISTANCE`：换道触发时的前车距离条件。
- `LANE_CLEAR_FRONT` / `LANE_CLEAR_REAR`：判断相邻车道是否安全的前后检测范围。

轨迹与 MPC 参数：

```python
LANE_CHANGE_LENGTH = 28.0
MPC_HORIZON_STEPS = 18
MPC_DT = 0.10
WHEEL_BASE = 2.85
```

含义：

- `LANE_CHANGE_LENGTH`：五次多项式换道轨迹纵向长度。
- `MPC_HORIZON_STEPS`：MPC 预测步数。
- `MPC_DT`：MPC 每一步的时间间隔。
- `WHEEL_BASE`：车辆轴距，用于运动学自行车模型。

## 5. 工具函数

工具函数负责基础数学计算和简单车辆控制。

主要函数：

```python
clamp(value, low, high)
vector_length(vector)
dot_2d(a, b)
normalize_angle(angle)
yaw_to_rad(rotation)
get_speed(vehicle)
```

作用：

- 限幅。
- 计算向量长度。
- 计算二维点积。
- 角度归一化。
- 将 CARLA yaw 转成弧度。
- 计算车辆速度。

基础控制函数：

```python
speed_control(current_speed, target_speed)
waypoint_steer(vehicle, carla_map, lookahead=12.0)
```

作用：

- `speed_control`：根据目标速度输出油门和制动。
- `waypoint_steer`：根据 CARLA 路点进行简单车道保持转向。

## 6. 场景生成模块

### 6.1 地图设置

函数：

```python
setup_world(client)
restore_world(world, original_settings)
```

作用：

- 加载或复用目标地图。
- 设置同步模式。
- 设置固定仿真步长。
- 程序结束时恢复原始 world settings。

### 6.2 固定起点选择

函数：

```python
find_fixed_scenario_waypoint(carla_map)
```

作用：

- 从地图 spawn points 中筛选适合演示的起点。
- 要求路段不是路口。
- 要求前方较长距离接近直线。
- 要求至少存在一个同向相邻车道。
- 最后选择一个确定性的候选点，保证每次演示尽量一致。

### 6.3 车辆生成

函数：

```python
spawn_scenario(world)
```

当前车型：

```text
后车/自车：vehicle.tesla.model3
前车：vehicle.lincoln.mkz_2020
```

生成方式：

- 后车生成在固定起点。
- 前车生成在后车前方 `INITIAL_GAP` 米处。
- 两车初始处于同一车道。

## 7. 虚拟传感器模块

类：

```python
VirtualGroundTruthSensor
```

当前没有使用真实 Radar/Lidar，而是使用 CARLA ground truth 作为虚拟传感器。

### 7.1 前车检测

方法：

```python
front_vehicle()
```

输出：

```python
FrontVehicleReading(
    distance,
    closing_speed,
    ttc,
    lateral_offset,
    is_front_vehicle
)
```

含义：

- `distance`：前车相对自车的纵向距离。
- `closing_speed`：自车相对前车的接近速度。
- `ttc`：Time To Collision，预计碰撞时间。
- `lateral_offset`：前车相对自车的横向偏移。
- `is_front_vehicle`：是否判定为本车道前方车辆。

TTC 计算逻辑：

```text
ttc = distance / closing_speed
```

当接近速度很小或前车不在前方时，TTC 记为无穷大。

### 7.2 相邻车道安全判断

方法：

```python
lane_clear(side)
```

作用：

- 判断左侧或右侧相邻车道是否存在。
- 判断相邻车道是否与当前车道同向。
- 检查目标车道前后一定范围内是否有其他车辆。

## 8. 轨迹生成模块

类：

```python
QuinticLaneChangeTrajectory
```

轨迹形式：

```text
d(s) = D * (10t^3 - 15t^4 + 6t^5)
t = s / L
```

其中：

- `s`：沿车道方向的纵向进度。
- `d`：相对起始车道的横向偏移。
- `D`：目标横向偏移，通常约等于一个车道宽。
- `L`：换道轨迹纵向长度。

特点：

- 起点横向位移为 0。
- 终点横向位移为目标车道偏移量。
- 起点和终点横向速度为 0。
- 起点和终点横向加速度为 0。
- 适合做平滑换道轨迹。

主要方法：

```python
to_local(location)
lateral_at(s)
lateral_slope_at(s)
```

作用：

- `to_local`：把 CARLA 世界坐标转换为轨迹局部坐标。
- `lateral_at`：计算某个纵向位置对应的横向参考位置。
- `lateral_slope_at`：计算轨迹斜率，用于参考航向角。

## 9. MPC 跟踪模块

类：

```python
SamplingMPCTracker
```

当前实现是轻量采样式 MPC，不依赖外部优化器。

### 9.1 车辆模型

使用运动学自行车模型近似车辆运动：

```text
s_next = s + v * cos(yaw) * dt
d_next = d + v * sin(yaw) * dt
yaw_next = yaw + v / Lw * tan(steer) * dt
v_next = v + accel * dt
```

其中：

- `s`：轨迹纵向坐标。
- `d`：轨迹横向坐标。
- `yaw`：车辆相对轨迹起始方向的航向角。
- `v`：车速。
- `steer`：方向盘控制量。
- `accel`：加速度候选值。
- `Lw`：车辆轴距。

### 9.2 控制量搜索

MPC 在多个候选控制量中搜索：

```text
steer_candidates
accel_candidates
```

对每一组候选控制量，在预测时域内模拟车辆运动，并计算总代价。

### 9.3 代价函数

主要考虑：

- 横向误差。
- 航向角误差。
- 速度误差。
- 转向幅度。
- 加速度幅度。
- 转向变化平滑性。

总代价越小，说明该控制量越适合当前轨迹跟踪。

### 9.4 输出

方法：

```python
control(ego_vehicle, trajectory, target_speed)
```

输出：

```python
carla.VehicleControl(
    throttle,
    brake,
    steer
)
```

## 10. 决策状态机

主状态机位于：

```python
main()
```

当前状态：

```text
FOLLOW
AVOID
LANE_KEEP
EMERGENCY_BRAKE
```

### 10.1 FOLLOW

正常跟车状态。

逻辑：

- 前车未急停或 TTC 较大时，后车保持目标速度并沿当前车道行驶。
- 如果 TTC 低于制动阈值，开始降低速度。
- 如果 TTC 低于避障阈值且前车距离小于安全距离，则尝试换道避障。

### 10.2 AVOID

紧急避障状态。

进入条件：

```text
front.is_front_vehicle == True
front.distance < SAFE_DISTANCE
front.ttc < TTC_AVOID_THRESHOLD
相邻车道安全
```

动作：

- 选择左侧或右侧安全车道。
- 生成五次多项式换道轨迹。
- 使用 MPC 跟踪轨迹。
- 同时保留一定纵向制动，降低追尾风险。

### 10.3 LANE_KEEP

避障完成后的车道保持状态。

进入条件：

- 自车沿轨迹前进超过换道长度。
- 横向位置接近目标车道。

动作：

- 重新使用路点车道保持控制。
- 继续向前行驶。

### 10.4 EMERGENCY_BRAKE

纯紧急制动状态。

进入条件：

- TTC 过低。
- 相邻车道不可用。

动作：

```python
throttle = 0.0
brake = 1.0
steer = 0.0
```

## 11. pygame 可视化模块

类：

```python
PygameCameraDisplay
```

作用：

- 给后车挂载一个 RGB 摄像头。
- 将 CARLA camera 的 BGRA 图像转为 RGB。
- 使用 `pygame.surfarray.make_surface` 显示图像。
- 在窗口上方叠加演示状态。

显示内容：

```text
仿真时间
当前状态
前车距离
TTC
自车速度
前车速度
```

退出方式：

```text
Esc
Q
关闭窗口
```

如果环境中缺少 `pygame` 或 `numpy`，程序会自动禁用动画窗口，但仍可以继续运行 CARLA 控制逻辑。

## 12. 碰撞监测模块

类：

```python
CollisionMonitor
```

作用：

- 给后车挂载 CARLA collision sensor。
- 如果发生碰撞，记录到 `history`。
- 程序结束时打印碰撞次数。

成功演示的目标结果：

```text
Collisions: 0
```

## 13. 主程序执行流程

主入口：

```python
if __name__ == "__main__":
    main()
```

`main()` 的核心流程：

```text
1. 创建 CARLA client
2. 设置 timeout
3. 保存原始 world settings
4. 加载或复用目标地图
5. 设置同步模式和固定步长
6. 生成后车和前车
7. 挂载 collision sensor
8. 创建虚拟传感器
9. 创建 MPC 控制器
10. 创建 pygame 摄像头窗口
11. 进入仿真主循环
12. 前车在指定时间后急刹
13. 后车读取虚拟感知信息
14. 决策 FOLLOW / AVOID / LANE_KEEP / EMERGENCY_BRAKE
15. 输出车辆控制量
16. 刷新 CARLA 世界和 pygame 窗口
17. 仿真结束后恢复设置并销毁 actor
```

## 14. 关键运行现象

正常运行时控制台会输出类似：

```text
Scenario started: map=Town04, ego=Tesla Model3, lead=Lincoln MKZ 2020
Lead car will brake hard at 6.0s.
t=00.00s state=FOLLOW ...
Avoidance started at ... side=right ...
Avoidance completed at ...
Scenario finished ... Collisions: 0
Cleanup finished.
```

如果看到：

```text
Loading map Town04 ...
```

说明 CARLA 正在切换地图，可能需要等待几十秒到一两分钟。

## 15. 常见问题

### 15.1 程序卡在加载地图

原因：

- CARLA 正在从当前地图切换到 `MAP_NAME`。
- 地图第一次加载可能较慢。

处理：

- 等待 1 到 2 分钟。
- 不要在 `client.load_world(MAP_NAME)` 时按 `Ctrl+C`。
- 如果想使用当前地图，可以把 `MAP_NAME` 改成当前 CARLA 窗口中已加载的地图。

### 15.2 无法导入 carla

原因：

- 使用了错误的 Python 环境。

处理：

使用：

```powershell
E:/Anaconda_envs/envs/carla_env/python.exe
```

不要直接使用系统默认 Python 3.13。

### 15.3 pygame 图像格式错误

现已处理。

程序使用 numpy 将 CARLA camera 的 BGRA 数据转换为 RGB，再交给 pygame 显示。

### 15.4 pygame 窗口没有打开

可能原因：

- 没有安装 pygame。
- 没有安装 numpy。
- CARLA 没有连接成功，程序还没有进入显示阶段。

## 16. 后续可扩展方向

可以在现有框架上继续扩展：

- 将虚拟传感器替换为真实 Radar。
- 将虚拟传感器替换为 Lidar 点云聚类。
- 加入前方多车场景。
- 加入旁车，测试无法换道时的纯制动逻辑。
- 将采样式 MPC 替换为带约束优化器的 MPC。
- 记录轨迹数据并导出 CSV。
- 使用 matplotlib 绘制距离、TTC、速度、转向角随时间变化曲线。

## 17. 维护记录

### 2026-05-30

- 建立本文档。
- 记录 `guiji.py` 的程序框架、模块职责、主流程和常见问题。
- 明确后续文档维护只更新本文档。
