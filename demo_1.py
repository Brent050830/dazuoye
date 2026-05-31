import random
import time

import carla
import cv2 # 导入OpenCV库，用于图像处理和显示
import numpy as np

actor_list = []

def img_process(data): # 处理传感器数据的回调函数，这里是将传感器捕获到的图像数据转换为numpy数组，并显示出来
    img = np.array(data.raw_data)
    img = img.reshape((1080, 1920, 4))
    cv2.imshow('', img) # 使用OpenCV的imshow函数显示图像，第一个参数是窗口名称，这里设置为空字符串，第二个参数是要显示的图像数据
    cv2.waitKey(1) # 使用OpenCV的waitKey函数等待键盘事件，这里设置为1毫秒，表示每隔1毫秒检查一次键盘事件，以便能够及时更新显示的图像

def callback(event): # 碰撞检测的回调函数，这里是当发生碰撞事件时，输出"碰撞"（因为输出是一个事件）
    print("碰撞") # （因为输出是一个事件）

def callback2(event): # 碰撞检测的回调函数，这里是当发生碰撞事件时，输出"穿越车道"（因为输出是一个事件）
    print("穿越车道") 

try:
    client = carla.Client('localhost', 2000) # 连接服务器
    client.set_timeout(30.0) # 连接服务器,设置超时时间
    max_attempts = 20
    for attempt in range(max_attempts):
        try:
            client.load_world('Town05') # 加载世界05（这里的地图加载一次就可以）
            world = client.get_world() # 获取当前世界
            if 'Town05' in world.get_map().name:
                break
        except RuntimeError:
            if attempt == max_attempts - 1:
                raise
            time.sleep(1.0)
    else:
        raise RuntimeError('Failed to load Town05 after retries')


    blueprint_library = world.get_blueprint_library() # 获取蓝图库
    v_bp = blueprint_library.filter('model3')[0] # 从蓝图库中筛选出特定的车辆模型,这里是特斯拉Model3

    spawn_point = random.choice(world.get_map().get_spawn_points()) # 从地图中随机选择一个生成点
    vehicle = world.spawn_actor(v_bp, spawn_point) # 在选定的生成点生成车辆
    actor_list.append(vehicle) # 将生成的车辆添加到actor_list中,以便后续管理

    '''1. 设置传感器：记录仪'''    
    blueprint = world.get_blueprint_library().find('sensor.camera.rgb') # 从蓝图库中找到RGB摄像头传感器的蓝图
    # 设置传感器的属性，包括图像分辨率（image_size_x和image_size_y）和视场角（fov）。这些属性决定了摄像头捕获图像的质量和范围。
    blueprint.set_attribute('image_size_x', '1920')
    blueprint.set_attribute('image_size_y', '1080')
    blueprint.set_attribute('fov', '110')

    blueprint.set_attribute('sensor_tick', '1.0') # 设置传感器捕获之间的时间间隔为1秒钟
    sensor_blueprint = blueprint # 将修改后的蓝图赋值给sensor_blueprint变量，以便后续使用
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    sensor = world.spawn_actor(sensor_blueprint, transform, attach_to=vehicle) # 在生成的车辆上安装一个RGB摄像头传感器，并设置其位置和旋转
    actor_list.append(sensor)
    # 保存数据
    sensor.listen(lambda data: img_process(data))   

    '''2. 设置碰撞检测：'''
    blueprint_collision = world.get_blueprint_library().find('sensor.other.collision') # 从蓝图库中找到碰撞传感器的蓝图
    # 但是没有其余可以配置的属性（只有输出的属性）
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    sensor_collision = world.spawn_actor(blueprint_collision, transform, attach_to=vehicle) # 换到自己的车上，传感器的一个定位，名称也变化了
    actor_list.append(sensor_collision)
    sensor_collision.listen(callback) # 设置碰撞检测的回调函数，当发生碰撞事件时，调用callback函数输出"碰撞"

    '''3. 设置车道偏离检测：'''
    blueprint_lane = world.get_blueprint_library().find('sensor.other.lane_invasion') # 从蓝图库中找到车道偏离传感器的蓝图
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    sensor_lane = world.spawn_actor(blueprint_lane, transform, attach_to=vehicle) # 在生成的车辆上安装一个车道偏离传感器，并设置其位置和旋转
    actor_list.append(sensor_lane)
    sensor_lane.listen(callback2) # 设置车道偏离检测的回调函数，当发生车道偏离事件时，调用callback2函数输出"穿越车道"

    '''4. 设置摄像机视角：'''
    spectator = world.get_spectator() # 获取观察者（摄像机）对象
    v_transform = vehicle.get_transform() # 获取生成车辆的变换信息（位置和旋转）
    camera_transform = carla.Transform(
        v_transform.location + carla.Location(z=30.0),
        carla.Rotation(pitch=-60.0, yaw=v_transform.rotation.yaw)
    )
    spectator.set_transform(camera_transform)

    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0)) # 对生成的车辆应用控制命令这里是设置油门为1,方向盘不转动
    time.sleep(10) # 让车辆保持行驶状态10秒钟
finally:
    for actor in actor_list:
        actor.destroy()
    print('结束')

# ceshi1