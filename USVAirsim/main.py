# PythonClient/car/hello_car.py
import airsim
import time

from udpCommunication import Udp

udp = Udp()

# 连接到Airsim仿真器
client = airsim.CarClient()

client.confirmConnection()  # 每1秒检查一次连接状态，并在控制台中报告，以便用户可以查看连接的进度（实际操作只显示一次，奇怪）。
# Connected!
# Client Ver:1 (Min Req: 1), Server Ver:1 (Min Req: 1)

client.enableApiControl(True)  # 默认是false，有的车辆不允许用API控制，所以用isApiControlEnabled可以查看是否可以用API控制
# 仿真器左上角会显示：ControlMode: API

while True:
    states = client.getCarState()
    car_state = states.__dict__['kinematics_estimated'].__dict__

    states = [car_state['position'].__dict__['x_val'], car_state['position'].__dict__['y_val'],
              car_state['position'].__dict__['z_val'],
              car_state['orientation'].__dict__['x_val'], car_state['orientation'].__dict__['y_val'],
              car_state['orientation'].__dict__['z_val'],
              car_state['linear_velocity'].__dict__['x_val'], car_state['linear_velocity'].__dict__['y_val'],
              car_state['linear_velocity'].__dict__['z_val'], car_state['angular_velocity'].__dict__['x_val'],
              car_state['angular_velocity'].__dict__['y_val'], car_state['angular_velocity'].__dict__['z_val'],
              car_state['linear_acceleration'].__dict__['x_val'], car_state['linear_acceleration'].__dict__['y_val'],
              car_state['linear_acceleration'].__dict__['z_val']]
    udp.states = states
    print(states)
    Udp.send(udp)
    Udp.receive(udp)
    car_controls = airsim.CarControls()  # 获得车辆控制数据
    car_controls.is_manual_gear = False
    car_controls.steering = udp.command[0] / 1000
    if udp.command[1] < 0:
        car_controls.is_manual_gear = True
        car_controls.manual_gear = -1
    car_controls.throttle = udp.command[1] / 1000
    client.setCarControls(car_controls)

# 将车辆初始化处理，后续如果需要重新控制则需要再次调用`enableApiControl`或`armDisarm

# client.reset()
# client.enableApiControl(False)  # 取消仿真器API控制

'''
    # get state of the car
    car_state = client.getCarState()  # 获取车辆状态，坐标统一是NED坐标，使用国际单位。
    # <CarState> {   'gear': 0,
    #    'handbrake': False,
    #    'kinematics_estimated': <KinematicsState> {   'angular_acceleration': <Vector3r> {   'x_val': 0.0,
    #    'y_val': 0.0,
    #    'z_val': 0.0},
    #    'angular_velocity': <Vector3r> {   'x_val': 0.0,
    #    'y_val': 0.0,
    #    'z_val': 0.0},
    #    'linear_acceleration': <Vector3r> {   'x_val': 0.0,
    #    'y_val': 0.0,
    #    'z_val': 0.0},
    #    'linear_velocity': <Vector3r> {   'x_val': 0.0,
    #    'y_val': 0.0,
    #    'z_val': -0.0},
    #    'orientation': <Quaternionr> {   'w_val': 1.0,
    #    'x_val': 3.1523319194093347e-05,
    #    'y_val': 0.0,
    #    'z_val': 0.0},
    #    'position': <Vector3r> {   'x_val': 0.0,
    #    'y_val': -2.2888182229507947e-06,
    #    'z_val': 0.23400458693504333}},
    #    'maxrpm': 7500.0,
    #    'rpm': 0.0,
    #    'speed': 0.0,
    #    'timestamp': 1597385459957857300}
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))'''

'''controls = airsim.CarControls()  # 获得车辆控制数据
# <CarControls> {   'brake': 0,
#    'gear_immediate': True,
#    'handbrake': False,
#    'is_manual_gear': False,
#    'manual_gear': 0,
#    'steering': 0,
#    'throttle': 0}
'''

'''    # 前进，配置car_controls，之后使用setCarControls设置状态
    # 这允许您设置油门(throttle)，转向(steering)，手制动(handbrake)和自动或手动档位(auto or manual gear)
    car_controls.throttle = 0.5
    car_controls.steering = 0
    client.setCarControls(car_controls)
    print("Go Forward")
    time.sleep(3)  # let car drive a bit

    # 前进+右转
    car_controls.throttle = 0.5
    car_controls.steering = 1
    client.setCarControls(car_controls)
    print("Go Forward, steer right")
    time.sleep(3)  # let car drive a bit

    # 倒车，记得要复原
    car_controls.throttle = -0.5
    car_controls.is_manual_gear = True
    car_controls.manual_gear = -1
    car_controls.steering = 0
    client.setCarControls(car_controls)
    print("Go reverse, steer right")
    time.sleep(3)  # let car drive a bit
    car_controls.is_manual_gear = False  # change back gear to auto
    car_controls.manual_gear = 0

    # 急刹
    car_controls.brake = 1
    client.setCarControls(car_controls)
    print("Apply brakes")
    time.sleep(3)  # let car drive a bit
    car_controls.brake = 0  # remove brake

# 将车辆初始化处理，后续如果需要重新控制则需要再次调用`enableApiControl`或`armDisarm`.'''
