import math
import time

from Utilities.global_data import *
from Utilities.geocoordinate import GeoCoordinate as Point
from GNC.guidance import calculate_los_angle


class Control:
    def __init__(self):
        self.waypoints = 0
        self.point_previous = Point(0.0, 0.0)
        self.point_current = Point(0.0, 0.0)
        self.point_desired = Point(0.0, 0.0)
        self.last_setting = 0
        self.waypoint_index = 0
        self.rudder_pid = PID()
        self.thrust_pid = PID()
        self.position_pid = PID()
        self.speed_max = 5

    def c_run(self):
        self.__update()
        self.__choose_mode()
        self.__control()

    def __update(self):
        self.point_current = Point(Nav_data['location']['latitude'],
                                   Nav_data['location']['longitude'])
        self.point_desired = Point(Gcs_command['desired_latitude'], Gcs_command['desired_longitude'])
        if Gcs_command['setting'] != 8:
            self.last_setting = Gcs_command['setting']

    def __control(self):
        if Ctrl_data['mode'] == 'lock':
            self.__lock()
        elif Ctrl_data['mode'] == 'manual':
            self.__manual()
        elif Ctrl_data['mode'] == 'line':
            self.__line()
        elif Ctrl_data['mode'] == 'gcs':
            self.__gcs()
        elif Ctrl_data['mode'] == 'heading':
            Ctrl_data['desired_heading'] = Gcs_command['desired_heading']
            self.__heading()
        elif Ctrl_data['mode'] == 'speed':
            Ctrl_data['desired_speed'] = Gcs_command['desired_speed']
            self.__speed()
        elif Ctrl_data['mode'] == 'heading_speed':
            Ctrl_data['desired_heading'] = Gcs_command['desired_heading']
            Ctrl_data['desired_speed'] = Gcs_command['desired_speed']
            self.__heading_speed()
        elif Ctrl_data['mode'] == 'waypoints':
            self.__waypoint()
        elif Ctrl_data['mode'] == 'waypoint_speed':
            self.__waypoint_speed()
        elif Ctrl_data['mode'] == 'trajectory_point':
            self.__trajectory_point()
        elif Ctrl_data['mode'] == 'mission':
            self.__mission()

    @staticmethod
    def __choose_mode():
        gcs_connected = time.time() - Gcs_heart_beat['timestamp'] < settings.gcs_disconnect_time_allow
        rcu_connected = (Rcu_data['flag'] & 0x04) == 0
        rcu_activity = abs(Rcu_data['channel1'] - Rcu_last_data['channel1']) + abs(
            Rcu_data['channel3'] - Rcu_last_data['channel3']) + abs(
            Rcu_data['channel5'] - Rcu_last_data['channel5']) > 10
        if rcu_connected or gcs_connected:
            # setting  0：遥控器控制，1：地面站控制，2：航向，3：航速，4：航向+航速，5：路点，6：路点+航速，7：轨点，8：任务(推力手动)
            if rcu_activity:
                Gcs_command['setting'] = 0  # 遥控器活跃，遥控器控制
            if gcs_connected and Gcs_command['setting'] != 0:  # 遥控器不活跃，地面站连接，根据地面站设置
                Ctrl_data['status'] = 2
                if Gcs_command['setting'] == 1:
                    Ctrl_data['mode'] = 'gcs'
                elif Gcs_command['setting'] == 2:
                    Ctrl_data['mode'] = 'heading'
                elif Gcs_command['setting'] == 3:
                    Ctrl_data['mode'] = 'speed'
                elif Gcs_command['setting'] == 4:
                    Ctrl_data['mode'] = 'heading_speed'
                elif Gcs_command['setting'] == 5:
                    Ctrl_data['mode'] = 'waypoints'
                elif Gcs_command['setting'] == 6:
                    Ctrl_data['mode'] = 'waypoint_speed'
                elif Gcs_command['setting'] == 7:
                    Ctrl_data['mode'] = 'trajectory_point'
                elif Gcs_command['setting'] == 8:
                    Ctrl_data['mode'] = 'mission'
            else:  # 遥控器连接，地面站未连接或遥控器活跃，遥控器控制
                Ctrl_data['status'] = 1
                if Rcu_data['channel5'] > 1360:
                    Ctrl_data['mode'] = 'manual'
                elif Rcu_data['channel5'] < 688:
                    Ctrl_data['mode'] = 'line'
                else:
                    Ctrl_data['mode'] = 'lock'
        else:
            Ctrl_data['status'] = 0
            Ctrl_data['mode'] = 'lock'  # 遥控器断开连接，锁定

    @staticmethod
    def __lock():
        Ctrl_data['thrust'] = 0
        Ctrl_data['rudder'] = 0

    @staticmethod
    def __manual():
        Ctrl_data['rudder'] = limit_1000(int(1.4881 * (Rcu_data['channel1'] - 1024)))
        Ctrl_data['thrust'] = limit_1000(int(-1.4881 * (Rcu_data['channel3'] - 1024)))

    def __line(self):
        Ctrl_data['thrust'] = limit_1000(int(-1.4881 * (Rcu_data['channel3'] - 1024)))

        if abs(Rcu_data['channel1'] - 1024) > 10:
            err = (Rcu_data['channel1'] - 1024) / 672 / 1 - Nav_data['gyroscope']['Z']
        else:
            err = 0 - Nav_data['gyroscope']['Z']
        Ctrl_data['rudder'] += int(
            self.rudder_pid.calculate_pid(err, Pid['heading_p'], Pid['heading_i'], Pid['heading_d']))

    @staticmethod
    def __gcs():  # 1
        Ctrl_data['rudder'] = limit_1000(int(Gcs_command['desired_rudder']))
        Ctrl_data['thrust'] = limit_1000(int(Gcs_command['desired_thrust']))

    def __heading(self):  # 2
        Ctrl_data['rudder'] = limit_1000(int(self.__control_heading(Ctrl_data['desired_heading'])))
        Ctrl_data['thrust'] = limit_1000(int(Gcs_command['desired_thrust']))

    def __speed(self):  # 3
        Ctrl_data['rudder'] = limit_1000(int(Gcs_command['desired_rudder']))
        Ctrl_data['thrust'] = limit_1000(int(self.__control_speed(Ctrl_data['desired_speed'])))

    def __heading_speed(self):  # 4
        Ctrl_data['rudder'] = limit_1000(int(self.__control_heading(Ctrl_data['desired_heading'])))
        Ctrl_data['thrust'] = limit_1000(int(self.__control_speed(Ctrl_data['desired_speed'])))

    def __waypoint(self):  # 5
        if self.point_current.distance2(self.point_desired) > settings.gcs_waypoint_err:
            Ctrl_data['desired_heading'] = self.point_current.azimuth2(self.point_desired)
            self.__heading()
        else:
            self.__point_keeping()

    def __waypoint_speed(self):  # 6
        if self.point_current.distance2(self.point_desired) > settings.gcs_waypoint_err:
            Ctrl_data['desired_heading'] = self.point_current.azimuth2(self.point_desired)
            Ctrl_data['desired_speed'] = Gcs_command['desired_speed']
            self.__heading_speed()
        else:
            self.__point_keeping()

    def __trajectory_point(self):  # 7
        # Ctrl_data['desired_heading'] = Gcs_command['desired_heading']
        # Ctrl_data['desired_speed'] = Gcs_command['desired_speed']
        # # 航向
        # desired_heading = self.point_current.azimuth2(self.point_desired)
        # if (abs(Ctrl_data['desired_heading'] - desired_heading) < math.pi / 2) or (3 * math.pi / 2 < abs(
        #         Ctrl_data['desired_heading'] - desired_heading) < 2 * math.pi):
        #     Ctrl_data['desired_heading'] = desired_heading
        # else:
        #     Ctrl_data['desired_heading'] = desired_heading + math.pi
        # if Ctrl_data['desired_heading'] > 2 * math.pi:
        #     Ctrl_data['desired_heading'] -= (2 * math.pi)
        #
        # # 航速
        # heading_distance = self.point_current.distance2(self.point_desired) * math.cos(
        #     desired_heading - Nav_data['posture']['yaw'])
        # Ctrl_data['desired_speed'] += self.position_pid.calculate_pid(heading_distance, Pid['position_p'],
        #                                                               Pid['position_i'], Pid['position_d'])
        speed = Gcs_command['desired_speed']
        yaw = Gcs_command['desired_heading']
        speedX = speed * math.cos(yaw)
        speedY = speed * math.sin(yaw)

        distance = self.point_current.distance2(self.point_desired)
        angle = self.point_current.azimuth2(self.point_desired)
        distanceX = distance * math.cos(angle)
        distanceY = distance * math.sin(angle)

        speedX += self.position_pid.calculate_pid(distanceX, Pid['position_p'], 0, 0)
        speedY += self.position_pid.calculate_pid(distanceY, Pid['position_p'], 0, 0)

        speed = math.hypot(speedX, speedY)

        if speed > self.speed_max:
            speed = self.speed_max

        yaw = math.atan2(speedY, speedX)
        if yaw < 0:
            yaw += math.pi * 2

        Ctrl_data['desired_heading'] = yaw
        Ctrl_data['desired_speed'] = speed

        self.__heading_speed()

    def __mission(self):  # 8
        if self.last_setting != 8:
            self.waypoints = mission.read()
            self.waypoint_index = 0
        if len(self.waypoints) != 0:
            way_point = self.waypoints[self.waypoint_index]
            point_next = Point(way_point[0], way_point[1])
            distance = self.point_current.distance2(point_next)
            # 根据容差确定是否要执行下一个点
            if distance < way_point[2] and self.waypoint_index < len(self.waypoints) - 1:
                self.point_previous.latitude = way_point[0]
                self.point_previous.longitude = way_point[1]
                self.waypoint_index += 1
                way_point = self.waypoints[self.waypoint_index]
                point_next.latitude = way_point[0]
                point_next.longitude = way_point[1]
            if self.waypoint_index == 0:
                Ctrl_data['desired_heading'] = self.point_current.azimuth2(point_next)
            else:
                Ctrl_data['desired_heading'] = calculate_los_angle(self.point_previous, self.point_current,
                                                                   point_next, settings.los_distance)
        self.last_setting = Gcs_command['setting']

        self.__heading()

    def __point_keeping(self):
        desired_heading = self.point_current.azimuth2(self.point_desired)
        if math.pi / 2 < abs(Nav_data['posture']['yaw'] - desired_heading) and abs(
                Nav_data['posture']['yaw'] - desired_heading) < 3 * math.pi / 2:
            Ctrl_data['desired_heading'] = desired_heading + math.pi
        else:
            Ctrl_data['desired_heading'] = desired_heading
        if Ctrl_data['desired_heading'] > 2 * math.pi:
            Ctrl_data['desired_heading'] -= (2 * math.pi)

        Ctrl_data['rudder'] = limit_1000(int(self.__control_heading(Ctrl_data['desired_heading'])))

        heading_distance = self.point_current.distance2(self.point_desired) * math.cos(
            desired_heading - Nav_data['posture']['yaw'])
        Ctrl_data['thrust'] = limit_1000(int(self.thrust_pid.calculate_pid(heading_distance, Pid['position_p'],
                                                                           Pid['position_i'], Pid['position_d'])))

    def __control_heading(self, desired_heading):
        heading_err = desired_heading - Nav_data['posture']['yaw']
        if heading_err < -math.pi:
            heading_err += 2 * math.pi
        if heading_err > math.pi:
            heading_err -= 2 * math.pi
        return self.rudder_pid.calculate_pid(heading_err, Pid['heading_p'], Pid['heading_i'], Pid['heading_d'])

    def __control_speed(self, desired_speed):
        speed_err = desired_speed - Nav_data['velocity']['speed']
        return self.thrust_pid.calculate_pid(speed_err, Pid['speed_p'], Pid['speed_i'], Pid['speed_d'])


def limit_1000(value):
    if value > 1000:
        value = 1000
    if value < -1000:
        value = -1000
    return value


class PID:
    def __init__(self):
        self.data = {'last_p': 0, 'last_i': 0, 'last_d': 0, 'err_i': 0.0, 'last_err': 0.0, 'period': 0.01, ' p': 0,
                     'i': 0, 'd': 0}

    def calculate_pid(self, err, p, i, d):
        if abs(self.data['last_p'] - p) > 0.001 or abs(self.data['last_i'] - i) > 0.001 or abs(
                self.data['last_d'] - d) > 0.001:
            self.data['err_i'] = 0

        self.data['err_i'] += err * self.data['period']
        err_d = (err - self.data['last_err']) / self.data['period']

        self.data['last_p'] = p
        self.data['last_i'] = i
        self.data['last_d'] = d
        self.data['last_err'] = err

        return err * p + self.data['err_i'] * i + err_d * d
