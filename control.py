import math
import time

from PySide2.QtPositioning import QGeoCoordinate as Point
from SelfBuiltModul.func import degrees_to_radians, radians_to_degrees
from guidance import calculate_los_angle


class Control:
    def __init__(self, usv):
        self.data = {'mode': 'lock', 'thrust': 0, 'rudder': 0, 'ignition': 0}
        self.pid = {'heading_p': usv.settings.heading_p, 'heading_i': usv.settings.heading_i,
                    'heading_d': usv.settings.heading_d, 'speed_p': usv.settings.speed_p,
                    'speed_i': usv.settings.speed_i, 'speed_d': usv.settings.speed_d,
                    'position_p': usv.settings.position_p, 'position_i': usv.settings.position_i,
                    'position_d': usv.settings.position_d, 'id': usv.settings.usv_id}
        self.point_previous = Point(0.0, 0.0)
        self.point_current = Point(0.0, 0.0)
        self.point_desired = Point(0.0, 0.0)
        self.last_setting = 0
        self.waypoint_index = 0
        self.rudder_pid = Pid()
        self.thrust_pid = Pid()
        self.position_pid = Pid()
        self.status = 0  # 无人船控制状态
        self.depth = 0
        self.battery = 0

    def update(self, usv):
        self.point_current = Point(radians_to_degrees(usv.navigation.data['location']['latitude']),
                                   radians_to_degrees(usv.navigation.data['location']['longitude']))
        self.point_desired = Point(radians_to_degrees(usv.gcs.command['desired_latitude']),
                                   radians_to_degrees(usv.gcs.command['desired_longitude']))
        if usv.gcs.command['setting'] != 8:
            self.last_setting = usv.gcs.command['setting']

    def c_run(self, usv):
        self.update(usv)
        self.choose_mode(usv)
        self.control(usv)

    def control(self, usv):
        if self.data['mode'] == 'lock':
            self.lock()
        elif self.data['mode'] == 'manual':
            self.manual(usv)
        elif self.data['mode'] == 'line':
            self.line(usv)
        elif self.data['mode'] == 'gcs':
            self.gcs(usv)
        elif self.data['mode'] == 'heading':
            self.heading(usv)
        elif self.data['mode'] == 'speed':
            self.speed(usv)
        elif self.data['mode'] == 'heading_speed':
            self.heading_speed(usv)
        elif self.data['mode'] == 'waypoints':
            self.waypoint(usv)
        elif self.data['mode'] == 'waypoint_speed':
            self.waypoint_speed(usv)
        elif self.data['mode'] == 'trajectory_point':
            self.trajectory_point(usv)
        elif self.data['mode'] == 'mission':
            self.mission(usv)

    def choose_mode(self, usv):
        gcs_connected = time.time() - usv.gcs.heart_beat['timestamp'] < usv.settings.gcs_disconnect_time_allow
        rcu_connected = (usv.futaba.receive_data['flag'] & 0x04) == 0
        rcu_activity = abs(usv.futaba.receive_data['channel1'] - usv.futaba.last_data['channel1']) + abs(
            usv.futaba.receive_data['channel3'] - usv.futaba.last_data['channel3']) + abs(
            usv.futaba.receive_data['channel5'] - usv.futaba.last_data['channel5']) > 10
        if rcu_connected or gcs_connected:
            # setting  0：遥控器控制，1：地面站控制，2：航向，3：航速，4：航向+航速，5：路点，6：路点+航速，7：轨点，8：任务(推力手动)
            if rcu_activity:
                usv.gcs.command['setting'] = 0  # 遥控器活跃，遥控器控制
            if gcs_connected and usv.gcs.command['setting'] != 0:  # 遥控器不活跃，地面站连接，根据地面站设置
                self.status = 2
                if usv.gcs.command['setting'] == 1:
                    self.data['mode'] = 'gcs'
                elif usv.gcs.command['setting'] == 2:
                    self.data['mode'] = 'heading'
                elif usv.gcs.command['setting'] == 3:
                    self.data['mode'] = 'speed'
                elif usv.gcs.command['setting'] == 4:
                    self.data['mode'] = 'heading_speed'
                elif usv.gcs.command['setting'] == 5:
                    self.data['mode'] = 'waypoints'
                elif usv.gcs.command['setting'] == 6:
                    self.data['mode'] = 'waypoint_speed'
                elif usv.gcs.command['setting'] == 7:
                    self.data['mode'] = 'trajectory_point'
                elif usv.gcs.command['setting'] == 8:
                    self.data['mode'] = 'mission'
            else:  # 遥控器连接，地面站未连接或遥控器活跃，遥控器控制
                self.status = 1
                if usv.futaba.receive_data['channel5'] > 1360:
                    self.data['mode'] = 'manual'
                elif usv.futaba.receive_data['channel5'] < 688:
                    self.data['mode'] = 'line'
                else:
                    self.data['mode'] = 'lock'
        else:
            self.status = 0
            self.data['mode'] = 'lock'  # 遥控器断开连接，锁定

    def lock(self):
        self.data['thrust'] = 0
        self.data['rudder'] = 0

    def manual(self, usv):
        self.data['rudder'] = limit_1000(int(1.4881 * (usv.futaba.receive_data['channel1'] - 1024)))
        self.data['thrust'] = limit_1000(int(-1.4881 * (usv.futaba.receive_data['channel3'] - 1024)))

    def line(self, usv):
        self.data['thrust'] = limit_1000(int(-1.4881 * (usv.futaba.receive_data['channel3'] - 1024)))

        if abs(usv.futaba.receive_data['channel1'] - 1024) > 10:
            err = (usv.futaba.receive_data['channel1'] - 1024) / 672 / 1 - usv.navigation.data['gyroscope']['Z']
        else:
            err = 0 - usv.navigation.data['gyroscope']['Z']
        self.data['rudder'] = limit_1000(
            int(self.rudder_pid.calculate_pid(err, self.pid['heading_p'], self.pid['heading_i'],
                                              self.pid['heading_d'])))

    def gcs(self, usv):  # 1
        self.data['rudder'] = limit_1000(int(usv.gcs.command['desired_rudder']))
        self.data['thrust'] = limit_1000(int(usv.gcs.command['desired_thrust']))

    def heading(self, usv):  # 2
        self.data['rudder'] = limit_1000(int(self.control_heading(usv)))
        self.data['thrust'] = limit_1000(int(usv.gcs.command['desired_thrust']))

    def speed(self, usv):  # 3
        self.data['rudder'] = limit_1000(int(usv.gcs.command['desired_rudder']))
        self.data['thrust'] = limit_1000(int(self.control_speed(usv)))

    def heading_speed(self, usv):  # 4
        self.data['rudder'] = limit_1000(int(self.control_heading(usv)))
        self.data['thrust'] = limit_1000(int(self.control_speed(usv)))

    def waypoint(self, usv):  # 5
        if self.point_current.distanceTo(self.point_desired) > usv.settings.gcs_waypoint_err:
            usv.gcs.command['desired_heading'] = degrees_to_radians(self.point_current.azimuthTo(self.point_desired))
            self.heading(usv)
        else:
            self.lock()

    def waypoint_speed(self, usv):  # 6
        if self.point_current.distanceTo(self.point_desired) > usv.settings.gcs_waypoint_err:
            usv.gcs.command['desired_heading'] = degrees_to_radians(self.point_current.azimuthTo(self.point_desired))
            self.heading_speed(usv)
        else:
            self.lock()

    def trajectory_point(self, usv):  # 7
        # 航向
        desired_heading = degrees_to_radians(self.point_current.azimuthTo(self.point_desired))
        if abs(usv.gcs.command['desired_heading'] - desired_heading > math.pi) / 2:
            usv.gcs.command['desired_heading'] = desired_heading + math.pi
        else:
            usv.gcs.command['desired_heading'] = desired_heading
        if usv.gcs.command['desired_heading'] > 2 * math.pi:
            usv.gcs.command['desired_heading'] -= (2 * math.pi)

        # 航速
        heading_distance = self.point_current.distanceTo(self.point_desired) * math.cos(
            desired_heading - usv.navigation.data['posture']['yaw'])
        usv.gcs.command['desired_speed'] += self.position_pid.calculate_pid(heading_distance, self.pid['position_p'],
                                                                            self.pid['position_i'],
                                                                            self.pid['position_d'])

        self.heading_speed(usv)

    def mission(self, usv):  # 8
        if self.last_setting != 8:
            usv.gcs.waypoints = usv.mission.read()
            self.waypoint_index = 0
        if len(usv.gcs.waypoints) != 0:
            way_point = usv.gcs.waypoints[self.waypoint_index]
            point_next = Point(radians_to_degrees(way_point[0]), radians_to_degrees(way_point[1]))
            distance = self.point_current.distanceTo(point_next)
            # 根据容差确定是否要执行下一个点
            if distance < way_point[2] and self.waypoint_index < len(usv.gcs.waypoints) - 1:
                self.point_previous.setLatitude(radians_to_degrees(way_point[0]))
                self.point_previous.setLongitude(radians_to_degrees(way_point[1]))
                self.waypoint_index += 1
                way_point = usv.gcs.waypoints[self.waypoint_index]
                point_next.setLatitude(radians_to_degrees(way_point[0]))
                point_next.setLongitude(radians_to_degrees(way_point[1]))
            if self.waypoint_index == 0:
                usv.gcs.command['desired_heading'] = degrees_to_radians(self.point_current.azimuthTo(point_next))
            else:
                usv.gcs.command['desired_heading'] = calculate_los_angle(self.point_previous, self.point_current,
                                                                         point_next, usv.settings.los_distance)
        self.last_setting = usv.gcs.command['setting']

        self.heading(usv)

    def control_heading(self, usv):
        heading_err = usv.gcs.command['desired_heading'] - usv.navigation.data['posture']['yaw']
        if heading_err < -math.pi:
            heading_err += 2 * math.pi
        if heading_err > math.pi:
            heading_err -= 2 * math.pi
        return self.rudder_pid.calculate_pid(heading_err, self.pid['heading_p'], self.pid['heading_i'],
                                             self.pid['heading_d'])

    def control_speed(self, usv):
        speed_err = usv.gcs.command['desired_speed'] - usv.navigation.data['velocity']['speed']
        return self.thrust_pid.calculate_pid(speed_err, self.pid['speed_p'], self.pid['speed_i'],
                                             self.pid['speed_d'])


def limit_1000(value):
    if value > 1000:
        value = 1000
    if value < -1000:
        value = -1000
    return value


class Pid:
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
