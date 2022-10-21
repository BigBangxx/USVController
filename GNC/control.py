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
        self.last_mode = 'lock'
        self.waypoint_index = 0
        self.rudder_pid = PID()
        self.thrust_expert_pid = ExpertPID()
        self.thrust_pid = PID()
        self.position_pid = PID()
        self.speed_max = settings.speed_max
        self.lastInRing = False

    def c_run(self):
        self.__update()
        self.__choose_mode()
        self.__control()
        Gcs_command['average_index'] = Gcs_command['index_sum'] / Gcs_command['ship_num']

    def __update(self):
        self.point_current = Point(Nav_data['location']['latitude'],
                                   Nav_data['location']['longitude'])
        self.point_desired = Point(Gcs_command['desired_latitude'], Gcs_command['desired_longitude'])
        if Gcs_command['setting'] != 8 and Gcs_command['setting'] != 9:
            self.last_setting = Gcs_command['setting']
        if Gcs_command['setting'] != 9:
            Gcs_command['index_sum'] = 0
            Gcs_command['index'] = 0
        if Ctrl_data['mode'] != 'point':
            self.last_mode = Ctrl_data['mode']

    def __control(self):
        if Ctrl_data['mode'] == 'lock':
            self.__lock()
        elif Ctrl_data['mode'] == 'manual':
            self.__manual()
        elif Ctrl_data['mode'] == 'point':
            if self.last_mode != 'point':  # 记录当前位置
                Gcs_command['desired_latitude'] = Nav_data['location']['latitude']
                Gcs_command['desired_longitude'] = Nav_data['location']['longitude']
                self.point_desired = Point(Gcs_command['desired_latitude'], Gcs_command['desired_longitude'])
            self.__point_keeping()
            self.last_mode = 'point'
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
        elif Ctrl_data['mode'] == 'formation_mission':
            self.__formation_mission()

    @staticmethod
    def __choose_mode():
        gcs_connected = time.time() - Gcs_heart_beat['timestamp'] < settings.gcs_disconnect_time_allow
        rcu_connected = (Rcu_data['flag'] & 0x04) == 0
        rcu_activity = abs(Rcu_data['channel1'] - Rcu_last_data['channel1']) + abs(
            Rcu_data['channel3'] - Rcu_last_data['channel3']) + abs(
            Rcu_data['channel5'] - Rcu_last_data['channel5']) > 10
        if rcu_connected or gcs_connected:
            # setting  0：遥控器控制，1：地面站控制，2：航向，3：航速，4：航向+航速，5：路点，6：路点+航速，7：轨点，8：任务(推力手动)
            # if rcu_activity:
            #     Gcs_command['setting'] = 0  # 遥控器活跃，遥控器控制
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
                elif Gcs_command['setting'] == 9:
                    Ctrl_data['mode'] = 'formation_mission'
            else:  # 遥控器连接，地面站未连接或遥控器活跃，遥控器控制
                Ctrl_data['status'] = 1
                if Rcu_data['channel5'] > 1360:
                    Ctrl_data['mode'] = 'manual'
                elif Rcu_data['channel5'] < 688:
                    Ctrl_data['mode'] = 'point'
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

    def __waypoint_speed(self):  # 6 路点航速航向
        distance = self.point_current.distance2(self.point_desired)
        if not self.lastInRing:  # 上次不在外圈内先路点模式进入内圈
            if distance <= 2:  # 进入内圈,控制航向
                self.lastInRing = True
            else:
                Ctrl_data['desired_heading'] = self.point_current.azimuth2(self.point_desired)
                Ctrl_data['desired_speed'] = Gcs_command['desired_speed']
                self.__heading_speed()
        else:  # 上次在外圈内，控制航向
            if distance > settings.gcs_waypoint_err:  # 出外圈
                self.lastInRing = False
            else:
                Ctrl_data['desired_heading'] = Gcs_command['desired_heading']
                self.__heading()
                Ctrl_data['thrust'] = 0

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
        # ############################################################################ 纯速度矢量
        if settings.formation_type == 0:
            speedX = speed * math.cos(yaw)
            speedY = speed * math.sin(yaw)

            angle = self.point_current.azimuth2(self.point_desired)
            distance = self.point_current.distance2(self.point_desired)
            distanceX = distance * math.cos(angle)
            distanceY = distance * math.sin(angle)

            speedX += self.position_pid.calculate_pid(distanceX, Pid['position_p'], Pid['position_i'],
                                                      Pid['position_d'])
            speedY += self.position_pid.calculate_pid(distanceY, Pid['position_p'], Pid['position_i'],
                                                      Pid['position_d'])

            speed = math.hypot(speedX, speedY)

            if speed > self.speed_max:
                speed = self.speed_max

            yaw = math.atan2(speedY, speedX)
            if yaw < 0:
                yaw += math.pi * 2
        # ############################################################################ 制导加速度矢量
        if settings.formation_type == 1:
            angle = self.point_current.azimuth2(self.point_desired)
            d_angle = abs(angle - yaw)
            if d_angle < math.pi / 2 or 3 * math.pi / 2 < d_angle < 2 * math.pi:  # 目标点在无人艇前方
                speedX = speed * math.cos(yaw)
                speedY = speed * math.sin(yaw)

                distance = self.point_current.distance2(self.point_desired)
                distanceX = distance * math.cos(angle)
                distanceY = distance * math.sin(angle)

                speedX += self.position_pid.calculate_pid(distanceX, Pid['position_p'], Pid['position_i'],
                                                          Pid['position_d'])
                speedY += self.position_pid.calculate_pid(distanceY, Pid['position_p'], Pid['position_i'],
                                                          Pid['position_d'])

                speed = math.hypot(speedX, speedY)

                if speed > self.speed_max:
                    speed = self.speed_max

                yaw = math.atan2(speedY, speedX)
                if yaw < 0:
                    yaw += math.pi * 2

            else:  # 目标点在无人艇后方
                speed *= settings.speed_coefficient
                self.position_pid.data['err_i'] = 0
                point_next = self.point_desired.at_distance_and_azimuth(999, yaw)
                yaw = calculate_los_angle(self.point_desired, self.point_current, point_next,
                                          settings.los_distance_tracking)

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
        if self.waypoint_index == len(self.waypoints) - 1:
            Gcs_command['desired_thrust'] = 0
        self.last_setting = Gcs_command['setting']

        self.__heading()

    def __formation_mission(self):  # 9
        if self.last_setting != 9:
            self.waypoints = mission.read()
            self.waypoints = generate_follow_path(self.waypoints, math.radians(Gcs_command['angle']),
                                                  Gcs_command['distance'])
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
                Globals['Send_arrive_waypoint_packet'] = True
            if self.waypoint_index == 0:
                Ctrl_data['desired_heading'] = self.point_current.azimuth2(point_next)
            else:
                Ctrl_data['desired_heading'] = calculate_los_angle(self.point_previous, self.point_current,
                                                                   point_next, settings.los_distance)
            # 推力控制
            if Gcs_command['index_sum'] / Gcs_command['ship_num'] < self.waypoint_index:  # 还有艇未到达相应路点
                Gcs_command['desired_thrust'] = 0
            # elif Gcs_command['index_sum'] / Gcs_command['ship_num'] > self.waypoint_index:  # 有艇到达路点，我还没到
            #     pass
            elif Gcs_command['angle'] == 0 and Gcs_command['distance'] == 0:
                pass
            else:
                distance = self.point_current.distance2(self.point_desired)
                angle = self.point_current.azimuth2(self.point_desired)
                d_angle = abs(angle - Gcs_command['desired_heading'])
                if d_angle < math.pi / 2 or 3 * math.pi / 2 < d_angle < 2 * math.pi:  # 目标点在无人艇前方
                    Gcs_command['desired_thrust'] += self.position_pid.calculate_pid(distance, Pid['position_p'],
                                                                                     Pid['position_i'],
                                                                                     Pid['position_d'])
                else:
                    Gcs_command['desired_thrust'] -= self.position_pid.calculate_pid(distance, Pid['position_p'],
                                                                                     Pid['position_i'],
                                                                                     Pid['position_d'])
                if Gcs_command['desired_thrust'] > 700:
                    Gcs_command['desired_thrust'] = 700
                if Gcs_command['desired_thrust'] < 150:
                    Gcs_command['desired_thrust'] = 150
        if self.waypoint_index == len(self.waypoints) - 1:
            Gcs_command['desired_thrust'] = 0

        self.last_setting = Gcs_command['setting']
        Gcs_command['index'] = self.waypoint_index

        self.__heading()

    def __point_keeping(self):
        # desired_heading = self.point_current.azimuth2(self.point_desired)
        # if math.pi / 2 < abs(Nav_data['posture']['yaw'] - desired_heading) and abs(
        #         Nav_data['posture']['yaw'] - desired_heading) < 3 * math.pi / 2:
        #     Ctrl_data['desired_heading'] = desired_heading + math.pi
        # else:
        #     Ctrl_data['desired_heading'] = desired_heading
        # if Ctrl_data['desired_heading'] > 2 * math.pi:
        #     Ctrl_data['desired_heading'] -= (2 * math.pi)
        #
        # Ctrl_data['rudder'] = limit_1000(int(self.__control_heading(Ctrl_data['desired_heading'])))
        #
        # heading_distance = self.point_current.distance2(self.point_desired) * math.cos(
        #     desired_heading - Nav_data['posture']['yaw'])
        # if abs(heading_distance) > 1.5:
        #     Ctrl_data['thrust'] = limit_1000(int(self.thrust_pid.calculate_pid(heading_distance, Pid['position_p'],
        #                                                                        Pid['position_i'], Pid['position_d'])))
        # else:
        #     Ctrl_data['thrust'] = 0
        self.__lock()

    def __control_heading(self, desired_heading):
        heading_err = desired_heading - Nav_data['posture']['yaw']
        if heading_err < -math.pi:
            heading_err += 2 * math.pi
        if heading_err > math.pi:
            heading_err -= 2 * math.pi
        return self.rudder_pid.calculate_pid(heading_err, Pid['heading_p'], Pid['heading_i'], Pid['heading_d'])

    def __control_speed(self, desired_speed):
        speed_err = desired_speed - Nav_data['velocity']['speed']
        if settings.enable_speed_expert_PID:
            return self.thrust_expert_pid.calculate_expert_pid(speed_err, Pid['speed_p'], Pid['speed_i'],
                                                               Pid['speed_d'], settings.speed_expert_PID_max,
                                                               settings.speed_expert_PID_mid,
                                                               settings.speed_expert_PID_min)
        else:
            return self.thrust_pid.calculate_pid(speed_err, Pid['speed_p'], Pid['speed_i'], Pid['speed_d'])


def limit_1000(value):
    if value > 1000:
        value = 1000
    if value < -1000:
        value = -1000
    return value


def generate_follow_path(coordinate, angle, distance):
    len_ = len(coordinate)
    if len_ == 0 or len_ == 1 or (angle == 0 and distance == 0):
        return coordinate
    point_first = Point(coordinate[0][0], coordinate[0][1])
    point_last = Point(coordinate[-1][0], coordinate[-1][1])
    result = []
    tolerance = coordinate[0][2]
    for i in range(len_):
        if i == 0:
            point_second = Point(coordinate[1][0], coordinate[1][1])
            north_angle = point_first.azimuth2(point_second)  # 第一二个点的弧度
            follow_point = point_first.at_distance_and_azimuth(distance, north_angle + angle)
            waypoint = (follow_point.latitude, follow_point.longitude, tolerance)
            result.append(waypoint)
            continue
        if i == len_ - 1:
            point_last_pre = Point(coordinate[-2][0], coordinate[-2][1])
            north_angle = point_last_pre.azimuth2(point_last)  # 倒一二个点的弧度
            follow_point = point_last.at_distance_and_azimuth(distance, north_angle + angle)
            waypoint = (follow_point.latitude, follow_point.longitude, tolerance)
            result.append(waypoint)
            break
        point = Point(coordinate[i][0], coordinate[i][1])
        point_pre = Point(coordinate[i - 1][0], coordinate[i - 1][1])
        point_next = Point(coordinate[i + 1][0], coordinate[i + 1][1])
        north_angle1 = point_pre.azimuth2(point)
        north_angle2 = point.azimuth2(point_next)

        point1 = point_pre.at_distance_and_azimuth(distance, north_angle1 + angle)
        point2 = point.at_distance_and_azimuth(distance, north_angle1 + angle)
        point3 = point.at_distance_and_azimuth(distance, north_angle2 + angle)
        point4 = point_next.at_distance_and_azimuth(distance, north_angle2 + angle)

        a1 = point1.latitude - point2.latitude
        b1 = point2.longitude - point1.longitude
        c1 = point1.longitude * point2.latitude - point2.longitude * point1.latitude

        a2 = point3.latitude - point4.latitude
        b2 = point4.longitude - point3.longitude
        c2 = point3.longitude * point4.latitude - point4.longitude * point3.latitude

        d = a1 * b2 - a2 * b1

        if d == 0:
            follow_point = point.at_distance_and_azimuth(distance, north_angle1 + angle)
            waypoint = (follow_point.latitude, follow_point.longitude, tolerance)
            result.append(waypoint)
        else:
            y = (a2 * c1 - a1 * c2) / d
            x = (b1 * c2 - b2 * c1) / d
            waypoint = (y, x, tolerance)
            result.append(waypoint)

    return result


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


class ExpertPID:
    def __init__(self):
        self.data = {'last_err': 0.0, 'last_d_err': 0.0, 'last_output': 0.0, 'k1': settings.expert_PID_k1,
                     'k2': settings.expert_PID_k2}

    def calculate_expert_pid(self, err, p, i, d, max_, mid_, min_):
        d_err = err - self.data['last_err']
        output = 0
        if abs(err) > max_:  # 误差较大输出最大值，快速响应
            output = err * 1000
        elif err * d_err > 0 or d_err == 0:  # 误差在朝误差绝对值增大方向变化，或误差为某一常值，未发生变化
            if abs(err) > mid_:  # 误差较大, 输出较强控制作用
                output = self.data['last_output'] + self.data['k1'] * p * d_err + i * err + d * (
                        d_err - self.data['last_d_err'])
            else:  # 误差不大， 输出一般控制作用
                output = self.data['last_output'] + p * d_err + i * err + d * (d_err - self.data['last_d_err'])
        elif (err * d_err < 0 and d_err * self.data['last_d_err'] > 0) or err == 0:  # 误差的绝对值朝减小的方向变化，或者已经达到平衡状态
            output = self.data['last_output']
        elif err * d_err < 0 and d_err * self.data['last_d_err'] < 0:  # 误差处于极值状态
            if abs(err) > mid_:  # 误差较大, 输出较强控制作用
                output = self.data['last_output'] + self.data['k1'] * p * err
            else:  # 误差不大， 输出较弱控制作用
                output = self.data['last_output'] + self.data['k2'] * p * err
        elif err < min_:
            output = self.data['last_output'] + p * d_err + i * err
        self.data['last_err'] = err
        self.data['last_d_err'] = d_err
        self.data['last_output'] = output
        return output
