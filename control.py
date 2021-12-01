import math
import time


class Control:
    def __init__(self):
        self.data = {'mode': 'lock', 'thrust': 0, 'rudder': 0, 'ignition': 0}
        self.pid = {'heading_p': 500.0, 'heading_i': 0.0, 'heading_d': 0.0, 'speed_p': 0.0, 'speed_i': 0.0,
                    'speed_d': 0.0, 'position_p': 0.0, 'position_i': 0.0, 'position_d': 0.0, 'id': 0x0000}
        self.rudder_pid = Pid()
        self.thrust_pid = Pid()
        self.status = 0  # 无人船状态
        self.depth = 0
        self.battery = 0

    def c_run(self, usv):
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
        elif self.data['mode'] == 'waypoint':
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
                    self.data['mode'] = 'waypoint'
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

    def gcs(self, usv):
        self.data['rudder'] = limit_1000(int(usv.gcs.command['desired_rudder']))
        self.data['thrust'] = limit_1000(int(usv.gcs.command['desired_thrust']))

    def heading(self, usv):
        self.data['rudder'] = limit_1000(int(self.control_heading(usv)))
        self.data['thrust'] = limit_1000(int(usv.gcs.command['desired_thrust']))

    def speed(self, usv):
        self.data['rudder'] = limit_1000(int(usv.gcs.command['desired_rudder']))
        self.data['thrust'] = limit_1000(int(self.control_speed(usv)))

    def heading_speed(self, usv):
        self.data['rudder'] = limit_1000(int(self.control_heading(usv)))
        self.data['thrust'] = limit_1000(int(self.control_speed(usv)))

    def waypoint(self, usv):
        pass

    def waypoint_speed(self, usv):
        pass

    def trajectory_point(self, usv):
        pass

    def mission(self, usv):
        pass

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
