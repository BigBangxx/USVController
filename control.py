class Control:
    def __init__(self):
        self.mode = ['lock', 'manual', 'line']
        self.data = {'mode': 'lock', 'thrust': 0, 'rudder': 0, 'ignition': 0}
        self.pid = {'heading_p': 500.0, 'heading_i': 0.0, 'heading_d': 0.0, 'speed_p': 0.0, 'speed_i': 0.0,
                    'speed_d': 0.0, 'position_p': 0.0, 'position_i': 0.0, 'position_d': 0.0, 'id': 0x0000}
        self.rudder_pid = Pid()
        self.status = 0
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

    def choose_mode(self, usv):
        if usv.futaba.receive_data['channel5'] > 1360:
            self.data['mode'] = self.mode[1]
        elif usv.futaba.receive_data['channel5'] < 688:
            self.data['mode'] = self.mode[2]
        else:
            self.data['mode'] = self.mode[0]

    def lock(self):
        self.data['thrust'] = 0
        self.data['rudder'] = 0

    def manual(self, usv):
        self.data['rudder'] = int(1.4881 * (usv.futaba.receive_data['channel1'] - 1024))
        self.data['thrust'] = int(-1.4881 * (usv.futaba.receive_data['channel3'] - 1024))

    def line(self, usv):
        self.data['thrust'] = int(-1.4881 * (usv.futaba.receive_data['channel3'] - 1024))

        if abs(usv.futaba.receive_data['channel1'] - 1024) > 10:
            err = (usv.futaba.receive_data['channel1'] - 1024) / 672 / 1 - usv.navigation.data['gyroscope']['Z']
        else:
            err = 0 - usv.navigation.data['gyroscope']['Z']
        self.data['rudder'] = int(self.rudder_pid.calculate_pid(err, self.pid['heading_p'], self.pid['heading_i'],
                                                                self.pid['heading_d']))


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
