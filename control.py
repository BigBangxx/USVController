class Control:
    def __init__(self):
        self.mode = ['lock', 'manual', 'line']
        self.data = {'mode': 'lock', 'thrust': 0, 'rudder': 0, }

    def c_run(self, usv):
        self.choose_mode(usv)
        self.control(usv)

    def control(self, usv):
        # 锁定
        if self.data['mode'] == 'lock':
            self.data['thrust'] = 0
            self.data['rudder'] = 0
        # 手动
        elif self.data['mode'] == 'manual':
            self.data['rudder'] = int(1.4881 * (usv.futaba.receive_data['channel1'] - 1024))
            self.data['thrust'] = int(-1.4881 * (usv.futaba.receive_data['channel3'] - 1024))

    def choose_mode(self, usv):
        if usv.futaba.receive_data['channel5'] > 1360:
            self.data['mode'] = self.mode[1]
        elif usv.futaba.receive_data['channel5'] < 688:
            self.data['mode'] = self.mode[2]
        else:
            self.data['mode'] = self.mode[0]
