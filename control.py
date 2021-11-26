import struct
from remote_control_unit import RemoteControlUnit


class Control:
    def __init__(self):
        self.mode = ['lock', 'manual']
        self.data = {'mode': self.mode[1], 'thrust': 0, 'rudder': 0, }

    def run(self, usv):
        if self.data['mode'] == self.mode[1]:
            self.data['rudder'] = int(1.4881 * (usv.futaba.receive_data['channel1'] - 1024))
            self.data['thrust'] = int(-1.4881 * (usv.futaba.receive_data['channel3'] - 1024))
