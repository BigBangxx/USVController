import serial

from GNC.control import limit_1000
from Utilities.global_data import Ctrl_data, Rcu_data, Rcu_last_data, settings


class RemoteControlUnit:
    """遥控器相关属性及方法"""

    def __init__(self):
        self.send_data = Rcu_data.copy()
        self.last_sent_data = self.send_data.copy()
        self.sbus = serial.Serial(settings.sbus_com, 100000, serial.EIGHTBITS, serial.PARITY_EVEN, serial.STOPBITS_TWO,
                                  0, False, False, 0)
        self.buffer = bytearray()

    def rcu_run(self):
        self.__backup()
        self.__decode()
        self.__send_command()

    def __send_command(self):
        if settings.navigation_type != "airsim":
            self.send_data = Rcu_data.copy()
            if settings.is_catamaran:
                self.send_data['channel3'] = int(
                    1024 + (limit_1000(Ctrl_data['rudder'] + Ctrl_data['thrust']) * 0.672))  # 左电机
                self.send_data['channel1'] = int(
                    1024 + (limit_1000(Ctrl_data['thrust'] - Ctrl_data['rudder']) * 0.672))  # 右电机
                self.send_data['channel3'] = self.__limit_change_rate(self.send_data['channel3'],
                                                                      self.last_sent_data['channel3'],
                                                                      settings.limit_sbus_change_rate,
                                                                      settings.ctrl_cycle_time)
                self.send_data['channel1'] = self.__limit_change_rate(self.send_data['channel1'],
                                                                      self.last_sent_data['channel1'],
                                                                      settings.limit_sbus_change_rate,
                                                                      settings.ctrl_cycle_time)
                self.last_sent_data = self.send_data.copy()
            else:
                self.send_data['channel3'] = int(1024 + (Ctrl_data['thrust'] * 0.672))  # 推进器
                self.send_data['channel1'] = int(1024 + (Ctrl_data['rudder'] * 0.672))  # 舵
            packet = self.__encode()
            self.sbus.write(packet)

    def __decode(self):
        """读取数据并解包数据"""
        self.buffer += self.sbus.read(self.sbus.in_waiting)

        while len(self.buffer) >= 25:
            if self.buffer[0] != 0x0f:
                del self.buffer[0]
                continue

            if self.buffer[24] != 0x04 and self.buffer[24] != 0x14 and self.buffer[24] != 0x24 and self.buffer[
                24] != 0x34:
                del self.buffer[0]
                Rcu_data['error'] += 1
                continue

            Rcu_data['channel1'] = (self.buffer[1] >> 0 | self.buffer[2] << 8) & 0x07ff
            Rcu_data['channel2'] = (self.buffer[2] >> 3 | self.buffer[3] << 5) & 0x07ff
            Rcu_data['channel3'] = (self.buffer[3] >> 6 | self.buffer[4] << 2 |
                                    self.buffer[5] << 10) & 0x07ff
            Rcu_data['channel4'] = (self.buffer[5] >> 1 | self.buffer[6] << 7) & 0x07ff
            Rcu_data['channel5'] = (self.buffer[6] >> 4 | self.buffer[7] << 4) & 0x07ff
            Rcu_data['channel6'] = (self.buffer[7] >> 7 | self.buffer[8] << 1 |
                                    self.buffer[9] << 9) & 0x07ff
            Rcu_data['channel7'] = (self.buffer[9] >> 2 | self.buffer[10] << 6) & 0x07ff
            Rcu_data['channel8'] = (self.buffer[10] >> 5 | self.buffer[11] << 3) & 0x07ff
            Rcu_data['channel9'] = (self.buffer[12] >> 0 | self.buffer[13] << 8) & 0x07ff
            Rcu_data['channel10'] = (self.buffer[13] >> 3 | self.buffer[14] << 5) & 0x07ff
            Rcu_data['channel11'] = (self.buffer[14] >> 6 | self.buffer[15] << 2 |
                                     self.buffer[16] << 10) & 0x07ff
            Rcu_data['channel12'] = (self.buffer[16] >> 1 | self.buffer[17] << 7) & 0x07ff
            Rcu_data['channel13'] = (self.buffer[17] >> 4 | self.buffer[18] << 4) & 0x07ff
            Rcu_data['channel14'] = (self.buffer[18] >> 7 | self.buffer[19] << 1 |
                                     self.buffer[20]) << 9 & 0x07ff
            Rcu_data['channel15'] = (self.buffer[20] >> 2 | self.buffer[21] << 6) & 0x07ff
            Rcu_data['channel16'] = (self.buffer[21] >> 5 | self.buffer[22] << 3) & 0x07ff
            Rcu_data['flag'] = self.buffer[23]

            del self.buffer[:25]

    def __encode(self):
        """打包数据"""
        packet = bytes((
            0x0f,
            self.send_data['channel1'] & 0xff,
            (self.send_data['channel1'] >> 8 | self.send_data['channel2'] << 3) & 0xff,
            (self.send_data['channel2'] >> 5 | self.send_data['channel3'] << 6) & 0xff,
            (self.send_data['channel3'] >> 2) & 0xff,
            (self.send_data['channel3'] >> 10 | self.send_data['channel4'] << 1) & 0xff,
            (self.send_data['channel4'] >> 7 | self.send_data['channel5'] << 4) & 0xff,
            (self.send_data['channel5'] >> 4 | self.send_data['channel6'] << 7) & 0xff,
            (self.send_data['channel6'] >> 1) & 0xff,
            (self.send_data['channel6'] >> 9 | self.send_data['channel7'] << 2) & 0xff,
            (self.send_data['channel7'] >> 6 | self.send_data['channel8'] << 5) & 0xff,
            (self.send_data['channel8'] >> 3) & 0xff,
            self.send_data['channel9'] & 0xff,
            (self.send_data['channel9'] >> 8 | self.send_data['channel10'] << 3) & 0xff,
            (self.send_data['channel10'] >> 5 | self.send_data['channel11'] << 6) & 0xff,
            (self.send_data['channel11'] >> 2) & 0xff,
            (self.send_data['channel11'] >> 10 | self.send_data['channel12'] << 1) & 0xff,
            (self.send_data['channel12'] >> 7 | self.send_data['channel13'] << 4) & 0xff,
            (self.send_data['channel13'] >> 4 | self.send_data['channel14'] << 7) & 0xff,
            (self.send_data['channel14'] >> 1) & 0xff,
            (self.send_data['channel14'] >> 9 | self.send_data['channel15'] << 2) & 0xff,
            (self.send_data['channel15'] >> 6 | self.send_data['channel16'] << 5) & 0xff,
            (self.send_data['channel16'] >> 3) & 0xff,
            self.send_data['flag'],
            0x04))

        return packet

    @staticmethod
    def __backup():
        for last in list(Rcu_last_data.keys()):
            for now in list(Rcu_data.keys()):
                if now == last:
                    Rcu_last_data[last] = Rcu_data[now]

    @staticmethod
    def __limit_change_rate(data, last_data, limit_change_rate, control_time):
        if (data - last_data) > limit_change_rate / 1000 * 672 * control_time:
            return int(last_data + limit_change_rate / 1000 * 672 * control_time)
        elif (data - last_data) < -limit_change_rate / 1000 * 672 * control_time:
            return int(last_data - limit_change_rate / 1000 * 672 * control_time)
        else:
            return int(data)
