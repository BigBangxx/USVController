import math
import serial
import time

from SelfBuiltModul.func import degrees_to_radians, to_signed_number


class Navigation:
    """组合导航模块"""

    def __init__(self, com, navigation_type, baudrate):
        self.data = {'location': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
                     'posture': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                     'velocity': {'speed': 0.0, 'north': 0.0, 'east': 0.0, 'down': 0.0},
                     'gyroscope': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                     'accelerometer': {'X': 0.0, 'Y': 0.0, 'Z': 0.0}, 'timestamp': 0.0, 'errors': 0}
        self.navigation = serial.Serial(com, baudrate, timeout=0, write_timeout=0)
        self.buffer = []
        self.packet = []
        self.navigation_type = navigation_type

    def n_run(self):
        if self.navigation_type == 'wit':
            self.wit_decode()

    def wit_decode(self):
        """接受维特组合导航数据并解析"""

        self.buffer += self.navigation.read(55)

        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue
            if self.buffer[10] != self.wit_check_sum(self.buffer):
                del self.buffer[0]
                self.data['errors'] += 1
                continue

            if self.buffer[1] == 0x57:
                # 位置
                latitude = to_signed_number(
                    self.buffer[9] << 24 | self.buffer[8] << 16 | self.buffer[7] << 8 | self.buffer[6], 4)
                longitude = to_signed_number(
                    self.buffer[5] << 24 | self.buffer[4] << 16 | self.buffer[3] << 8 | self.buffer[2], 4)
                self.data['location']['latitude'] = degrees_to_radians(
                    latitude // 10000000 + latitude % 10000000 / 100000 / 60)
                self.data['location']['longitude'] = degrees_to_radians(
                    longitude // 10000000 + latitude % 10000000 / 100000 / 60)

            elif self.buffer[1] == 0x53:
                # 姿态
                self.data['posture']['pitch'] = to_signed_number(self.buffer[3] << 8 | self.buffer[2],
                                                                 2) / 32768.0 * math.pi
                self.data['posture']['roll'] = to_signed_number(self.buffer[5] << 8 | self.buffer[4],
                                                                2) / 32768.0 * math.pi
                self.data['posture']['yaw'] = -to_signed_number(self.buffer[7] << 8 | self.buffer[6],
                                                                2) / 32768.0 * math.pi
                if self.data['posture']['yaw'] < 0:
                    self.data['posture']['yaw'] += 2 * math.pi

            elif self.buffer[1] == 0x58:
                # 速度
                self.data['velocity']['speed'] = (self.buffer[9] << 24 | self.buffer[8] << 16 | self.buffer[7] << 8 |
                                                  self.buffer[6]) / 1000 / 3.6

            elif self.buffer[1] == 0x52:
                # 角加速度
                self.data['gyroscope']['Y'] = degrees_to_radians(to_signed_number
                                                                 (self.buffer[3] << 8 | self.buffer[2],
                                                                  2) / 32768.0 * 2000)
                self.data['gyroscope']['X'] = degrees_to_radians(to_signed_number
                                                                 (self.buffer[5] << 8 | self.buffer[4],
                                                                  2) / 32768.0 * 2000)
                self.data['gyroscope']['Z'] = -degrees_to_radians(to_signed_number
                                                                  (self.buffer[7] << 8 | self.buffer[6],
                                                                   2) / 32768.0 * 2000)

            elif self.buffer[1] == 0x51:
                # 加速度
                self.data['accelerometer']['Y'] = to_signed_number(self.buffer[3] << 8 | self.buffer[2],
                                                                   2) / 32768.0 * 16 * 9.8
                self.data['accelerometer']['X'] = to_signed_number(self.buffer[5] << 8 | self.buffer[4],
                                                                   2) / 32768.0 * 16 * 9.8
                self.data['accelerometer']['Z'] = -to_signed_number(self.buffer[7] << 8 | self.buffer[6],
                                                                    2) / 32768.0 * 16 * 9.8

            self.data['timestamp'] = time.time()

            self.packet = self.buffer[1:10]

            del self.buffer[:11]
            # break  组合导航100hz，控制周期100hz，不需要break

    def encode(self):
        """打包维特导航数据"""
        pass

    @staticmethod
    def wit_check_sum(data):
        return (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8] + data[
            9]) & 0xff
