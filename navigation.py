import math
import struct

import serial
import time
import socket

from SelfBuiltModul.func import degrees_to_radians, to_signed_number
from ground_control_station import calculate_header_lrc, calculate_crc16_ccitt


class Navigation:
    """组合导航模块"""

    def __init__(self, usv):
        self.data = {'location': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
                     'posture': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                     'velocity': {'speed': 0.0, 'north': 0.0, 'east': 0.0, 'down': 0.0},
                     'gyroscope': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                     'accelerometer': {'X': 0.0, 'Y': 0.0, 'Z': 0.0}, 'timestamp': 0.0, 'errors': 0, 'system_status': 0,
                     'filter_status': 0}
        self.buffer = []
        self.packet = []
        if usv.settings.navigation_type == 'wit':
            self.navigation = serial.Serial(usv.settings.navigation_com, usv.settings.navigation_baudrate, timeout=0,
                                            write_timeout=0)
        if usv.settings.navigation_type == 'airsim':
            ip_port = (usv.settings.usv_ip, usv.settings.airsim_receive_port)
            self.receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive.bind(ip_port)
            self.receive.setblocking(False)

    def __del__(self):
        self.receive.close()

    def n_run(self, usv):
        if usv.settings.navigation_type == 'wit':
            self.wit_decode()
        elif usv.settings.navigation_type == 'airsim':
            self.airsim_decode(usv)

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

    def airsim_decode(self, usv):
        try:
            data, send_address = self.receive.recvfrom(87)
        except BlockingIOError:
            data = ""
            send_address = ()

        if send_address == (usv.settings.airsim_ip, usv.settings.airsim_send_port):
            self.buffer += data

        while len(self.buffer) >= 5:
            if self.buffer[0] != calculate_header_lrc(self.buffer):
                del self.buffer[0]
                continue

            packet_data_length = self.buffer[2]
            packet_length = packet_data_length + 5

            if len(self.buffer) < packet_length:
                break
            if (self.buffer[3] | self.buffer[4] << 8) != calculate_crc16_ccitt(self.buffer[5:], packet_data_length):
                del self.buffer[0]
                self.data['errors'] += 1
                continue

            packet_id = self.buffer[1]
            packet_data = self.buffer[5:packet_length]

            if (packet_data[0] | packet_data[1] << 8) != usv.settings.usv_id:  # 数据过滤
                del self.buffer[:packet_length]
                continue

            elif packet_id == 10 and packet_data_length == 82:  # Command
                command = struct.unpack('<dddffffffffffff', bytes(packet_data[10:packet_data_length]))
                self.data['timestamp'] = time.time()
                self.data['location']['latitude'] = command[0]
                self.data['location']['longitude'] = command[1]
                self.data['location']['altitude'] = command[2]
                self.data['posture']['pitch'] = command[3]
                self.data['posture']['roll'] = command[4]
                self.data['posture']['yaw'] = command[5]
                self.data['velocity']['north'] = command[6]
                self.data['velocity']['east'] = command[7]
                self.data['velocity']['down'] = command[8]
                self.data['gyroscope']['Y'] = command[9]
                self.data['gyroscope']['X'] = command[10]
                self.data['gyroscope']['Z'] = command[11]
                self.data['accelerometer']['Y'] = command[12]
                self.data['accelerometer']['X'] = command[13]
                self.data['accelerometer']['Z'] = command[14]
                del self.buffer[:packet_length]
                continue

    @staticmethod
    def wit_check_sum(data):
        return (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8] + data[
            9]) & 0xff
