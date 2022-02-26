import array
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
        if usv.settings.navigation_type == 'wit' or usv.settings.navigation_type == 'rion':
            self.navigation = serial.Serial(usv.settings.navigation_com, usv.settings.navigation_baudrate, timeout=0,
                                            write_timeout=0)
        if usv.settings.navigation_type == 'airsim':
            ip_port = ('0.0.0.0', usv.settings.airsim_port)
            self.navigation_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.navigation_socket.bind(ip_port)
            self.navigation_socket.setblocking(False)

    def n_run(self, usv):
        if usv.settings.navigation_type == 'wit':
            self.wit_decode()
        elif usv.settings.navigation_type == 'airsim':
            self.airsim_decode(usv)
        elif usv.settings.navigation_type == 'rion':
            self.rion_decode()

    def wit_decode(self):
        """接受维特组合导航数据并解析"""

        self.buffer += self.navigation.read(self.navigation.in_waiting)

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

    def encode(self):
        """打包维特导航数据"""
        pass

    def rion_decode(self):
        self.buffer += self.navigation.read(self.navigation.in_waiting)
        while len(self.buffer) >= 5:
            # Find the start of a packet by scanning for a valid data head
            if (self.buffer[0] != 0xff) or (self.buffer[1] != 0x02):
                del self.buffer[0]
                continue
            # Calculate the packet_data length and compare it with data packets
            packet_data_length = self.buffer[3] << 8 | self.buffer[4]
            packet_length = 8 + packet_data_length
            if len(self.buffer) < packet_length:
                break
            # Check crc16_kermit of the buffer
            if (self.buffer[packet_length - 3] << 8 | self.buffer[packet_length - 2]) != self.__calculate_crc16_kermit(
                    bytes(self.buffer[2:(packet_length - 3)])):
                del self.buffer[0]
                self.data['errors'] += 1
                continue
            # Decode one packet from the buffer
            if self.buffer[2] == 0x90:
                data_field = struct.unpack('<fffffffffdddffff', bytes(self.buffer[5:5 + packet_data_length]))
                self.data['posture']['roll'] = data_field[0]  # 输出姿态角，单位rad
                self.data['posture']['pitch'] = data_field[1]
                yaw = data_field[2]
                if yaw < 0:
                    yaw += 2 * math.pi
                self.data['posture']['yaw'] = yaw
                self.data['gyroscope']['X'] = data_field[3]  # 输出角速率，单位rad/s
                self.data['gyroscope']['Y'] = data_field[4]
                self.data['gyroscope']['Z'] = data_field[5]
                self.data['accelerometer']['X'] = data_field[6]  # 输出加速度，单位m/s2
                self.data['accelerometer']['Y'] = data_field[7]
                self.data['accelerometer']['Z'] = data_field[8]
                self.data['location']['latitude'] = data_field[9]  # 输出滤波补偿后得经纬度（单位：度）高度（单位：米）
                self.data['location']['longitude'] = data_field[10]
                self.data['location']['altitude'] = data_field[11]
                self.data['velocity']['north'] = data_field[12]  # 输出三维速度单位m/s
                self.data['velocity']['east'] = data_field[13]
                self.data['velocity']['down'] = data_field[14]

            self.data['timestamp'] = time.time()
            del self.buffer[:packet_length]

    def airsim_decode(self, usv):
        try:
            data, send_address = self.navigation_socket.recvfrom(1024)
        except BlockingIOError:
            data = ""
            send_address = ()

        if send_address == (usv.settings.airsim_ip, usv.settings.airsim_port):
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

    @staticmethod
    def wit_check_sum(data):
        return (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8] + data[
            9]) & 0xff

    @staticmethod
    def __calculate_crc16_kermit(data: bytes):
        crc16 = 0
        crc16_kermit_array = array.array('H', [
            0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
            0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
            0x1081, 0x108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
            0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
            0x2102, 0x308b, 0x210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
            0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
            0x3183, 0x200a, 0x1291, 0x318, 0x77a7, 0x662e, 0x54b5, 0x453c,
            0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
            0x4204, 0x538d, 0x6116, 0x709f, 0x420, 0x15a9, 0x2732, 0x36bb,
            0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
            0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x528, 0x37b3, 0x263a,
            0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
            0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x630, 0x17b9,
            0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
            0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x738,
            0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
            0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
            0x840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
            0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
            0x18c1, 0x948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
            0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
            0x2942, 0x38cb, 0xa50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
            0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
            0x39c3, 0x284a, 0x1ad1, 0xb58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
            0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
            0x4a44, 0x5bcd, 0x6956, 0x78df, 0xc60, 0x1de9, 0x2f72, 0x3efb,
            0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
            0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0xd68, 0x3ff3, 0x2e7a,
            0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
            0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0xe70, 0x1ff9,
            0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
            0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0xf78
        ])
        for item in data:
            crc16 = (crc16 >> 8 ^ crc16_kermit_array[crc16 & 0xff ^ item]) & 0xffff
        return crc16
