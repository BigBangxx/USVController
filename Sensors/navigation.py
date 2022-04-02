import math
import struct

import serial
import time
import socket

from Protocols.FDILink import FDILink
from Protocols.Wit import Wit
from Protocols.Rion import Rion
from Protocols.Anpp import Anpp
from Communicator.ground_control_station import calculate_header_lrc, calculate_crc16_ccitt


class Navigation:
    """组合导航模块"""

    def __init__(self, usv):
        self.data = {'location': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0, 'hACC': 100, 'vACC': 100},
                     'posture': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                     'velocity': {'speed': 0.0, 'north': 0.0, 'east': 0.0, 'down': 0.0, 'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                     'gyroscope': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                     'accelerometer': {'X': 0.0, 'Y': 0.0, 'Z': 0.0}, 'timestamp': 0.0, 'errors': 0, 'system_status': 0,
                     'filter_status': 0}
        self.buffer = bytearray()
        self.packet = []

        if usv.settings.navigation_type == 'airsim':
            ip_port = ('0.0.0.0', usv.settings.airsim_port)
            self.navigation_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.navigation_socket.bind(ip_port)
            self.navigation_socket.setblocking(False)
        else:
            self.navigation = serial.Serial(usv.settings.navigation_com, usv.settings.navigation_baudrate, timeout=0,
                                            write_timeout=0)

    def n_run(self, usv):
        if usv.settings.navigation_type == 'wit':
            self.wit_decode()
        elif usv.settings.navigation_type == 'airsim':
            self.airsim_decode(usv)
        elif usv.settings.navigation_type == 'rion':
            self.rion_decode()
        elif usv.settings.navigation_type == 'fdi':
            self.fdi_device_decode()
        elif usv.settings.navigation_type == 'anpp':
            self.anpp_decode()

    def anpp_decode(self):
        self.buffer += self.navigation.read(self.navigation.in_waiting)
        while True:
            packet_id, packet_data, errors = Anpp.decode(self.buffer, self.data["errors"])
            if packet_id is None:
                break
            elif packet_id == 20:
                data_field = struct.unpack('<HHIIdddffffffffffffffff', packet_data)
                self.data['posture']['roll'] = data_field[14]  # 输出姿态角，单位rad
                self.data['posture']['pitch'] = data_field[15]
                self.data['posture']['yaw'] = data_field[16]
                self.data['gyroscope']['X'] = data_field[17]  # 输出角速率，单位rad/s
                self.data['gyroscope']['Y'] = data_field[18]
                self.data['gyroscope']['Z'] = data_field[19]
                self.data['accelerometer']['X'] = data_field[10]  # 输出加速度，单位m/s2
                self.data['accelerometer']['Y'] = data_field[11]
                self.data['accelerometer']['Z'] = data_field[12]
                self.data['location']['latitude'] = data_field[4]  # 输出滤波补偿后得经纬度（单位：度）高度（单位：米）
                self.data['location']['longitude'] = data_field[5]
                self.data['location']['altitude'] = data_field[6]
                self.data['velocity']['north'] = data_field[7]  # 输出三维速度单位m/s
                self.data['velocity']['east'] = data_field[8]
                self.data['velocity']['down'] = data_field[9]
                self.data['system_status'] = data_field[0]
                self.data['filter_status'] = data_field[1]

            self.data['timestamp'] = time.time()

    def fdi_device_decode(self):
        self.buffer += self.navigation.read(self.navigation.in_waiting)

        while True:
            packet_id, packet_data, errors = FDILink.decode(self.buffer, self.data["errors"])
            if packet_id is None:
                break
            if packet_id == 0x5C:
                data_field = struct.unpack('<dddff', packet_data)
                self.data['location']['latitude'] = data_field[0]
                self.data['location']['longitude'] = data_field[1]
                self.data['location']['altitude'] = data_field[2]
                self.data['location']['hACC'] = data_field[3]
                self.data['location']['vACC'] = data_field[4]
            elif packet_id == 0x5F:
                data_field = struct.unpack('<fff', packet_data)
                self.data['velocity']['north'] = data_field[0]
                self.data['velocity']['east'] = data_field[1]
                self.data['velocity']['down'] = data_field[2]
            elif packet_id == 0x60:
                data_field = struct.unpack('<fff', packet_data)
                self.data['velocity']['X'] = data_field[0]
                self.data['velocity']['Y'] = data_field[1]
                self.data['velocity']['Z'] = data_field[2]
            elif packet_id == 0x61:
                data_field = struct.unpack('<fff', packet_data)
                self.data['accelerometer']['X'] = data_field[0]
                self.data['accelerometer']['Y'] = data_field[1]
                self.data['accelerometer']['Z'] = data_field[2]
            elif packet_id == 0x63:
                data_field = struct.unpack('<fff', packet_data)
                self.data['posture']['roll'] = data_field[0]
                self.data['posture']['pitch'] = data_field[1]
                self.data['posture']['yaw'] = data_field[2]
            elif packet_id == 0x66:
                data_field = struct.unpack('<fff', packet_data)
                self.data['gyroscope']['X'] = data_field[0]
                self.data['gyroscope']['Y'] = data_field[1]
                self.data['gyroscope']['Z'] = data_field[2]

    def wit_decode(self):
        """接受维特组合导航数据并解析"""
        self.buffer += self.navigation.read(self.navigation.in_waiting)
        while True:
            packet_id, packet_data, errors = Wit.decode(self.buffer, self.data["errors"])
            if packet_id is None:
                break
            elif packet_id == 0x57:
                # 位置
                position = struct.unpack('<ii', packet_data)
                self.data['location']['latitude'] = math.radians(
                    position[0] // 10000000 + position[0] % 10000000 / 100000 / 60)
                self.data['location']['longitude'] = math.radians(
                    position[1] // 10000000 + position[1] % 10000000 / 100000 / 60)
            elif packet_id == 0x53:
                # 姿态
                posture = struct.unpack('<hhhh', packet_data)
                self.data['posture']['pitch'] = posture[0] / 32768.0 * math.pi
                self.data['posture']['roll'] = posture[1] / 32768.0 * math.pi
                self.data['posture']['yaw'] = -posture[2] / 32768.0 * math.pi
                if self.data['posture']['yaw'] < 0:
                    self.data['posture']['yaw'] += 2 * math.pi

            elif packet_id == 0x58:
                # 速度
                velocity = struct.unpack('<hhI', packet_data)
                self.data['velocity']['speed'] = velocity[2] / 1000 / 3.6

            elif packet_id == 0x52:
                # 角加速度
                gyroscope = struct.unpack('<hhhh', packet_data)
                self.data['gyroscope']['Y'] = math.radians(gyroscope[0] / 32768.0 * 2000)
                self.data['gyroscope']['X'] = math.radians(gyroscope[1] / 32768.0 * 2000)
                self.data['gyroscope']['Z'] = math.radians(-gyroscope[2] / 32768.0 * 2000)

            elif packet_id == 0x51:
                # 加速度
                accelerometer = struct.unpack('<hhhh', packet_data)
                self.data['accelerometer']['Y'] = accelerometer[0] / 32768.0 * 16 * 9.8
                self.data['accelerometer']['X'] = accelerometer[1] / 32768.0 * 16 * 9.8
                self.data['accelerometer']['Z'] = -accelerometer[2] / 32768.0 * 16 * 9.8

            self.data['timestamp'] = time.time()

    def rion_decode(self):
        self.buffer += self.navigation.read(self.navigation.in_waiting)
        while True:
            packet_id, packet_data, errors = Rion.decode(self.buffer, self.data["errors"])
            if packet_id is None:
                break
            elif packet_id == 0x90:
                data_field = struct.unpack('>fffffffffdddfff', packet_data)
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
