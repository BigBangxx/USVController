import math
import struct

import serial
import time
import socket

from Protocols.FDILink import FDILink
from Protocols.Wit import Wit
from Protocols.Rion import Rion
from Protocols.Anpp import Anpp
from Protocols import Nmea0183
from Utilities.global_data import Nav_data, Ctrl_data, settings


class Navigation:
    """组合导航模块"""

    def __init__(self):

        self.buffer = bytearray()

        if settings.navigation_type == 'airsim':
            ip_port = ('0.0.0.0', 0)
            self.navigation_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.navigation_socket.bind(ip_port)
            self.navigation_socket.setblocking(False)
        else:
            self.navigation = serial.Serial(settings.navigation_com, settings.navigation_baudrate, timeout=0,
                                            write_timeout=0)

    def n_run(self):
        if settings.navigation_type == 'wit':
            self.__wit_decode()
        elif settings.navigation_type == 'airsim':
            self.__airsim_send_command()
            self.__airsim_decode()
        elif settings.navigation_type == 'rion':
            self.__rion_decode()
        elif settings.navigation_type == 'fdi':
            self.__fdi_device_decode()
        elif settings.navigation_type == 'anpp':
            self.__anpp_decode()
        elif settings.navigation_type == 'nmea0183':
            self.__nmea0183_decode()

    def __anpp_decode(self):
        try:
            self.buffer += self.navigation.read(self.navigation.in_waiting)
        except serial.SerialException:
            print('导航串口异常,尝试重置')
            try:
                self.navigation = serial.Serial(settings.navigation_com, settings.navigation_baudrate, timeout=0,
                                                write_timeout=0)
                print('重置成功')
            except serial.SerialException:
                print('重置失败')

        while True:
            packet_id, packet_data, errors = Anpp.decode(self.buffer, Nav_data["errors"])
            if packet_id is None:
                break
            elif packet_id == 20:
                data_field = struct.unpack('<HHIIdddffffffffffffffff', packet_data)
                Nav_data['posture']['roll'] = data_field[14]  # 输出姿态角，单位rad
                Nav_data['posture']['pitch'] = data_field[15]
                Nav_data['posture']['yaw'] = data_field[16]
                Nav_data['gyroscope']['X'] = data_field[17]  # 输出角速率，单位rad/s
                Nav_data['gyroscope']['Y'] = data_field[18]
                Nav_data['gyroscope']['Z'] = data_field[19]
                Nav_data['accelerometer']['X'] = data_field[10]  # 输出加速度，单位m/s2
                Nav_data['accelerometer']['Y'] = data_field[11]
                Nav_data['accelerometer']['Z'] = data_field[12]
                Nav_data['location']['latitude'] = data_field[4]  # 输出滤波补偿后得经纬度（单位：rad）高度（单位：米）
                Nav_data['location']['longitude'] = data_field[5]
                Nav_data['location']['altitude'] = data_field[6]
                Nav_data['velocity']['north'] = data_field[7]  # 输出三维速度单位m/s
                Nav_data['velocity']['east'] = data_field[8]
                Nav_data['velocity']['down'] = data_field[9]
                Nav_data['system_status'] = data_field[0]
                Nav_data['filter_status'] = data_field[1]
                Nav_data['velocity']['speed'] = math.hypot(Nav_data['velocity']['north'], Nav_data['velocity']['east'])

            Nav_data['timestamp'] = time.time()

    def __fdi_device_decode(self):
        try:
            self.buffer += self.navigation.read(self.navigation.in_waiting)
        except serial.SerialException:
            print('导航串口异常,尝试重置')
            try:
                self.navigation = serial.Serial(settings.navigation_com, settings.navigation_baudrate, timeout=0,
                                                write_timeout=0)
                print('重置成功')
            except serial.SerialException:
                print('重置失败')

        while True:
            packet_id, packet_data, errors = FDILink.decode(self.buffer, Nav_data["errors"])
            if packet_id is None:
                break
            if packet_id == 0x5C:
                data_field = struct.unpack('<dddff', packet_data)
                Nav_data['location']['latitude'] = data_field[0]
                Nav_data['location']['longitude'] = data_field[1]
                Nav_data['location']['altitude'] = data_field[2]
                Nav_data['location']['hACC'] = data_field[3]
                Nav_data['location']['vACC'] = data_field[4]
            elif packet_id == 0x5F:
                data_field = struct.unpack('<fff', packet_data)
                Nav_data['velocity']['north'] = data_field[0]
                Nav_data['velocity']['east'] = data_field[1]
                Nav_data['velocity']['down'] = data_field[2]
            elif packet_id == 0x60:
                data_field = struct.unpack('<fff', packet_data)
                Nav_data['velocity']['X'] = data_field[0]
                Nav_data['velocity']['Y'] = data_field[1]
                Nav_data['velocity']['Z'] = data_field[2]
                Nav_data['velocity']['speed'] = math.hypot(Nav_data['velocity']['X'], Nav_data['velocity']['Y'])

            elif packet_id == 0x61:
                data_field = struct.unpack('<fff', packet_data)
                Nav_data['accelerometer']['X'] = data_field[0]
                Nav_data['accelerometer']['Y'] = data_field[1]
                Nav_data['accelerometer']['Z'] = data_field[2]
            elif packet_id == 0x63:
                data_field = struct.unpack('<fff', packet_data)
                Nav_data['posture']['roll'] = data_field[0]
                Nav_data['posture']['pitch'] = data_field[1]
                Nav_data['posture']['yaw'] = data_field[2]
            elif packet_id == 0x66:
                data_field = struct.unpack('<fff', packet_data)
                Nav_data['gyroscope']['X'] = data_field[0]
                Nav_data['gyroscope']['Y'] = data_field[1]
                Nav_data['gyroscope']['Z'] = data_field[2]

    def __wit_decode(self):
        """接受维特组合导航数据并解析"""
        try:
            self.buffer += self.navigation.read(self.navigation.in_waiting)
        except serial.SerialException:
            print('导航串口异常,尝试重置')
            try:
                self.navigation = serial.Serial(settings.navigation_com, settings.navigation_baudrate, timeout=0,
                                                write_timeout=0)
                print('重置成功')
            except serial.SerialException:
                print('重置失败')
        while True:
            packet_id, packet_data, errors = Wit.decode(self.buffer, Nav_data["errors"])
            if packet_id is None:
                break
            elif packet_id == 0x57:
                # 位置
                position = struct.unpack('<ii', packet_data)
                Nav_data['location']['latitude'] = math.radians(
                    position[0] // 10000000 + position[0] % 10000000 / 100000 / 60)
                Nav_data['location']['longitude'] = math.radians(
                    position[1] // 10000000 + position[1] % 10000000 / 100000 / 60)
            elif packet_id == 0x53:
                # 姿态
                posture = struct.unpack('<hhhh', packet_data)
                Nav_data['posture']['pitch'] = posture[0] / 32768.0 * math.pi
                Nav_data['posture']['roll'] = posture[1] / 32768.0 * math.pi
                Nav_data['posture']['yaw'] = -posture[2] / 32768.0 * math.pi
                if Nav_data['posture']['yaw'] < 0:
                    Nav_data['posture']['yaw'] += 2 * math.pi

            elif packet_id == 0x58:
                # 速度
                velocity = struct.unpack('<hhI', packet_data)
                Nav_data['velocity']['speed'] = velocity[2] / 1000 / 3.6  # wit的是速率

            elif packet_id == 0x52:
                # 角加速度
                gyroscope = struct.unpack('<hhhh', packet_data)
                Nav_data['gyroscope']['Y'] = math.radians(gyroscope[0] / 32768.0 * 2000)
                Nav_data['gyroscope']['X'] = math.radians(gyroscope[1] / 32768.0 * 2000)
                Nav_data['gyroscope']['Z'] = math.radians(-gyroscope[2] / 32768.0 * 2000)

            elif packet_id == 0x51:
                # 加速度
                accelerometer = struct.unpack('<hhhh', packet_data)
                Nav_data['accelerometer']['Y'] = accelerometer[0] / 32768.0 * 16 * 9.8
                Nav_data['accelerometer']['X'] = accelerometer[1] / 32768.0 * 16 * 9.8
                Nav_data['accelerometer']['Z'] = -accelerometer[2] / 32768.0 * 16 * 9.8

            Nav_data['timestamp'] = time.time()

    def __rion_decode(self):
        try:
            self.buffer += self.navigation.read(self.navigation.in_waiting)
        except serial.SerialException:
            print('导航串口异常,尝试重置')
            try:
                self.navigation = serial.Serial(settings.navigation_com, settings.navigation_baudrate, timeout=0,
                                                write_timeout=0)
                print('重置成功')
            except serial.SerialException:
                print('重置失败')
        while True:
            packet_id, packet_data, errors = Rion.decode(self.buffer, Nav_data["errors"])
            if packet_id is None:
                break
            elif packet_id == 0x90:
                data_field = struct.unpack('<ffffffffffffffffBBffdddffff', packet_data)
                Nav_data['posture']['roll'] = data_field[0]  # 输出姿态角，单位rad
                Nav_data['posture']['pitch'] = data_field[1]
                yaw = data_field[2]
                if yaw < 0:
                    yaw += 2 * math.pi
                Nav_data['posture']['yaw'] = yaw
                Nav_data['gyroscope']['X'] = data_field[3]  # 输出角速率，单位rad/s
                Nav_data['gyroscope']['Y'] = data_field[4]
                Nav_data['gyroscope']['Z'] = data_field[5]
                Nav_data['accelerometer']['X'] = data_field[6]  # 输出加速度，单位m/s2
                Nav_data['accelerometer']['Y'] = data_field[7]
                Nav_data['accelerometer']['Z'] = data_field[8]
                Nav_data['location']['latitude'] = math.radians(data_field[20])  # 输出滤波补偿后得经纬度（单位：度）高度（单位：米）
                Nav_data['location']['longitude'] = math.radians(data_field[21])
                Nav_data['location']['altitude'] = data_field[22]
                Nav_data['velocity']['north'] = data_field[23]  # 输出三维速度单位m/s
                Nav_data['velocity']['east'] = data_field[24]
                Nav_data['velocity']['down'] = data_field[25]
                Nav_data['velocity']['speed'] = math.hypot(Nav_data['velocity']['north'], Nav_data['velocity']['east'])

            Nav_data['timestamp'] = time.time()

    def __nmea0183_decode(self):
        try:
            self.buffer += self.navigation.read(self.navigation.in_waiting)
        except serial.SerialException:
            print('导航串口异常,尝试重置')
            try:
                self.navigation = serial.Serial(settings.navigation_com, settings.navigation_baudrate, timeout=0,
                                                write_timeout=0)
                print('重置成功')
            except serial.SerialException:
                print('重置失败')
        while True:
            packet_data, Nav_data["errors"] = Nmea0183.decode_buffer(self.buffer, Nav_data["errors"])
            if packet_data is None:
                break

            string = packet_data.decode()
            fields = string.split(",")
            if len(fields) != 23:
                break
            if "KSXT" in fields[0]:
                try:
                    Nav_data['timestamp'] = time.time()
                    Nav_data['location']['latitude'] = math.radians(float(fields[3]))
                    Nav_data['location']['longitude'] = math.radians(float(fields[2]))
                    Nav_data['location']['altitude'] = float(fields[4])
                    Nav_data['posture']['yaw'] = math.radians(float(fields[5]))
                    Nav_data['posture']['pitch'] = math.radians(float(fields[6]))
                    Nav_data['posture']['roll'] = math.radians(float(fields[9]))
                    if int(fields[10]) == 2 or int(fields[10]) == 3:
                        Nav_data['location']['hACC'] = 5
                        Nav_data['location']['vACC'] = 5
                    else:
                        Nav_data['location']['hACC'] = 100
                        Nav_data['location']['vACC'] = 100
                    Nav_data['velocity']['east'] = float(fields[17]) / 3.6
                    Nav_data['velocity']['north'] = float(fields[18]) / 3.6
                    Nav_data['velocity']['X'] = Nav_data['velocity']['north'] * math.cos(Nav_data['posture']['yaw']) + \
                                                Nav_data['velocity']['east'] * math.sin(Nav_data['posture']['yaw'])
                    Nav_data['velocity']['Y'] = Nav_data['velocity']['north'] * math.sin(Nav_data['posture']['yaw']) + \
                                                Nav_data['velocity']['east'] * math.cos(Nav_data['posture']['yaw'])
                    Nav_data['velocity']['down'] = float(fields[19]) / 3.6
                    Nav_data['velocity']['speed'] = math.hypot(Nav_data['velocity']['north'],
                                                               Nav_data['velocity']['east'])
                except ValueError:
                    pass

    def __airsim_decode(self):
        try:
            while True:
                self.buffer += self.navigation_socket.recv(1024)
        except BlockingIOError:
            pass

        while True:
            packet_id, packet_data, errors = Anpp.decode(self.buffer, Nav_data["errors"])

            if packet_id is None:
                break

            packet_data_length = len(packet_data)

            if (packet_data[0] | packet_data[1] << 8) != settings.usv_id:  # 数据过滤
                continue
            elif packet_id == 10 and packet_data_length == 82:  # Command
                command = struct.unpack('<dddffffffffffff', bytes(packet_data[10:packet_data_length]))
                Nav_data['timestamp'] = time.time()
                Nav_data['location']['latitude'] = command[0]
                Nav_data['location']['longitude'] = command[1]
                Nav_data['location']['altitude'] = command[2]
                Nav_data['posture']['pitch'] = command[3]
                Nav_data['posture']['roll'] = command[4]
                Nav_data['posture']['yaw'] = command[5]
                Nav_data['velocity']['north'] = command[6]
                Nav_data['velocity']['east'] = command[7]
                Nav_data['velocity']['down'] = command[8]
                Nav_data['gyroscope']['Y'] = command[9]
                Nav_data['gyroscope']['X'] = command[10]
                Nav_data['gyroscope']['Z'] = command[11]
                Nav_data['accelerometer']['Y'] = command[12]
                Nav_data['accelerometer']['X'] = command[13]
                Nav_data['accelerometer']['Z'] = command[14]
                Nav_data['velocity']['speed'] = math.hypot(Nav_data['velocity']['north'], Nav_data['velocity']['east'])

    def __airsim_send_command(self):
        data_bytes = struct.pack('<Hdhh', settings.usv_id, time.time(), Ctrl_data['rudder'], Ctrl_data['thrust'])
        send_data = Anpp.encode(data_bytes, 0)  # 回应包id=0，包长14
        self.navigation_socket.sendto(send_data, (settings.airsim_ip, settings.airsim_port))
