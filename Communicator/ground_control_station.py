import time
import socket

import serial
import struct

from Protocols.Anpp import Anpp
from Utilities.global_data import Gcs_heart_beat, Gcs_command, Nav_data, Ctrl_data, Pid, settings, mission


class GroundControlStation:
    def __init__(self):
        self.waypoints = []
        self.communication_type = settings.gcs_communication_type
        usv_ip_port = ('0.0.0.0', 0)
        self.server_ip_port = (settings.gcs_server_ip, settings.gcs_server_port)
        if self.communication_type == 'udp':
            self.gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.gcs_socket.bind(usv_ip_port)
            self.gcs_socket.setblocking(False)
        elif self.communication_type == 'tcp':
            self.gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.gcs_socket.bind(usv_ip_port)
            self.gcs_socket.connect(self.server_ip_port)
            data_bytes = struct.pack('<H', settings.usv_id)
            send_data = Anpp.encode(data_bytes, 255)  # 参数包id=17，包长46
            self.gcs_socket.send(send_data)
            self.gcs_socket.setblocking(False)
        else:
            self.gcs_serial = serial.Serial(settings.gcs_com, 57600, timeout=0, write_timeout=0)
        self.buffer = bytearray()
        self.errors = 0

    def __del__(self):
        if self.communication_type == 'udp':
            self.gcs_socket.close()

    def g_run(self):
        self.__receive_decode()
        self.__send_status()

    def __receive_decode(self):
        if self.communication_type == 'udp':
            try:
                data, send_address = self.gcs_socket.recvfrom(1024)
            except BlockingIOError:
                data = b""
            self.buffer += data
        elif self.communication_type == 'tcp':
            try:
                data = self.gcs_socket.recv(1024)
            except BlockingIOError:
                data = b""
            self.buffer += data
        else:
            self.buffer += self.gcs_serial.read(self.gcs_serial.in_waiting)

        while True:
            packet_id, packet_data, self.errors = Anpp.decode(self.buffer, self.errors)
            if packet_id is None:
                break

            packet_data_length = len(packet_data)

            if packet_id == 0 and packet_data_length == 8:  # 心跳包
                command = struct.unpack('<d', bytes(packet_data))
                Gcs_heart_beat['timestamp'] = time.time()
                Gcs_heart_beat['delay'] = time.time() - command[0]
                Gcs_heart_beat['buffer_err'] = self.errors
            elif (packet_data[0] | packet_data[1] << 8) != settings.usv_id:  # 数据过滤
                continue
            elif packet_id == 1 and packet_data_length == 50:  # Command
                command = struct.unpack('<Bffddhhbhfhh', bytes(packet_data[10:packet_data_length]))
                Gcs_command['timestamp'] = time.time()
                Gcs_command['setting'] = command[0]
                Gcs_command['desired_heading'] = command[1]
                Gcs_command['desired_speed'] = command[2]
                Gcs_command['desired_latitude'] = command[3]
                Gcs_command['desired_longitude'] = command[4]
                Gcs_command['desired_rudder'] = command[5]
                Gcs_command['desired_thrust'] = command[6]
                Gcs_command['ignition'] = command[7]
                Gcs_command['angle'] = command[8]
                Gcs_command['distance'] = command[9]
                Gcs_command['ship_num'] = command[10]
                if command[11] > Gcs_command['index_sum']:
                    Gcs_command['index_sum'] = command[11]

            elif packet_id == 2 and packet_data_length == 10:  # 参数请求
                data_bytes = struct.pack('<Hdffffffffffff', settings.usv_id, time.time(), Pid['heading_p'],
                                         Pid['heading_i'], Pid['heading_d'], Pid['speed_p'], Pid['speed_i'],
                                         Pid['speed_d'], Pid['position_p'], Pid['position_i'], Pid['position_d'],
                                         Pid['position_p2'], Pid['position_i2'], Pid['position_d2'])
                send_data = Anpp.encode(data_bytes, 17)  # 参数包id=17，包长58
                if self.communication_type == 'udp':
                    self.gcs_socket.sendto(send_data, self.server_ip_port)
                elif self.communication_type == 'tcp':
                    self.gcs_socket.send(send_data)
                else:
                    self.gcs_serial.write(send_data)

            elif packet_id == 3 and packet_data_length == 58:  # 设置请求
                # 解析PID参数
                pid = struct.unpack('<ffffffffffff', bytes(packet_data[10:packet_data_length]))
                # 保存PID
                Pid['heading_p'] = pid[0]
                Pid['heading_i'] = pid[1]
                Pid['heading_d'] = pid[2]
                Pid['speed_p'] = pid[3]
                Pid['speed_i'] = pid[4]
                Pid['speed_d'] = pid[5]
                Pid['position_p'] = pid[6]
                Pid['position_i'] = pid[7]
                Pid['position_d'] = pid[8]
                Pid['position_p2'] = pid[9]
                Pid['position_i2'] = pid[10]
                Pid['position_d2'] = pid[11]
                settings.update_pid(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8], pid[9],
                                    pid[10], pid[11])
                # 设置回应
                data_bytes = struct.pack('<HdB', settings.usv_id, time.time(), 0xff)
                send_data = Anpp.encode(data_bytes, 18)  # 回应包id=18，包长11
                if self.communication_type == 'udp':
                    self.gcs_socket.sendto(send_data, self.server_ip_port)
                elif self.communication_type == 'tcp':
                    self.gcs_socket.send(send_data)
                else:
                    self.gcs_serial.write(send_data)

            elif packet_id == 4:  # 任务上传请求
                data_type = packet_data[10]
                if packet_data_length == 11 and data_type == 0:
                    self.waypoints.clear()
                if packet_data_length == 31 and data_type == 1:
                    way_point = struct.unpack('<ddf', bytes(packet_data[11:packet_data_length]))
                    is_same = False
                    if len(self.waypoints) != 0:
                        if self.waypoints[-1][0] != way_point[0] and self.waypoints[-1][1] != way_point[1] and \
                                self.waypoints[-1][2] != way_point[2]:
                            is_same = True
                    if not is_same:
                        self.waypoints.append(way_point)
                if packet_data_length == 11 and data_type == 2:
                    mission.write(self.waypoints)
                # 返回成功信号
                data_bytes = struct.pack('<HdB', settings.usv_id, time.time(), 0xff)
                send_data = Anpp.encode(data_bytes, 19)  # 回应包id=19，包长11
                if self.communication_type == 'udp':
                    self.gcs_socket.sendto(send_data, self.server_ip_port)
                elif self.communication_type == 'tcp':
                    self.gcs_socket.send(send_data)
                else:
                    self.gcs_serial.write(send_data)

    def __send_status(self):
        data_bytes = struct.pack('<HdBdddffffffffffffHHffhhbH', settings.usv_id, time.time(), Ctrl_data['status'],
                                 Nav_data['location']['latitude'], Nav_data['location']['longitude'],
                                 Nav_data['location']['altitude'], Nav_data['posture']['roll'],
                                 Nav_data['posture']['pitch'], Nav_data['posture']['yaw'],
                                 Nav_data['velocity']['north'], Nav_data['velocity']['east'],
                                 Nav_data['velocity']['down'], Nav_data['gyroscope']['X'], Nav_data['gyroscope']['Y'],
                                 Nav_data['gyroscope']['Z'], Nav_data['accelerometer']['X'],
                                 Nav_data['accelerometer']['Y'], Nav_data['accelerometer']['Z'],
                                 Nav_data['system_status'], Nav_data['filter_status'], 0, 0, Ctrl_data['rudder'],
                                 Ctrl_data['thrust'], Ctrl_data['ignition'], Gcs_command['index'])
        send_data = Anpp.encode(data_bytes, 16)  # 回应包id=16，包长102
        if self.communication_type == 'udp':
            self.gcs_socket.sendto(send_data, self.server_ip_port)
        elif self.communication_type == 'tcp':
            self.gcs_socket.send(send_data)
        else:
            self.gcs_serial.write(send_data)
