import time
import serial
import struct
from PySide2.QtGui import QPolygon
import socket


class GroundControlStation:
    def __init__(self, com):
        self.heart_beat = {'timestamp': 0.0, 'buffer_err': 0}
        self.command = {'timestamp': 0.0, 'setting': 0, 'desired_heading': 0.0, 'desired_speed': 0.0,
                        'desired_latitude': 0.0, 'desired_longitude': 0.0, 'rudder': 0, 'thrust': 0, 'ignition': 0,
                        'buffer_err': 0}
        self.gcs = serial.Serial(com, 57600, timeout=0, write_timeout=0)
        self.buffer = []
        self.errors = 0

    def g_run(self, usv):
        self.receive_decode(usv)
        self.send_status(usv)

    def receive_decode(self, usv):
        self.buffer += self.gcs.read()  # 超时时间要设置 ##############

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
                self.errors += 1
                continue

            packet_id = self.buffer[1]
            packet_data = self.buffer[5:packet_length]

            if packet_id == 0 and packet_data_length == 8:  # 心跳包
                self.heart_beat['timestamp'] = time.time()
                self.heart_beat['buffer_err'] = self.errors
                del self.buffer[:packet_length]
                continue

            elif (packet_data[0] | packet_data[1] << 8) != usv.control.pid['id']:  # 数据过滤
                del self.buffer[:packet_length]
                continue

            elif packet_id == 1 and packet_data_length == 40:  # Command
                command = struct.unpack('<Bffddhhb', bytes(packet_data[10:packet_data_length]))
                self.command['timestamp'] = time.time()
                self.command['setting'] = command[0]
                self.command['desired_heading'] = command[1]
                self.command['desired_speed'] = command[2]
                self.command['desired_latitude'] = command[3]
                self.command['desired_longitude'] = command[4]
                self.command['desired_rudder'] = command[5]
                self.command['desired_thrust'] = command[6]
                self.command['ignition'] = command[7]
                del self.buffer[:packet_length]
                continue

            elif packet_id == 2 and packet_data_length == 10:  # 参数请求
                data_bytes = struct.pack('<Hdfffffffff', usv.control.pid['id'], time.time(),
                                         usv.control.pid['heading_p'], usv.control.pid['heading_i'],
                                         usv.control.pid['heading_d'], usv.control.pid['speed_p'],
                                         usv.control.pid['speed_i'], usv.control.pid['speed_d'],
                                         usv.control.pid['position_p'], usv.control.pid['position_i'],
                                         usv.control.pid['position_d'])
                crc16 = calculate_crc16_ccitt(data_bytes, len(data_bytes))
                header_bytes = calculate_header_bytes(17, 46, crc16)  # 参数包id=17，包长46
                send_data = header_bytes + data_bytes
                self.gcs.write(send_data)
                del self.buffer[:packet_length]
                continue

            elif packet_id == 3 and packet_data_length == 46:  # 设置请求
                # 解析PID参数
                pid = struct.unpack('<fffffffff', bytes(packet_data[10:packet_data_length]))
                # 保存PID
                usv.control.pid['heading_p'] = pid[0]
                usv.control.pid['heading_i'] = pid[1]
                usv.control.pid['heading_d'] = pid[2]
                usv.control.pid['speed_p'] = pid[3]
                usv.control.pid['speed_i'] = pid[4]
                usv.control.pid['speed_d'] = pid[5]
                usv.control.pid['position_p'] = pid[6]
                usv.control.pid['position_i'] = pid[7]
                usv.control.pid['position_d'] = pid[8]
                # 设置回应
                data_bytes = struct.pack('<HdB', usv.control.pid['id'], time.time(), 0xff)
                crc16 = calculate_crc16_ccitt(data_bytes, len(data_bytes))
                header_bytes = calculate_header_bytes(18, 11, crc16)  # 回应包id=18，包长11
                send_data = header_bytes + data_bytes
                self.gcs.write(send_data)
                del self.buffer[:packet_length]
                continue

            elif packet_id == 4:
                data_type = packet_data[10]
                if packet_data_length == 11 and data_type == 0:
                    pass
                if packet_data_length == 31 and data_type == 1:
                    pass
                if packet_data_length == 11 and data_type == 2:
                    pass
                # 返回成功信号
                data_bytes = struct.pack('<HdB', usv.control.pid['id'], time.time(), 0xff)
                crc16 = calculate_crc16_ccitt(data_bytes, len(data_bytes))
                header_bytes = calculate_header_bytes(19, 11, crc16)  # 回应包id=19，包长11
                send_data = header_bytes + data_bytes
                self.gcs.write(send_data)
                del self.buffer[:packet_length]
                continue

    def send_status(self, usv):
        data_bytes = struct.pack('<HdBdddffffffffffffHHffhhb', usv.control.pid['id'], time.time(), usv.control.status,
                                 usv.navigation.data['location']['latitude'],
                                 usv.navigation.data['location']['longitude'],
                                 usv.navigation.data['location']['altitude'], usv.navigation.data['posture']['roll'],
                                 usv.navigation.data['posture']['pitch'], usv.navigation.data['posture']['yaw'],
                                 usv.navigation.data['velocity']['north'], usv.navigation.data['velocity']['east'],
                                 usv.navigation.data['velocity']['down'], usv.navigation.data['gyroscope']['X'],
                                 usv.navigation.data['gyroscope']['Y'], usv.navigation.data['gyroscope']['Z'],
                                 usv.navigation.data['accelerometer']['X'], usv.navigation.data['accelerometer']['Y'],
                                 usv.navigation.data['accelerometer']['Z'], usv.navigation.data['system_status'],
                                 usv.navigation.data['filter_status'], usv.control.depth, usv.control.battery,
                                 usv.control.data['rudder'], usv.control.data['thrust'], usv.control.data['ignition'])
        crc16 = calculate_crc16_ccitt(data_bytes, len(data_bytes))
        header_bytes = calculate_header_bytes(16, 100, crc16)  # 回应包id=16，包长100
        send_data = header_bytes + data_bytes
        self.gcs.write(send_data)


def calculate_header_bytes(id, length, crc16):
    header_list = [0, id, length, crc16 & 0xff, crc16 // 0x100]
    header_list[0] = calculate_header_lrc(header_list)
    return struct.pack('<BBBBB', header_list[0], header_list[1], header_list[2], header_list[3], header_list[4])


def calculate_header_lrc(data_list):
    return ((((data_list[1] + data_list[2] + data_list[3] + data_list[4]) & 0xff) ^ 0xff) + 1) & 0xff


def calculate_crc16_ccitt(data_list, length):
    crc16_ccitt = 0xffff
    crc16_ccitt_list = [0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
                        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
                        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
                        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
                        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
                        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
                        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
                        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
                        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
                        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
                        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
                        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
                        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
                        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
                        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
                        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
                        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
                        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
                        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
                        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
                        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
                        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
                        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
                        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
                        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
                        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
                        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
                        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
                        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
                        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
                        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0]
    for i in range(length):
        crc16_ccitt = ((crc16_ccitt << 8 & 0xffff) ^ crc16_ccitt_list[
            ((crc16_ccitt >> 8) & 0xffff) ^ data_list[i]]) & 0xffff
    return crc16_ccitt
