import socket
import struct
import time

import serial
import array
from ground_control_station import calculate_header_bytes, calculate_crc16_ccitt


class RemoteControlUnit:
    """遥控器相关属性及方法"""

    def __init__(self, com, ):
        self.receive_data = {'channel1': 0, 'channel2': 0, 'channel3': 0, 'channel4': 0, 'channel5': 0, 'channel6': 0,
                             'channel7': 0, 'channel8': 0, 'channel9': 0, 'channel10': 0, 'channel11': 0,
                             'channel12': 0, 'channel13': 0, 'channel14': 0, 'channel15': 0, 'channel16': 0, 'flag': 0,
                             'error': 0, }
        self.last_data = self.receive_data.copy()
        self.send_data = self.receive_data.copy()
        self.sbus = serial.Serial(com, 100000, serial.EIGHTBITS, serial.PARITY_EVEN, serial.STOPBITS_TWO, 0, False,
                                  False, 0)
        self.buffer = []
        self.packet = array.array('B')

    def rcu_run(self, usv):
        self.decode()
        self.send_command(usv)

    def send_command(self, usv):
        if usv.settings.navigation_type != "airsim":
            self.send_data = self.receive_data.copy()
            self.send_data['channel1'] = int(
                1024 + ((usv.control.data['rudder'] + usv.control.data['thrust']) * 0.672))  # 左电机
            self.send_data['channel3'] = int(
                1024 + ((usv.control.data['thrust'] - usv.control.data['rudder']) * 0.672))  # 右电机

            self.encode()
            self.sbus.write(self.packet.tobytes())
        else:
            data_bytes = struct.pack('<Hdhh', usv.settings.usv_id, time.time(), usv.control.data['rudder'],
                                     usv.control.data['thrust'])
            crc16 = calculate_crc16_ccitt(data_bytes, len(data_bytes))
            header_bytes = calculate_header_bytes(0, 14, crc16)  # 回应包id=0，包长14
            send_data = header_bytes + data_bytes
            send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            send.bind((usv.settings.usv_ip,usv.settings.airsim_send_port))
            send.sendto(send_data, (usv.settings.airsim_ip, usv.settings.airsim_receive_port))
            send.close()

    def decode(self, ):
        """读取数据并解包数据"""
        self.buffer += self.sbus.read(25)

        while len(self.buffer) >= 25:
            if self.buffer[0] != 0x0f:
                del self.buffer[0]
                continue

            if self.buffer[24] != 0x04 and self.buffer[24] != 0x14 and self.buffer[24] != 0x24 and self.buffer[
                24] != 0x34:
                del self.buffer[0]
                self.receive_data['error'] += 1
                continue

            self.receive_data['channel1'] = (self.buffer[1] >> 0 | self.buffer[2] << 8) & 0x07ff
            self.receive_data['channel2'] = (self.buffer[2] >> 3 | self.buffer[3] << 5) & 0x07ff
            self.receive_data['channel3'] = (self.buffer[3] >> 6 | self.buffer[4] << 2 |
                                             self.buffer[5] << 10) & 0x07ff
            self.receive_data['channel4'] = (self.buffer[5] >> 1 | self.buffer[6] << 7) & 0x07ff
            self.receive_data['channel5'] = (self.buffer[6] >> 4 | self.buffer[7] << 4) & 0x07ff
            self.receive_data['channel6'] = (self.buffer[7] >> 7 | self.buffer[8] << 1 |
                                             self.buffer[9] << 9) & 0x07ff
            self.receive_data['channel7'] = (self.buffer[9] >> 2 | self.buffer[10] << 6) & 0x07ff
            self.receive_data['channel8'] = (self.buffer[10] >> 5 | self.buffer[11] << 3) & 0x07ff
            self.receive_data['channel9'] = (self.buffer[12] >> 0 | self.buffer[13] << 8) & 0x07ff
            self.receive_data['channel10'] = (self.buffer[13] >> 3 | self.buffer[14] << 5) & 0x07ff
            self.receive_data['channel11'] = (self.buffer[14] >> 6 | self.buffer[15] << 2 |
                                              self.buffer[16] << 10) & 0x07ff
            self.receive_data['channel12'] = (self.buffer[16] >> 1 | self.buffer[17] << 7) & 0x07ff
            self.receive_data['channel13'] = (self.buffer[17] >> 4 | self.buffer[18] << 4) & 0x07ff
            self.receive_data['channel14'] = (self.buffer[18] >> 7 | self.buffer[19] << 1 |
                                              self.buffer[20]) << 9 & 0x07ff
            self.receive_data['channel15'] = (self.buffer[20] >> 2 | self.buffer[21] << 6) & 0x07ff
            self.receive_data['channel16'] = (self.buffer[21] >> 5 | self.buffer[22] << 3) & 0x07ff
            self.receive_data['flag'] = self.buffer[23]

            del self.buffer[:25]
            break

    def encode(self):
        """打包数据"""
        self.packet = array.array('B')
        self.packet.append(0x0f)
        self.packet.append(self.send_data['channel1'] & 0xff)
        self.packet.append((self.send_data['channel1'] >> 8 | self.send_data['channel2'] << 3) & 0xff)
        self.packet.append((self.send_data['channel2'] >> 5 | self.send_data['channel3'] << 6) & 0xff)
        self.packet.append((self.send_data['channel3'] >> 2) & 0xff)
        self.packet.append((self.send_data['channel3'] >> 10 | self.send_data['channel4'] << 1) & 0xff)
        self.packet.append((self.send_data['channel4'] >> 7 | self.send_data['channel5'] << 4) & 0xff)
        self.packet.append((self.send_data['channel5'] >> 4 | self.send_data['channel6'] << 7) & 0xff)
        self.packet.append((self.send_data['channel6'] >> 1) & 0xff)
        self.packet.append((self.send_data['channel6'] >> 9 | self.send_data['channel7'] << 2) & 0xff)
        self.packet.append((self.send_data['channel7'] >> 6 | self.send_data['channel8'] << 5) & 0xff)
        self.packet.append((self.send_data['channel8'] >> 3) & 0xff)
        self.packet.append(self.send_data['channel9'] & 0xff)
        self.packet.append((self.send_data['channel9'] >> 8 | self.send_data['channel10'] << 3) & 0xff)
        self.packet.append((self.send_data['channel10'] >> 5 | self.send_data['channel11'] << 6) & 0xff)
        self.packet.append((self.send_data['channel11'] >> 2) & 0xff)
        self.packet.append((self.send_data['channel11'] >> 10 | self.send_data['channel12'] << 1) & 0xff)
        self.packet.append((self.send_data['channel12'] >> 7 | self.send_data['channel13'] << 4) & 0xff)
        self.packet.append((self.send_data['channel13'] >> 4 | self.send_data['channel14'] << 7) & 0xff)
        self.packet.append((self.send_data['channel14'] >> 1) & 0xff)
        self.packet.append((self.send_data['channel14'] >> 9 | self.send_data['channel15'] << 2) & 0xff)
        self.packet.append((self.send_data['channel15'] >> 6 | self.send_data['channel16'] << 5) & 0xff)
        self.packet.append((self.send_data['channel16'] >> 3) & 0xff)
        self.packet.append(self.send_data['flag'])
        self.packet.append(0x04)

    def backup_data(self):
        self.last_data = self.receive_data.copy()
