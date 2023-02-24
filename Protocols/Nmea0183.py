"""
Copyright (C) 2023 by 熊俊峰. All rights reserved. Email: xjf_whut@qq.com

Nmea0183 message structure: $payload(id,data1,data2,...,datan)*checksum(AB)\r\n
"""
import struct

_NMEA0183_MESSAGE_HEADER_LENGTH = 6  # 除去负载的消息长度


def encode_message(payload: bytes):
    return b'$' + payload + b'*' + struct.pack('<H', _calculate_nmea0183_checksum(payload)) + b'\r\n'


def decode_buffer(buffered: bytearray, errors: int):
    """
    :return: payload, errors
    """
    payload = None

    while len(buffered) >= _NMEA0183_MESSAGE_HEADER_LENGTH:
        # 定位消息头
        if buffered[0] != ord('$'):
            del buffered[0]
            continue
        # 如果缓存长度小于消息长度，则继续接收数据
        try:
            payload_length = buffered.index(ord('*')) - 1
        except ValueError:
            break
        message_length = _NMEA0183_MESSAGE_HEADER_LENGTH + payload_length
        if len(buffered) < message_length:
            break
        # 校验异或校验和
        if (buffered[payload_length + 2] | buffered[payload_length + 3] << 8) != _calculate_nmea0183_checksum(buffered[1:payload_length + 1]):
            del buffered[0]
            errors += 1
            continue
        # 从缓存中解析一包消息
        payload = buffered[1:payload_length + 1]
        # 从缓存中删除解析的消息
        del buffered[:message_length]
        break

    return payload, errors


def _calculate_nmea0183_checksum(data: bytes):
    u8_checksum = 0
    for item in data:
        u8_checksum ^= item

    a = u8_checksum >> 4  # 高字节，放在低地址
    b = u8_checksum & 0x0f  # 低字节，放在高地址

    if a > 0x09:
        a += ord('A') - 0x0a
    else:
        a += ord('0')
    if b > 0x09:
        b += ord('A') - 0x0a
    else:
        b += ord('0')

    return a + (b << 8)
