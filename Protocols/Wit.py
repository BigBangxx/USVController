class Wit:
    """Wit相关方法"""

    __frame_length = 11

    @staticmethod
    def encode(packet_data: bytes, packet_id: int):
        head_date = bytearray((0x55, packet_id)) + packet_data
        check_sum = bytes((Wit.__check_sum(head_date),))
        return head_date + check_sum

    @staticmethod
    def decode(buffer: bytearray, errors: int):
        packet_id = None
        packet_data = None
        while len(buffer) >= Wit.__frame_length:
            if buffer[0] != 0x55:
                del buffer[0]
                continue
            if buffer[10] != Wit.__check_sum(buffer):
                del buffer[0]
                errors += 1
                continue
            packet_id = buffer[1]
            packet_data = buffer[2:10]
            del buffer[:Wit.__frame_length]
            break
        return packet_id, packet_data, errors

    @staticmethod
    def __check_sum(data: bytearray):
        return (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8] + data[
            9]) & 0xff
