import math


def degrees_to_radians(degrees):
    """degrees go to radians"""
    return degrees / 180 * math.pi


def to_signed_number(number, max_bytes):
    """Converts a binary number to a signed number"""
    if number < 2 ** (8 * max_bytes - 1):
        return number
    else:
        return number - 2 ** (8 * max_bytes)
