import math

from Utilities.geocoordinate import GeoCoordinate


def calculate_los_angle(qgc_previous: GeoCoordinate, qgc_current: GeoCoordinate, qgc_next: GeoCoordinate,
                        los_distance=1.5):
    # 直线方向 degrees
    previous_2_next_angle = qgc_previous.azimuth2(qgc_next)
    # 计算垂点
    angle_between = previous_2_next_angle - qgc_previous.azimuth2(qgc_current)
    # 距离
    previous_2_current_distance = qgc_previous.distance2(qgc_current)
    current_2_line_distance = abs(previous_2_current_distance * math.sin(angle_between))
    # GPS
    previous_2_perpendicular_distance = previous_2_current_distance * math.cos(angle_between)
    qgc_perpendicular = qgc_previous.at_distance_and_azimuth(previous_2_perpendicular_distance, previous_2_next_angle)

    # 计算交点 修正
    if los_distance < current_2_line_distance:
        los_distance = current_2_line_distance
    distance_added = math.sqrt(los_distance * los_distance - current_2_line_distance * current_2_line_distance)

    # 交点
    qgc_cross = qgc_perpendicular.at_distance_and_azimuth(distance_added, previous_2_next_angle)

    # 计算los
    previous_2_next_latitude = qgc_next.latitude - qgc_previous.latitude
    previous_2_cross_latitude = qgc_cross.latitude - qgc_previous.latitude
    next_2_cross_latitude = qgc_cross.latitude - qgc_next.latitude
    # 后向
    if previous_2_next_latitude * previous_2_cross_latitude < 0:
        return qgc_current.azimuth2(qgc_previous)
    # 前向
    if previous_2_next_latitude * next_2_cross_latitude > 0:
        return qgc_current.azimuth2(qgc_next)
    # 中间
    return qgc_current.azimuth2(qgc_cross)
