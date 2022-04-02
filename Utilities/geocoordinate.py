import math


class GeoCoordinate:
    __geo_coordinate_EARTH_MEAN_RADIUS = 6371.0072  # Km

    def __init__(self, latitude, longitude):
        self.latitude = latitude  # 弧度
        self.longitude = longitude

    def distance2(self, other_point):  # m
        dlat = other_point.latitude - self.latitude
        dlon = other_point.longitude - self.longitude
        haversine_dlat = math.sin(dlat / 2.0)
        haversine_dlat *= haversine_dlat
        haversine_dlon = math.sin(dlon / 2.0)
        haversine_dlon *= haversine_dlon
        y = haversine_dlat + math.cos(self.latitude) * math.cos(other_point.latitude) * haversine_dlon
        x = 2 * math.asin(math.sqrt(y))

        return x * GeoCoordinate.__geo_coordinate_EARTH_MEAN_RADIUS * 1000

    def azimuth2(self, other_point):  # radians
        dlon = other_point.longitude - self.longitude
        y = math.sin(dlon) * math.cos(other_point.latitude)
        x = math.cos(self.latitude) * math.sin(other_point.latitude) - math.sin(self.latitude) * math.cos(
            other_point.latitude) * math.cos(dlon)
        azimuth = math.degrees(math.atan2(y, x)) + 360
        fraction, whole = math.modf(azimuth)
        return math.radians(((whole + 360) % 360) + fraction)

    def at_distance_and_azimuth(self, distance, azimuth):
        ratio = (distance / (self.__geo_coordinate_EARTH_MEAN_RADIUS * 1000))
        latitude = math.asin(
            math.sin(self.latitude) * math.cos(ratio) + math.cos(self.latitude) * math.sin(ratio) * math.cos(azimuth))
        longitude = self.longitude + math.atan2(math.sin(azimuth) * math.sin(ratio) * math.cos(self.latitude),
                                                math.cos(ratio) - math.sin(self.latitude) * math.sin(latitude))
        if longitude > 180:
            longitude -= 360
        elif longitude < -180:
            longitude += 360

        return GeoCoordinate(latitude, longitude)
