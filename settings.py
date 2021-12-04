import os


class Settings:
    """配置项"""

    def __init__(self):
        if not os.path.exists("AppData"):
            os.mkdir("AppData")
        self.usv_id = 0
        self.los_distance = 1.5

        # 串口配置
        self.sbus_com = 'COM50'
        self.navigation_com = 'COM51'
        self.gcs_com = 'COM52'

        # 外设配置
        # 导航
        self.navigation_type = 'wit'
        self.navigation_baudrate = '115200'
        # 地面站
        self.gcs_disconnect_time_allow = 3
        self.gcs_waypoint_err = 1
