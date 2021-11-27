class Settings:
    """配置项"""

    def __init__(self):
        # 串口配置
        self.sbus_com = 'COM50'
        self.navigation_com = 'COM51'

        # 外设配置
        self.navigation_type = 'wit'
        self.navigation_baudrate = '115200'
