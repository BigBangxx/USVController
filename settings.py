class Settings:
    """配置项"""

    def __init__(self):

        self.usv_id = 0x0000

        # 串口配置
        self.sbus_com = 'COM50'
        self.navigation_com = 'COM51'
        self.gcs_com = 'COM52'

        # 外设配置
        self.navigation_type = 'wit'
        self.navigation_baudrate = '115200'
