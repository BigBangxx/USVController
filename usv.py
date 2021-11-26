from remote_control_unit import RemoteControlUnit
from control import Control
from settings import Settings


class UsvControl:
    """程序入口"""

    def __init__(self):
        self.settings = Settings()
        self.futaba = RemoteControlUnit(self.settings.sbus_com)
        self.control = Control()

    def run(self):
        while True:
            self.futaba.run(self)
            self.control.run(self)


# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    usv1 = UsvControl()
    usv1.run()
