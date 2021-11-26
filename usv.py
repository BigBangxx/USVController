import threading

from remote_control_unit import RemoteControlUnit
from control import Control
from settings import Settings


class UsvControl:
    """程序入口"""

    def __init__(self):
        self.settings = Settings()
        self.futaba = RemoteControlUnit(self.settings.sbus_com, )
        self.control = Control()

    def main_run(self):
        self.futaba.rcu_run(self)
        self.control.c_run(self)
        timer_10 = threading.Timer(0.005, self.main_run, )
        timer_10.start()


# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    usv1 = UsvControl()
    usv1.main_run()
