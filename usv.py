import threading

from remote_control_unit import RemoteControlUnit
from navigation import Navigation
from control import Control
from settings import Settings
from ground_control_station import GroundControlStation


class UsvControl:
    """程序入口"""

    def __init__(self):
        self.settings = Settings()
        self.futaba = RemoteControlUnit(self.settings.sbus_com)
        self.navigation = Navigation(self.settings.navigation_com, self.settings.navigation_type,
                                     self.settings.navigation_baudrate)
        self.control = Control()
        self.gcs = GroundControlStation(self.settings.gcs_com)

    def main_run(self):
        self.futaba.rcu_run(self)
        self.control.c_run(self)
        self.navigation.n_run()
        self.gcs.g_run(self)
        timer_10 = threading.Timer(0.01, self.main_run, )
        timer_10.start()


# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    usv1 = UsvControl()
    usv1.main_run()
