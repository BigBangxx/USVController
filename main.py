import threading

from Communicator.remote_control_unit import RemoteControlUnit
from Sensors.navigation import Navigation
from GNC.control import Control
from Utilities.settings import Settings
from Communicator.ground_control_station import GroundControlStation
from Utilities.mission import Mission
from Utilities.log import Log


class UsvControl:
    """程序入口"""

    def __init__(self):
        self.settings = Settings()
        self.futaba = RemoteControlUnit(self.settings.sbus_com)
        self.navigation = Navigation(self)
        self.control = Control(self)
        self.gcs = GroundControlStation(self)
        self.mission = Mission(self.settings.usv_id)
        self.log = Log(self.settings.usv_id)

    def ms10_run(self):
        self.futaba.rcu_run(self)
        self.control.c_run(self)
        self.navigation.n_run(self)
        self.gcs.g_run(self)
        self.futaba.backup_data()
        self.log.write_log(self)
        timer_10 = threading.Timer(0.01, self.ms10_run, )
        timer_10.start()

    def ms1000_run(self):
        print(self.futaba.receive_data)
        print(self.navigation.data)
        print(self.control.data)
        print(self.control.pid)
        print(self.gcs.heart_beat)
        print(self.gcs.command)
        timer_1000 = threading.Timer(1, self.ms1000_run, )
        timer_1000.start()


# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    usv1 = UsvControl()
    usv1.ms10_run()
    usv1.ms1000_run()
