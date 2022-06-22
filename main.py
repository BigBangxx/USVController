import os
import threading

from Communicator.remote_control_unit import RemoteControlUnit
from Communicator.ground_control_station import GroundControlStation
from Sensors.navigation import Navigation
from GNC.control import Control
from Utilities.log import Log

from Utilities.global_data import Rcu_data, Gcs_command, Gcs_heart_beat, Nav_data, Ctrl_data, Pid


class UsvControl:
    """程序入口"""

    def __init__(self):
        self.futaba = RemoteControlUnit()
        self.navigation = Navigation()
        self.gcs = GroundControlStation()
        self.control = Control()
        self.log = Log()
        pass

    def ms10_run(self):
        self.futaba.rcu_run()
        self.navigation.n_run()
        self.gcs.g_run()
        self.control.c_run()
        self.log.write_log()
        timer_10 = threading.Timer(0.01, self.ms10_run, )
        timer_10.start()

    def ms1000_run(self):
        os.system("clear")
        print(Rcu_data)
        print(Nav_data)
        print(Ctrl_data)
        print(Pid)
        print(Gcs_heart_beat)
        print(Gcs_command)
        timer_1000 = threading.Timer(1, self.ms1000_run, )
        timer_1000.start()


# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    usv1 = UsvControl()
    usv1.ms10_run()
    usv1.ms1000_run()
