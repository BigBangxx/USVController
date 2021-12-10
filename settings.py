import os
import configparser


class Settings:
    """配置项"""

    def __init__(self):
        if not os.path.exists("AppData"):
            os.mkdir("AppData")
        if not os.path.exists("AppData/settings.ini"):
            config = configparser.ConfigParser()

            config.add_section('usv')
            config.set('usv', 'usv_id', '0')
            config.set('usv', 'usv_ip', '192.2.5.53')
            config.set('usv', 'los_distance', '1.5')

            config.add_section('rcu')
            config.set('rcu', 'sbus_com', 'COM50')

            config.add_section('navigation')
            config.set('navigation', 'navigation_type', 'airsim')
            config.set('navigation', 'navigation_com', 'COM51')
            config.set('navigation', 'navigation_baudrate', '115200')
            config.set('navigation', 'airsim_ip', '192.168.31.86')
            config.set('navigation', 'airsim_receive_port', '9876')
            config.set('navigation', 'airsim_send_port', '6789')

            config.add_section('gcs')
            config.set('gcs', 'gcs_com', 'COM52')
            config.set('gcs', 'gcs_disconnect_time_allow', '3')
            config.set('gcs', 'gcs_waypoint_err', '1')

            with open('AppData/settings.ini', 'w') as ini:
                config.write(ini)

        with open("AppData/settings.ini") as ini:
            config = configparser.ConfigParser()
            config.read_file(ini)

            self.usv_id = eval(config.get('usv', 'usv_id'))
            self.usv_ip = config.get('usv', 'usv_ip')
            self.los_distance = eval(config.get('usv', 'los_distance'))

            self.sbus_com = config.get('rcu', 'sbus_com')

            self.navigation_com = config.get('navigation', 'navigation_com')
            self.navigation_type = config.get('navigation', 'navigation_type')
            self.navigation_baudrate = eval(config.get('navigation', 'navigation_baudrate'))
            self.airsim_ip = config.get('navigation', 'airsim_ip')
            self.airsim_receive_port = eval(config.get('navigation', 'airsim_receive_port'))
            self.airsim_send_port = eval(config.get('navigation', 'airsim_send_port'))

            self.gcs_com = config.get('gcs', 'gcs_com')
            self.gcs_disconnect_time_allow = eval(config.get('gcs', 'gcs_disconnect_time_allow'))
            self.gcs_waypoint_err = eval(config.get('gcs', 'gcs_waypoint_err'))
