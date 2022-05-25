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
            config.set('usv', 'los_distance', '1.5')
            config.set('usv', 'is_catamaran', 'True')
            config.set('usv', 'heading_p', '500')
            config.set('usv', 'heading_i', '0')
            config.set('usv', 'heading_d', '100')
            config.set('usv', 'speed_p', '0')
            config.set('usv', 'speed_i', '0')
            config.set('usv', 'speed_d', '0')
            config.set('usv', 'position_p', '0')
            config.set('usv', 'position_i', '0')
            config.set('usv', 'position_d', '0')
            config.set('usv', 'ctrl_cycle_time', '0.01')

            config.add_section('rcu')
            config.set('rcu', 'sbus_com', 'COM50')
            config.set('rcu', 'limit_sbus_change_rate', '1000')

            config.add_section('navigation')
            config.set('navigation', 'navigation_type', 'airsim')
            config.set('navigation', 'navigation_com', 'COM51')
            config.set('navigation', 'navigation_baudrate', '115200')
            config.set('navigation', 'airsim_ip', '192.168.31.86')
            config.set('navigation', 'airsim_port', '9876')

            config.add_section('gcs')
            config.set('gcs', 'communication_type', 'udp')
            config.set('gcs', 'gcs_com', 'COM52')
            config.set('gcs', 'server_ip', '')
            config.set('gcs', 'server_port', '')
            config.set('gcs', 'gcs_disconnect_time_allow', '3')
            config.set('gcs', 'gcs_waypoint_err', '1')

            with open('AppData/settings.ini', 'w') as ini:
                config.write(ini)

        with open("AppData/settings.ini") as ini:
            config = configparser.ConfigParser()
            config.read_file(ini)

            self.usv_id = eval(config.get('usv', 'usv_id'))
            self.los_distance = eval(config.get('usv', 'los_distance'))
            self.is_catamaran = eval(config.get('usv', 'is_catamaran'))
            self.heading_p = eval(config.get('usv', 'heading_p'))
            self.heading_i = eval(config.get('usv', 'heading_i'))
            self.heading_d = eval(config.get('usv', 'heading_d'))
            self.speed_p = eval(config.get('usv', 'speed_p'))
            self.speed_i = eval(config.get('usv', 'speed_i'))
            self.speed_d = eval(config.get('usv', 'speed_d'))
            self.position_p = eval(config.get('usv', 'position_p'))
            self.position_i = eval(config.get('usv', 'position_i'))
            self.position_d = eval(config.get('usv', 'position_d'))
            self.ctrl_cycle_time = eval(config.get('usv', 'ctrl_cycle_time'))

            self.sbus_com = config.get('rcu', 'sbus_com')
            self.limit_sbus_change_rate = eval(config.get('rcu', 'limit_sbus_change_rate'))

            self.navigation_com = config.get('navigation', 'navigation_com')
            self.navigation_type = config.get('navigation', 'navigation_type')
            self.navigation_baudrate = eval(config.get('navigation', 'navigation_baudrate'))
            self.airsim_ip = config.get('navigation', 'airsim_ip')
            self.airsim_port = eval(config.get('navigation', 'airsim_port'))

            self.gcs_communication_type = config.get('gcs', 'communication_type')
            self.gcs_com = config.get('gcs', 'gcs_com')
            self.gcs_server_ip = config.get('gcs', 'server_ip')
            self.gcs_server_port = eval(config.get('gcs', 'server_port'))
            self.gcs_disconnect_time_allow = eval(config.get('gcs', 'gcs_disconnect_time_allow'))
            self.gcs_waypoint_err = eval(config.get('gcs', 'gcs_waypoint_err'))

    @staticmethod
    def update_pid(hp, hi, hd, sp, si, sd, pp, pi, pd):
        with open("AppData/settings.ini", ) as ini:
            config = configparser.ConfigParser()
            config.read_file(ini)
            config.set('usv', 'heading_p', f'{hp}')
            config.set('usv', 'heading_i', f'{hi}')
            config.set('usv', 'heading_d', f'{hd}')
            config.set('usv', 'speed_p', f'{sp}')
            config.set('usv', 'speed_i', f'{si}')
            config.set('usv', 'speed_d', f'{sd}')
            config.set('usv', 'position_p', f'{pp}')
            config.set('usv', 'position_i', f'{pi}')
            config.set('usv', 'position_d', f'{pd}')
        with open("AppData/settings.ini", 'w') as ini:
            config.write(ini)
