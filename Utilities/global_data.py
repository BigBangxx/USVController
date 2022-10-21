from Utilities.settings import Settings
from Utilities.mission import Mission

# Settings
settings = Settings()

# Mission
mission = Mission(settings.usv_id)

# PID parameter
Pid = {'id': settings.usv_id, 'heading_p': settings.heading_p, 'heading_i': settings.heading_i,
       'heading_d': settings.heading_d, 'speed_p': settings.speed_p, 'speed_i': settings.speed_i,
       'speed_d': settings.speed_d, 'position_p': settings.position_p, 'position_i': settings.position_i,
       'position_d': settings.position_d}

# Control parameter
Ctrl_data = {'status': 0, 'mode': 'lock', 'thrust': 0, 'rudder': 0, 'ignition': 0, 'desired_heading': 0.0,
             'desired_speed': 0.0}

# Remoter Control Unit
Rcu_data = {'channel1': 1024, 'channel2': 0, 'channel3': 1024, 'channel4': 0, 'channel5': 1024, 'channel6': 0,
            'channel7': 0, 'channel8': 0, 'channel9': 0, 'channel10': 0, 'channel11': 0, 'channel12': 0, 'channel13': 0,
            'channel14': 0, 'channel15': 0, 'channel16': 0, 'flag': 12, 'error': 0, }
Rcu_last_data = Rcu_data.copy()

# Navigation
Nav_data = {'location': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0, 'hACC': 100, 'vACC': 100},
            'posture': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'velocity': {'speed': 0.0, 'north': 0.0, 'east': 0.0, 'down': 0.0, 'X': 0.0, 'Y': 0.0, 'Z': 0.0},
            'gyroscope': {'X': 0.0, 'Y': 0.0, 'Z': 0.0}, 'accelerometer': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
            'timestamp': 0.0, 'errors': 0, 'system_status': 0, 'filter_status': 0}

# Ground Control Station
Gcs_heart_beat = {'timestamp': 0.0, 'buffer_err': 0}
Gcs_command = {'timestamp': 0.0, 'setting': 0, 'desired_heading': 0.0, 'desired_speed': 0.0, 'desired_latitude': 0.0,
               'desired_longitude': 0.0, 'desired_rudder': 0, 'desired_thrust': 0, 'ignition': 0, 'angle': 0,
               'distance': 0, 'ship_num': 0, 'index_sum': 0,  'average_index': 0, 'index': 0, 'buffer_err': 0}
Globals = {'Send_arrive_waypoint_packet': False}
