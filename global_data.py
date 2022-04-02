# PID parameter
Pid = {'id': 0, 'heading_p': 0, 'heading_i': 0, 'heading_d': 0, 'speed_p': 0, 'speed_i': 0, 'speed_d': 0,
       'position_p': 0,
       'position_i': 0, 'position_d': 0}

# Control parameter
Ctrl_data = {'mode': 'lock', 'thrust': 0, 'rudder': 0, 'ignition': 0}

# Remoter Control Unit
Rcu_data = {'channel1': 0, 'channel2': 0, 'channel3': 0, 'channel4': 0, 'channel5': 0, 'channel6': 0, 'channel7': 0,
            'channel8': 0, 'channel9': 0, 'channel10': 0, 'channel11': 0, 'channel12': 0, 'channel13': 0,
            'channel14': 0, 'channel15': 0, 'channel16': 0, 'flag': 0, 'error': 0, }

# Navigation
Nav_data = {'location': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0, 'hACC': 100, 'vACC': 100},
            'posture': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'velocity': {'speed': 0.0, 'north': 0.0, 'east': 0.0, 'down': 0.0, 'X': 0.0, 'Y': 0.0, 'Z': 0.0},
            'gyroscope': {'X': 0.0, 'Y': 0.0, 'Z': 0.0}, 'accelerometer': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
            'timestamp': 0.0, 'errors': 0, 'system_status': 0, 'filter_status': 0}

# Ground Control Station
Gcs_heart_beat = {'timestamp': 0.0, 'buffer_err': 0}
Gcs_command = {'timestamp': 0.0, 'setting': 0, 'desired_heading': 0.0, 'desired_speed': 0.0, 'desired_latitude': 0.0,
               'desired_longitude': 0.0, 'desired_rudder': 0, 'desired_thrust': 0, 'ignition': 0, 'buffer_err': 0}
