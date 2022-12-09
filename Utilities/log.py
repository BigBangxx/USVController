import time

from Utilities.global_data import Ctrl_data, Gcs_command, Nav_data, Pid, settings, data_of_formation


class Log:
    """记日志的类"""

    def __init__(self):
        self.file_name = f"AppData/{settings.usv_id}_{time.strftime('%Y_%m_%d_%H_%M')}.csv"
        with open(self.file_name, 'a') as log:
            log.write('Timestamp,mode,')
            log.write('Rudder,Thrust,Status,')
            log.write('Latitude,Longitude,Altitude,')
            log.write('Roll,Pitch,Yaw,')
            log.write('Speed,VelocityNorth,VelocityEast,VelocityDown,VelocityX,VelocityY,VelocityZ,')
            log.write('GyroscopeX,GyroscopeY,GyroscopeZ,')
            log.write('AccelerometerX,AccelerometerY,AccelerometerZ,')
            log.write('SystemStatus,FilterStatus,')
            log.write('command_Rudder,command_Thrust,')
            log.write('DesiredHeading,DesiredSpeed,DesiredLatitude,DesiredLongitude,')
            log.write('HeadingP,HeadingI,HeadingD,')
            log.write('SpeedP,SpeedI,SpeedD,')
            log.write('PositionP,PositionI,PositionD,')
            log.write('PositionP2,PositionI2,PositionD2,')
            log.write("formation_type,target,target_yaw,target_speed,target_speedX,target_speedY,")
            log.write('distance,distanceX,distanceY,speed,speedX,speedY,yaw\n')

    def write_log(self):
        with open(self.file_name, 'a') as log:
            log.write(f"{time.strftime('%H_%M_%S')}{str(round(time.time() % 1, 3))[1:]},{Ctrl_data['mode']},")

            log.write(f"{Ctrl_data['rudder']},{Ctrl_data['thrust']},{Ctrl_data['status']},")

            log.write(f"{Nav_data['location']['latitude']},{Nav_data['location']['longitude']},")
            log.write(f"{Nav_data['location']['altitude']},")

            log.write(f"{Nav_data['posture']['roll']},{Nav_data['posture']['pitch']},")
            log.write(f"{Nav_data['posture']['yaw']},")

            log.write(f"{Nav_data['velocity']['speed']},{Nav_data['velocity']['north']},")
            log.write(f"{Nav_data['velocity']['east']},{Nav_data['velocity']['down']},")
            log.write(f"{Nav_data['velocity']['X']},{Nav_data['velocity']['Y']},{Nav_data['velocity']['Z']},")

            log.write(f"{Nav_data['gyroscope']['X']},{Nav_data['gyroscope']['Y']},")
            log.write(f"{Nav_data['gyroscope']['Z']},")

            log.write(f"{Nav_data['accelerometer']['X']},{Nav_data['accelerometer']['Y']},")
            log.write(f"{Nav_data['accelerometer']['Z']},")

            log.write(f"{Nav_data['system_status']},{Nav_data['filter_status']},")

            log.write(f"{Gcs_command['desired_rudder']},{Gcs_command['desired_thrust']},")

            log.write(f"{Gcs_command['desired_heading']},{Gcs_command['desired_speed']},")
            log.write(f"{Gcs_command['desired_latitude']},{Gcs_command['desired_longitude']},")

            log.write(f"{Pid['heading_p']},{Pid['heading_i']},{Pid['heading_d']},")
            log.write(f"{Pid['speed_p']},{Pid['speed_i']},{Pid['speed_d']},")
            log.write(f"{Pid['position_p']},{Pid['position_i']},{Pid['position_d']},")
            log.write(f"{Pid['position_p2']},{Pid['position_i2']},{Pid['position_d2']},")

            log.write(f"{data_of_formation['type']},{data_of_formation['target']},")
            log.write(f"{data_of_formation['target_yaw']},{data_of_formation['target_speed']},")
            log.write(f"{data_of_formation['target_speedX']},{data_of_formation['target_speedY']},")
            log.write(f"{data_of_formation['distance']},{data_of_formation['distanceX']},")
            log.write(f"{data_of_formation['distanceY']},{data_of_formation['speed']},{data_of_formation['speedX']},")
            log.write(f"{data_of_formation['speedY']},{data_of_formation['yaw']}\n")
