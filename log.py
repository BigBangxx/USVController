import time


class Log:
    """记日志的类"""

    def __init__(self, usv_id):
        self.file_name = f"AppData/{usv_id}_{time.strftime('%Y_%m_%d_%H_%M')}.csv"
        with open(self.file_name, 'a') as log:
            log.write('Timestamp,mode,')
            log.write('Rudder,Thrust,Status,')
            log.write('Latitude,Longitude,Altitude,')
            log.write('Roll,Pitch,Yaw,')
            log.write('Speed,VelocityNorth,VelocityEast,VelocityDown,')
            log.write('GyroscopeX,GyroscopeY,GyroscopeZ,')
            log.write('AccelerometerX,AccelerometerY,AccelerometerZ,')
            log.write('SystemStatus,FilterStatus,')
            log.write('command_Rudder,command_Thrust,')
            log.write('DesiredHeading,DesiredSpeed,DesiredLatitude,DesiredLongitude,')
            log.write('HeadingP,HeadingI,HeadingD,')
            log.write('SpeedP,SpeedI,SpeedD,')
            log.write('PositionP,PositionI,PositionD\n')

    def write_log(self, usv):
        with open(self.file_name, 'a') as log:
            log.write(f"{time.strftime('%H_%M_%S')}{str(round(time.time() % 1, 3))[1:]},{usv.control.data['mode']},")

            log.write(f"{usv.control.data['rudder']},{usv.control.data['thrust']},{usv.control.status},")

            log.write(f"{usv.navigation.data['location']['latitude']},{usv.navigation.data['location']['longitude']},")
            log.write(f"{usv.navigation.data['location']['altitude']},")

            log.write(f"{usv.navigation.data['posture']['roll']},{usv.navigation.data['posture']['pitch']},")
            log.write(f"{usv.navigation.data['posture']['yaw']},")

            log.write(f"{usv.navigation.data['velocity']['speed']},{usv.navigation.data['velocity']['north']},")
            log.write(f"{usv.navigation.data['velocity']['east']},{usv.navigation.data['velocity']['down']},")

            log.write(f"{usv.navigation.data['gyroscope']['X']},{usv.navigation.data['gyroscope']['Y']},")
            log.write(f"{usv.navigation.data['gyroscope']['Z']},")

            log.write(f"{usv.navigation.data['accelerometer']['X']},{usv.navigation.data['accelerometer']['Y']},")
            log.write(f"{usv.navigation.data['accelerometer']['Z']},")

            log.write(f"{usv.navigation.data['system_status']},{usv.navigation.data['filter_status']},")

            log.write(f"{usv.gcs.command['desired_rudder']},{usv.gcs.command['desired_thrust']},")

            log.write(f"{usv.gcs.command['desired_heading']},{usv.gcs.command['desired_speed']},")
            log.write(f"{usv.gcs.command['desired_latitude']},{usv.gcs.command['desired_longitude']},")

            log.write(f"{usv.control.pid['heading_p']},{usv.control.pid['heading_i']},{usv.control.pid['heading_d']},")
            log.write(f"{usv.control.pid['speed_p']},{usv.control.pid['speed_i']},{usv.control.pid['speed_d']},")
            log.write(
                f"{usv.control.pid['position_p']},{usv.control.pid['position_i']},{usv.control.pid['position_d']}\n")
