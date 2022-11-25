from controller import Robot
import numpy as np
import matplotlib.pyplot as plt


class BB_8(Robot):
    def __init__(self):
        super().__init__()
        self.timeStep = int(self.getBasicTimeStep())

        self.counterweight_gyro = self.getDevice("counterweight gyro")
        self.body_gyro = self.getDevice("body gyro")
        self.head_gyro = self.getDevice("head gyro")

        self.body_yaw_motor = self.getDevice("body yaw motor")
        self.body_pitch_motor = self.getDevice("body pitch motor")
        self.head_yaw_motor = self.getDevice("head yaw motor")

        self.counterweight_gyro.enable(self.timeStep)
        self.body_gyro.enable(self.timeStep)
        self.head_gyro.enable(self.timeStep)

        self.body_yaw_motor.setPosition(float('inf'))
        self.body_pitch_motor.setPosition(float('inf'))
        self.head_yaw_motor.setPosition(float('inf'))

        self.body_yaw_motor.setVelocity(float(0.0))
        self.body_pitch_motor.setVelocity(float(0.0))
        self.head_yaw_motor.setVelocity(float(0.0))

        self.init_time = self.getTime()

        self.yaw_speed = 0.0
        self.pitch_speed = 0.0
        self.max_speed = 4.0
        self.attenuation = 0.9

series_x = []
series_y = []
series_z = []
time_series = []
bb_8 = BB_8()
while bb_8.step(bb_8.timeStep) != -1:
    current_time = bb_8.getTime()
    bb_8.init_time = current_time
    val = bb_8.body_gyro.getValues()
    series_x.append(val[0])
    series_y.append(val[1])
    series_z.append(val[2])
    time_series.append(current_time)
    if current_time >= 15.0:
        plt.plot(time_series, series_x, 'r', label='X gyroscope')
        plt.plot(time_series, series_y, 'g', label='Y gyroscope')
        plt.plot(time_series, series_z, 'b', label='Z gyroscope')
        plt.legend()
        plt.show()
        with open('gyroscope_x.txt', 'w') as filehandle:
            filehandle.write(f'{series_x}\n')
        with open('gyroscope_y.txt', 'w') as filehandle:
            filehandle.write(f'{series_y}\n')
        with open('gyroscope_z.txt', 'w') as filehandle:
            filehandle.write(f'{series_z}\n')
        with open('time_steps_in_sec.txt', 'w') as filehandle:
            filehandle.write(f'{time_series}\n')
        break


    bb_8.pitch_speed += bb_8.attenuation
    bb_8.yaw_speed = min(bb_8.max_speed, max(-bb_8.max_speed, bb_8.attenuation * bb_8.pitch_speed))
    bb_8.pitch_speed = min(bb_8.max_speed, max(-bb_8.max_speed, bb_8.attenuation * bb_8.yaw_speed))

    if current_time > 1.0:
        bb_8.yaw_speed = 1.0 * np.sin(5.0 * current_time / 6.24)
        bb_8.pitch_speed = 4.0
    bb_8.body_yaw_motor.setVelocity(bb_8.yaw_speed)
    bb_8.body_pitch_motor.setVelocity(bb_8.pitch_speed)
    bb_8.head_yaw_motor.setVelocity(bb_8.yaw_speed)



