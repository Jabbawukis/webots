import math

from controller import Robot, TouchSensor, Camera, Motor, PositionSensor, Motion
import numpy as np
import matplotlib.pyplot as plt


class Nao(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # create sensor member
        self.camera_top = self.getDevice("CameraTop")
        self.camera_bottom = self.getDevice("CameraBottom")
        self.accelerometer = self.getDevice("accelerometer")
        # accelerometer:
        # X;Y;Z [ -0.590, -4.853, -8.693 ]
        # Z=Erdanziehungskraft
        # stillstand = [ 0, -0, -9.926 ]
        # neigung kann dadurch berechnet werden mit atan2(y,x) experiment mit aufprall

        self.gyro = self.getDevice("gyro")
        self.imu = self.getDevice("inertial unit")
        self.gps = self.getDevice("gps")

        # TODO add fsr like in webots nao demo
        self.fsr_r = self.getDevice("RFsr")
        self.fsr_r.enable(self.timestep)
        # TODO move enable down to the other enable statements when the correct fsr implementation is finished
        self.fsr_l = self.getDevice("LFsr")
        self.fsr_l.enable(self.timestep)

        self.ps_head_yaw = self.getDevice("HeadYawS")
        self.ps_head_pitch = self.getDevice("HeadPitchS")
        self.psr_shoulder_pitch = self.getDevice("RShoulderPitchS")
        self.psr_shoulder_roll = self.getDevice("RShoulderRollS")
        self.psr_elbow_yaw = self.getDevice("RElbowYawS")
        self.psr_elbow_roll = self.getDevice("RElbowRollS")
        self.psr_wrist_yaw = self.getDevice("RWristYawS")
        # self.psr_phalanx1 = self.getDevice("RPhalanx1S")
        # self.psr_phalanx2 = self.getDevice("RPhalanx2S")
        # self.psr_phalanx3 = self.getDevice("RPhalanx3S")
        # self.psr_phalanx4 = self.getDevice("RPhalanx4S")
        # self.psr_phalanx5 = self.getDevice("RPhalanx5S")
        # self.psr_phalanx6 = self.getDevice("RPhalanx6S")
        # self.psr_phalanx7 = self.getDevice("RPhalanx7S")
        # self.psr_phalanx8 = self.getDevice("RPhalanx8S")
        self.psl_shoulder_pitch = self.getDevice("LShoulderPitchS")
        self.psl_shoulder_roll = self.getDevice("LShoulderRollS")
        self.psl_elbow_yaw = self.getDevice("LElbowYawS")
        self.psl_elbow_roll = self.getDevice("LElbowRollS")
        self.psl_wrist_yaw = self.getDevice("LWristYawS")
        # self.psl_phalanx1 = self.getDevice("LPhalanx1S")
        # self.psl_phalanx2 = self.getDevice("LPhalanx2S")
        # self.psl_phalanx3 = self.getDevice("LPhalanx3S")
        # self.psl_phalanx4 = self.getDevice("LPhalanx4S")
        # self.psl_phalanx5 = self.getDevice("LPhalanx5S")
        # self.psl_phalanx6 = self.getDevice("LPhalanx6S")
        # self.psl_phalanx7 = self.getDevice("LPhalanx7S")
        # self.psl_phalanx8 = self.getDevice("LPhalanx8S")
        self.psr_hip_yaw_pitch = self.getDevice("RHipYawPitchS")
        self.psr_hip_roll = self.getDevice("RHipRollS")
        self.psr_hip_pitch = self.getDevice("RHipPitchS")
        self.psr_knee_pitch = self.getDevice("RKneePitchS")
        self.psr_ankle_pitch = self.getDevice("RAnklePitchS")
        self.psr_ankle_roll = self.getDevice("RAnkleRollS")
        self.psl_hip_yaw_pitch = self.getDevice("LHipYawPitchS")
        self.psl_hip_roll = self.getDevice("LHipRollS")
        self.psl_hip_pitch = self.getDevice("LHipPitchS")
        self.psl_knee_pitch = self.getDevice("LKneePitchS")
        self.psl_ankle_pitch = self.getDevice("LAnklePitchS")
        self.psl_ankle_roll = self.getDevice("LAnkleRollS")

        # create actuator members
        self.head_yaw = self.getDevice("HeadYaw")
        self.head_pitch = self.getDevice("HeadPitch")
        self.r_shoulder_pitch = self.getDevice("RShoulderPitch")
        self.r_shoulder_roll = self.getDevice("RShoulderRoll")
        self.r_elbow_yaw = self.getDevice("RElbowYaw")
        self.r_elbow_roll = self.getDevice("RElbowRoll")
        self.r_wrist_yaw = self.getDevice("RWristYaw")
        # self.r_phalanx1 = self.getDevice("RPhalanx1")
        # self.r_phalanx2 = self.getDevice("RPhalanx2")
        # self.r_phalanx3 = self.getDevice("RPhalanx3")
        # self.r_phalanx4 = self.getDevice("RPhalanx4")
        # self.r_phalanx5 = self.getDevice("RPhalanx5")
        # self.r_phalanx6 = self.getDevice("RPhalanx6")
        # self.r_phalanx7 = self.getDevice("RPhalanx7")
        # self.r_phalanx8 = self.getDevice("RPhalanx8")
        self.l_shoulder_pitch = self.getDevice("LShoulderPitch")
        self.l_shoulder_roll = self.getDevice("LShoulderRoll")
        self.l_elbow_yaw = self.getDevice("LElbowYaw")
        self.l_elbow_roll = self.getDevice("LElbowRoll")
        self.l_wrist_yaw = self.getDevice("LWristYaw")
        # self.l_phalanx1 = self.getDevice("LPhalanx1")
        # self.l_phalanx2 = self.getDevice("LPhalanx2")
        # self.l_phalanx3 = self.getDevice("LPhalanx3")
        # self.l_phalanx4 = self.getDevice("LPhalanx4")
        # self.l_phalanx5 = self.getDevice("LPhalanx5")
        # self.l_phalanx6 = self.getDevice("LPhalanx6")
        # self.l_phalanx7 = self.getDevice("LPhalanx7")
        # self.l_phalanx8 = self.getDevice("LPhalanx8")
        self.r_hip_yaw_pitch = self.getDevice("RHipYawPitch")
        self.r_hip_roll = self.getDevice("RHipRoll")
        self.r_hip_pitch = self.getDevice("RHipPitch")
        self.r_knee_pitch = self.getDevice("RKneePitch")
        self.r_ankle_pitch = self.getDevice("RAnklePitch")
        self.r_ankle_roll = self.getDevice("RAnkleRoll")
        self.l_hip_yaw_pitch = self.getDevice("LHipYawPitch")
        self.l_hip_roll = self.getDevice("LHipRoll")
        self.l_hip_pitch = self.getDevice("LHipPitch")
        self.l_knee_pitch = self.getDevice("LKneePitch")
        self.l_ankle_pitch = self.getDevice("LAnklePitch")
        self.l_ankle_roll = self.getDevice("LAnkleRoll")

        # init sensors/actuators
        self.enable_sensors()
        self.load_motion_files()

    def left(self, x, y, z):
        # x, z
        self.l_knee_pitch.setPosition(z)
        self.l_hip_pitch.setPosition(-z / 2 + x)
        self.l_ankle_pitch.setPosition(-z / 2 - x)

        # y
        self.l_hip_roll.setPosition(y)
        self.l_ankle_roll.setPosition(-y)

    def right(self, x, y, z):
        # x, z
        self.r_knee_pitch.setPosition(z)
        self.r_hip_pitch.setPosition(-z / 2 + x)
        self.r_ankle_pitch.setPosition(-z / 2 - x)

        # y
        self.r_hip_roll.setPosition(y)
        self.r_ankle_roll.setPosition(-y)

    def stand(self):
        self.r_shoulder_pitch.setPosition(1.60)
        self.l_shoulder_pitch.setPosition(1.60)

    def steady(self, alpha, y, z):
        # TODO hocke
        self.l_ankle_pitch.setPosition(-alpha)
        self.r_ankle_pitch.setPosition(-alpha)

        self.l_knee_pitch.setPosition(2 * alpha)
        self.r_knee_pitch.setPosition(2 * alpha)

        self.l_hip_pitch.setPosition(-alpha)
        self.r_hip_pitch.setPosition(-alpha)

        # more stability
        self.l_hip_roll.setPosition(y)
        self.l_ankle_roll.setPosition(-y)
        self.r_hip_roll.setPosition(-y)
        self.r_ankle_roll.setPosition(y)

    def enable_sensors(self):
        self.camera_top.enable(self.timestep)
        self.camera_bottom.enable(self.timestep)
        self.accelerometer.enable(self.timestep)
        self.gyro.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.gps.enable(self.timestep)
        # TODO enable the rest of the sensors

        self.ps_head_yaw.enable(self.timestep)
        self.ps_head_pitch.enable(self.timestep)
        self.psr_shoulder_pitch.enable(self.timestep)
        self.psr_shoulder_roll.enable(self.timestep)
        self.psr_elbow_yaw.enable(self.timestep)
        self.psr_elbow_roll.enable(self.timestep)
        self.psr_wrist_yaw.enable(self.timestep)
        # self.psr_phalanx1.enable(self.timestep)
        # self.psr_phalanx2.enable(self.timestep)
        # self.psr_phalanx3.enable(self.timestep)
        # self.psr_phalanx4.enable(self.timestep)
        # self.psr_phalanx5.enable(self.timestep)
        # self.psr_phalanx6.enable(self.timestep)
        # self.psr_phalanx7.enable(self.timestep)
        # self.psr_phalanx8.enable(self.timestep)
        self.psl_shoulder_pitch.enable(self.timestep)
        self.psl_shoulder_roll.enable(self.timestep)
        self.psl_elbow_yaw.enable(self.timestep)
        self.psl_elbow_roll.enable(self.timestep)
        self.psl_wrist_yaw.enable(self.timestep)
        # self.psl_phalanx1.enable(self.timestep)
        # self.psl_phalanx2.enable(self.timestep)
        # self.psl_phalanx3.enable(self.timestep)
        # self.psl_phalanx4.enable(self.timestep)
        # self.psl_phalanx5.enable(self.timestep)
        # self.psl_phalanx6.enable(self.timestep)
        # self.psl_phalanx7.enable(self.timestep)
        # self.psl_phalanx8.enable(self.timestep)
        self.psr_hip_yaw_pitch.enable(self.timestep)
        self.psr_hip_roll.enable(self.timestep)
        self.psr_hip_pitch.enable(self.timestep)
        self.psr_knee_pitch.enable(self.timestep)
        self.psr_ankle_pitch.enable(self.timestep)
        self.psr_ankle_roll.enable(self.timestep)
        self.psl_hip_yaw_pitch.enable(self.timestep)
        self.psl_hip_roll.enable(self.timestep)
        self.psl_hip_pitch.enable(self.timestep)
        self.psl_knee_pitch.enable(self.timestep)
        self.psl_ankle_pitch.enable(self.timestep)
        self.psl_ankle_roll.enable(self.timestep)

    def load_motion_files(self):
        self.taiChi = Motion(
            '/home/danielc/PycharmProjects/webots_projects/nao_sensor_world_demo/motions/TaiChi.motion')
        # self.taiChi = Motion('/home/daniel/PycharmProjects/webots_projects/nao_sensor_world_demo/motions/TaiChi.motion')

    def get_joint_positions(self):
        return [
            self.ps_head_yaw.getValue(),
            self.ps_head_pitch.getValue(),
            self.psr_shoulder_pitch.getValue(),
            self.psr_shoulder_roll.getValue(),
            self.psr_elbow_yaw.getValue(),
            self.psr_elbow_roll.getValue(),
            self.psr_wrist_yaw.getValue(),
            # self.psr_phalanx1.getValue(),
            # self.psr_phalanx2.getValue(),
            # self.psr_phalanx3.getValue(),
            # self.psr_phalanx4.getValue(),
            # self.psr_phalanx5.getValue(),
            # self.psr_phalanx6.getValue(),
            # self.psr_phalanx7.getValue(),
            # self.psr_phalanx8.getValue(),
            self.psl_shoulder_pitch.getValue(),
            self.psl_shoulder_roll.getValue(),
            self.psl_elbow_yaw.getValue(),
            self.psl_elbow_roll.getValue(),
            self.psl_wrist_yaw.getValue(),
            # self.psl_phalanx1.getValue(),
            # self.psl_phalanx2.getValue(),
            # self.psl_phalanx3.getValue(),
            # self.psl_phalanx4.getValue(),
            # self.psl_phalanx5.getValue(),
            # self.psl_phalanx6.getValue(),
            # self.psl_phalanx7.getValue(),
            # self.psl_phalanx8.getValue(),
            self.psr_hip_yaw_pitch.getValue(),
            self.psr_hip_roll.getValue(),
            self.psr_hip_pitch.getValue(),
            self.psr_knee_pitch.getValue(),
            self.psr_ankle_pitch.getValue(),
            self.psr_ankle_roll.getValue(),
            self.psl_hip_yaw_pitch.getValue(),
            self.psl_hip_roll.getValue(),
            self.psl_hip_pitch.getValue(),
            self.psl_knee_pitch.getValue(),
            self.psl_ankle_pitch.getValue(),
            self.psl_ankle_roll.getValue()
        ]


series_x = []
series_y = []
series_z = []
time_series = []
pitch_series = []
roll_series = []
nao = Nao()
init_time_step = 0.0
while nao.step(nao.timestep) != -1:
    current_time = nao.getTime()
    if current_time >= init_time_step + 1.0:
        init_time_step = current_time
        time_step = nao.getTime()
        val = nao.accelerometer.getValues()
        # print(val)
        accelerationX = val[0]
        accelerationY = val[1]
        accelerationZ = val[2]

        series_x.append(accelerationX)
        series_y.append(accelerationY)
        series_z.append(accelerationZ)
        time_series.append(nao.getTime())

        pitch = 180 * np.arctan2(accelerationX,
                                 np.sqrt(accelerationY * accelerationY + accelerationZ * accelerationZ)) / np.pi

        roll = 180 * np.arctan2(accelerationY,
                                np.sqrt(accelerationX * accelerationX + accelerationZ * accelerationZ)) / np.pi
        pitch_series.append(pitch)
        roll_series.append(roll)

        print(f"{pitch}, {roll}")

        nao.taiChi.play()
        if time_step >= 32.0:
            plt.plot(time_series, pitch_series, 'r', label='Pitch')
            plt.plot(time_series, roll_series, 'g', label='Roll')
            # plt.plot(time_series, series_z, 'b', label='Z Acceleration')
            plt.legend()
            plt.show()
            break
