import atexit

from controller import Robot, Supervisor
import numpy as np
from pathlib import Path
import os
import pickle

class Panda(Supervisor):
    def __init__(self):
        super().__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.persistent_cache_path = Path(os.getcwd()).joinpath("cache")
        self.persistent_cache_path.mkdir(parents=True, exist_ok=True)
        self.event_pickle_filename = "events.p"

        self.panda_finger_joint1 = self.getDevice("panda_finger_joint1")
        self.panda_finger_joint2 = self.getDevice("panda_finger_joint2")

        self.finger_joints = [self.panda_finger_joint1,
                              self.panda_finger_joint2]

        for joint in self.finger_joints:
            joint.setVelocity(0.2)

        self.panda_joint1 = self.getDevice("panda_joint1")
        self.panda_joint2 = self.getDevice("panda_joint2")
        self.panda_joint3 = self.getDevice("panda_joint3")
        self.panda_joint4 = self.getDevice("panda_joint4")
        self.panda_joint5 = self.getDevice("panda_joint5")
        self.panda_joint6 = self.getDevice("panda_joint6")
        self.panda_joint7 = self.getDevice("panda_joint7")

        self.joint_motors = [
             self.panda_joint1,
             self.panda_joint2,
             self.panda_joint3,
             self.panda_joint4,
             self.panda_joint5,
             self.panda_joint6,
             self.panda_joint7]

        for joint in self.joint_motors:
            joint.setVelocity(0.2)

        self.panda_finger_joint1_sensor = self.getDevice("panda_finger_joint1_sensor")
        self.panda_finger_joint2_sensor = self.getDevice("panda_finger_joint2_sensor")
        self.panda_finger_joint1_sensor.enable(self.timeStep)
        self.panda_finger_joint2_sensor.enable(self.timeStep)

        self.panda_joint1_sensor = self.getDevice("panda_joint1_sensor")
        self.panda_joint2_sensor = self.getDevice("panda_joint2_sensor")
        self.panda_joint3_sensor = self.getDevice("panda_joint3_sensor")
        self.panda_joint4_sensor = self.getDevice("panda_joint4_sensor")
        self.panda_joint5_sensor = self.getDevice("panda_joint5_sensor")
        self.panda_joint6_sensor = self.getDevice("panda_joint6_sensor")
        self.panda_joint7_sensor = self.getDevice("panda_joint7_sensor")

        self.joint_sensors = [
            self.panda_joint1_sensor,
            self.panda_joint2_sensor,
            self.panda_joint3_sensor,
            self.panda_joint4_sensor,
            self.panda_joint5_sensor,
            self.panda_joint6_sensor,
            self.panda_joint7_sensor]

        for sensor in self.joint_sensors:
            sensor.enable(self.timeStep)

        path = self.persistent_cache_path.joinpath(self.event_pickle_filename)
        if path.exists():
            with path.open("rb") as f:
                string = pickle.load(f)
                self.joint_grip_pos_set = set(string)
                print(self.joint_grip_pos_set)
        else:
            self.joint_grip_pos_set = set()

    def save_to_file(self):
        path = self.persistent_cache_path.joinpath(self.event_pickle_filename)
        with path.open("wb") as f:
            pickle.dump(self.joint_grip_pos_set, f, protocol=pickle.HIGHEST_PROTOCOL)

    def open_fingers(self):
        position = self.finger_joints[0].getMaxPosition()
        self.finger_joints[0].setPosition(position)
        self.finger_joints[1].setPosition(position)

    def close_fingers(self):
        self.finger_joints[0].setPosition(0)
        self.finger_joints[1].setPosition(0)

    def move_arm(self, positions: list):
        for pos, joint in zip(positions, self.joint_motors):
            joint.setPosition(pos)

    def reached_arm(self, positions: list):
        error = 0
        for pos, sensor in zip(positions, self.joint_sensors):
            e = np.abs(sensor.getValue() - pos)
            error = np.max([error, e])
        return error < np.radians(1.0)

    def save_rotation(self):
        temp = []
        for sensor in self.joint_sensors:
            temp.append(float("{:.2f}".format(sensor.getValue())))
        if tuple(temp) not in self.joint_grip_pos_set:
            # print(tuple(temp))
            self.joint_grip_pos_set.add(tuple(temp))
            print(self.joint_grip_pos_set)



panda = Panda()
gripper = panda.getFromDef('GRIPPER')
box = panda.getFromDef('BOX')
panda.open_fingers()
grasp = [0.0, 0.520, 0.0, -2.370, -1.600, 2.730, 0.790]
panda.move_arm(grasp)
while panda.step(panda.timeStep) != -1:
    box_vec = np.array(box.getPosition())
    gripper_vec = np.array(gripper.getPosition())
    if np.linalg.norm(box_vec - gripper_vec) < 0.12:
        panda.save_rotation()
    current_time = panda.getTime()
    if current_time >= 10.0:
        panda.save_to_file()

