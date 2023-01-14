import math
from controller import Robot, Camera
import cv2
import numpy as np

class Nao(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep() * 4)
        # create sensor member
        self.camera_top = self.getDevice("CameraTop")
        self.camera_top.enable(self.timestep)
        self.width = self.camera_top.getWidth()
        self.height = self.camera_top.getHeight()

        ## enable the bottom camera if necessary
        # camera_bottom = robot.getDevice("CameraBottom")
        # camera_bottom.enable(timestep)

        self.head_yaw = self.getDevice("HeadYaw")
        self.head_pitch = self.getDevice("HeadPitch")

        # move arms down
        self.lShoulderPitch = self.getDevice("LShoulderPitch")
        self.rShoulderPitch = self.getDevice("RShoulderPitch")
        self.lShoulderPitch.setPosition(math.radians(90))
        self.rShoulderPitch.setPosition(math.radians(90))

    def get_image(self):
        img = self.camera_top.getImage()
        img = np.frombuffer(img, dtype=np.uint8)
        img = img.reshape((self.height, self.width, 4))
        img = img[:, :, [0, 1, 2]]  # last channel not needed
        return img


nao = Nao()
while nao.step(nao.timestep) != -1:
    current_time = nao.getTime()

    # save frames (slows down the simulation)
    # nao.camera_top.saveImage('./image.png', 100)
    img = nao.get_image()
    # show image
    cv2.imshow('image', img)
    cv2.waitKey(1)

    # calculate the target joints
    target_head_yaw = math.radians(100) * math.sin(current_time)
    target_head_pitch = math.radians(10) * math.cos(current_time)

    # set joints
    nao.head_yaw.setPosition(target_head_yaw)
    nao.head_pitch.setPosition(target_head_pitch)
