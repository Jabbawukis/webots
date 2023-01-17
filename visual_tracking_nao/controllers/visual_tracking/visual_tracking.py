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

    def get_color_mask_and_morphological_of_image(self, image, colour="yellow"):
        if colour == "yellow":
            h = 60 / 2
            lower = np.array([h - 10, 100, 100])
            upper = np.array([h + 10, 255, 255])
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def detect_center_of_objects(self, image):
        ret, thresh = cv2.threshold(image, 127, 255, 0)
        M = cv2.moments(thresh)
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except ZeroDivisionError:
            return -1, -1
        return cX, cY

    def track_object_distance_to_center(self, image, x, y):
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        distance = [center_x - x, center_y - y]
        norm = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
        return [distance[0] / norm, distance[1] / norm]


nao = Nao()
while nao.step(nao.timestep) != -1:
    current_time = nao.getTime()
    mask = nao.get_color_mask_and_morphological_of_image(nao.get_image())
    cX, cY = nao.detect_center_of_objects(mask)
    if cX == -1 and cY == -1:
        target_head_yaw = math.radians(100) * math.sin(current_time)
        target_head_pitch = math.radians(10) * math.cos(current_time)
        nao.head_yaw.setPosition(target_head_yaw)
        nao.head_pitch.setPosition(target_head_pitch)
    else:
        direction = nao.track_object_distance_to_center(mask, cX, cY)

        cv2.imshow('image', mask)
        cv2.waitKey(1)

        #####
        yaw = math.atan2(direction[0], 0)
        magnitude = math.sqrt(direction[0] ** 2 + direction[1] ** 2)
        pitch = math.atan2(-direction[1], magnitude)

        target_head_yaw = math.degrees(yaw)
        target_head_pitch = math.degrees(pitch)
        #####

        # set joints
        nao.head_yaw.setPosition(math.radians(target_head_yaw))
        nao.head_pitch.setPosition(math.radians(target_head_pitch))
