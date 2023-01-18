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

        self.ps_head_yaw = self.getDevice("HeadYawS")
        self.ps_head_pitch = self.getDevice("HeadPitchS")

        self.ps_head_yaw.enable(self.timestep)
        self.ps_head_pitch.enable(self.timestep)

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
        mask = cv2.erode(mask, kernel, iterations=3)
        mask = cv2.dilate(mask, kernel, iterations=6)
        return mask

    def detect_center_of_objects(self, img):
        centers = []
        contours, hierarchy = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
        for c in contours:
            rect = cv2.boundingRect(c)
            x, y, w, h = rect
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)
            M = cv2.moments(c)
            a = M["m00"]
            if a > 0:
                # calculate the center of mass (COM)
                cX = int(M["m10"] / a)  # average x-coordinate
                cY = int(M["m01"] / a)  # average y-coordinate
                cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1)
                centers.append((cX, cY))
        return centers

    def get_object_angle_from_distance(self, img, x, y):
        # Get the center of the image
        img_center = (img.shape[1] / 2, img.shape[0] / 2)
        # Get the point in the image
        point = (x, y)
        distance = math.dist(img_center, point)
        dx = point[0] - img_center[0]
        dy = point[1] - img_center[1]
        angle = np.arctan2(dy, dx) * 180 / np.pi
        if angle < 0:
            angle = 360 - abs(angle)
        return angle, distance


nao = Nao()
while nao.step(nao.timestep) != -1:
    current_time = nao.getTime()
    img = nao.get_image()
    mask = nao.get_color_mask_and_morphological_of_image(img)
    centers = nao.detect_center_of_objects(mask)
    print(centers)
    try:
        cX = centers[0][0]
        cY = centers[0][1]
    except IndexError:
        target_head_yaw = math.radians(100) * math.sin(current_time)
        target_head_pitch = math.radians(10) * math.cos(current_time)
        nao.head_yaw.setPosition(target_head_yaw)
        nao.head_pitch.setPosition(target_head_pitch)
        continue

    angle, distance = nao.get_object_angle_from_distance(mask, cX, cY)
    print(f"Angle: {angle}")
    print(f"Distance: {distance}")
    cv2.imshow('image', img)
    cv2.waitKey(1)


    target_head_yaw = math.radians(100) * math.sin(math.radians(angle))
    target_head_pitch = math.radians(10) * math.cos(math.radians(angle))
    print(f"Current {math.degrees(nao.ps_head_yaw.getValue())} {math.degrees(nao.ps_head_pitch.getValue())}")

    # set joints
    nao.head_yaw.setPosition(target_head_yaw)
    nao.head_pitch.setPosition(target_head_pitch)
