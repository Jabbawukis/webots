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
        self.width_top = self.camera_top.getWidth()
        self.height_top = self.camera_top.getHeight()

        self.ps_head_yaw = self.getDevice("HeadYawS")
        self.ps_head_pitch = self.getDevice("HeadPitchS")
        self.ps_head_yaw.enable(self.timestep)
        self.ps_head_pitch.enable(self.timestep)

        self.head_yaw = self.getDevice("HeadYaw")
        self.head_pitch = self.getDevice("HeadPitch")

        self.velocity = 0.3

        self.head_yaw.setVelocity(self.velocity)
        self.head_pitch.setVelocity(self.velocity)

        self.angle = 0.0
        self.distance = 0.0

        self.target_head_yaw = 0
        self.target_head_pitch = 0

        self.state = "no_object_detected"
        self.previous_state = ""

        # move arms down
        self.lShoulderPitch = self.getDevice("LShoulderPitch")
        self.rShoulderPitch = self.getDevice("RShoulderPitch")
        self.lShoulderPitch.setPosition(math.radians(90))
        self.rShoulderPitch.setPosition(math.radians(90))

    def set_velocity(self, velocity):
        self.head_yaw.setVelocity(velocity)
        self.head_pitch.setVelocity(velocity)

    def get_image(self, camera="top"):
        if camera == "top":
            img = self.camera_top.getImage()
            height = self.height_top
            width = self.width_top
        elif camera == "bottom":
            img = self.camera_bottom.getImage()
            height = self.height_bottom
            width = self.width_bottom

        img = np.frombuffer(img, dtype=np.uint8)
        img = img.reshape((height, width, 4))
        img = img[:, :, [0, 1, 2]]  # last channel not needed
        # Blur für welt 03 -> verbessert das rauschen
        img = cv2.medianBlur(img, 5)

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
        rects = []
        contours, hierarchy = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

        for c in contours:
            rect = cv2.boundingRect(c)
            rects.append(rect)
            M = cv2.moments(c)
            a = M["m00"]

            if a > 0:
                # calculate the center of mass (COM)
                cX = int(M["m10"] / a)  # average x-coordinate
                cY = int(M["m01"] / a)  # average y-coordinate
                centers.append((cX, cY))

        return centers, rects, contours

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

    def calculate_target_yaw_pitch(self, img, cX, cY):
        # Get the center of the image
        img_center = (img.shape[1] / 2, img.shape[0] / 2)
        # Get the point in the image
        point = (cX, cY)

        dx = (point[0] - img_center[0]) * (-1)
        dy = point[1] - img_center[1]
        dx_normalized = dx / img_center[0]
        dy_normalized = dy / img_center[1]

        head_yaw = nao.target_head_yaw + dx_normalized
        new_target_head_yaw = max(math.radians(-100), min(head_yaw, math.radians(100)))
        head_pitch = nao.target_head_pitch + dy_normalized
        new_target_head_pitch = max(math.radians(-10), min(head_pitch, math.radians(10)))

        return new_target_head_yaw, new_target_head_pitch

    def select_object(self, contours_len, rects):
        out = []
        if contours_len > 1:
            for rect in rects:
                x, y, w, h = rect
                if w / h > 0.60:
                    out.append(rects.index(rect))
                else:
                    pass
        else:
            return 0
        if len(out) == 0:
            return 0
        else:
            return out[0]

nao = Nao()
while nao.step(nao.timestep) != -1:
    current_time = nao.getTime()
    img = nao.get_image("top")
    mask = nao.get_color_mask_and_morphological_of_image(img)
    centers, rects, contours = nao.detect_center_of_objects(mask)

    nao.previous_state = nao.state
    if len(centers) == 0:
        nao.state = "no_object_detected"
    else:
        nao.state = "object_detected"

    if nao.state == "no_object_detected":
        nao.set_velocity(0.3)
        nao.target_head_yaw = math.radians(100) * math.cos(math.radians(nao.angle)) * (-1)
        nao.target_head_pitch = math.radians(10) * math.sin(math.radians(nao.angle))

        # Set joints
        nao.head_yaw.setPosition(nao.target_head_yaw)
        nao.head_pitch.setPosition(nao.target_head_pitch)

    if nao.state == "object_detected":
        # Select centroid to be tracked
        c_index = nao.select_object(len(contours), rects)
        cX = centers[c_index][0]
        cY = centers[c_index][1]

        nao.set_velocity(2)
        nao.angle, nao.distance = nao.get_object_angle_from_distance(mask, cX, cY)

        # Calculate head movements
        nao.target_head_yaw, nao.target_head_pitch = nao.calculate_target_yaw_pitch(mask, cX, cY)

        # Set joints
        nao.head_yaw.setPosition(nao.target_head_yaw)
        nao.head_pitch.setPosition(nao.target_head_pitch)

    # Show processed robot camera
    #####################
    blob = cv2.bitwise_and(img, img, mask=mask)

    cv2.drawContours(blob, contours, -1, (0, 0, 255), 1)
    for i in range(0, len(centers)):
        center = centers[i]
        rect = rects[i]
        x, y, w, h = rect
        cX, cY = center

        color = (255, 0, 255) if i == c_index else (255, 0, 0)
        cv2.rectangle(blob, (x, y), (x + w, y + h), color, 1)
        cv2.circle(blob, (cX, cY), 7, color, -1)

    color = (0, 255, 0) if nao.state == "object_detected" else (0, 0, 255)
    cv2.line(blob, (0, int(blob.shape[0] / 2)), (blob.shape[1], int(blob.shape[0] / 2)), color, 1)
    cv2.line(blob, (int(blob.shape[1] / 2), 0), (int(blob.shape[1] / 2), blob.shape[0]), color, 1)

    cv2.imshow('Ausgewertete Ansicht (Oben)', blob)
    cv2.waitKey(1)
    #####################

