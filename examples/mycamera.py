# This is a controller for the world mybot_camera.wbt
# File -> Open Sample World -> samples -> mybot -> mybot_camera.wbt

import cv2
import numpy as np

from controller import Robot

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# Motoren
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
# immer fahren
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# drehe um die eigene achse
leftMotor.setVelocity(1)
rightMotor.setVelocity(-1)

camera = robot.getDevice("camera")
camera.enable(timeStep)

width = camera.getWidth()
height = camera.getHeight()


# Ziel: Bild holen und in einen NumPy array umwandeln
# Variante 1
def getImage1(camera):
    img = camera.getImage()
    img = np.frombuffer(img, dtype=np.uint8)
    img = img.reshape((height, width, 4))
    img = img[:, :, [0, 1, 2]]  # last channel not needed
    return img


# Variante 2
def getImage2(camera):
    img = camera.getImageArray()
    img = np.array(img, dtype=np.uint8)
    # switch width and height
    img = np.transpose(img, (1, 0, 2))
    # switch red and blue channels
    img = img[:, :, [2, 1, 0]]
    return img


i = 0
while robot.step(timeStep) != -1:

    # save images
    # https://cyberbotics.com/doc/reference/camera?tab-language=python#wb_camera_save_image
    # camera.saveImage('./{}.png'.format(i), 100)  -> save image for debugging if needed
    i += 1

    img = getImage1(camera)
    # img = getImage2(camera)

    # show image
    cv2.imshow('image', img)

    # show red channel
    cv2.imshow('r', img[:, :, 0])

    # (1) show but don't block
    cv2.waitKey(1)

    # stop if center pixel is red
    x, y = int(height / 2), int(width / 2)
    if img[x, y, 0] < 50 and img[x, y, 1] < 50 and img[x, y, 2] > 100:  # -> does not work (find correct values)
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

# close all windows
cv2.destroyAllWindows()
