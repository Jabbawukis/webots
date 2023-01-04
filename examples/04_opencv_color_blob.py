import cv2
import numpy as np

# Oeffne das Bild
img = cv2.imread('robocup.png')

# convert to hsv
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# gray scale image
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv2.imshow('image', img)
cv2.imshow('gray', gray)

# define range of blue color in HSV
h = 320 / 2
lower = np.array([h - 20, 102, 153])
upper = np.array([h + 20, 204, 255])

# mask color
mask = cv2.inRange(hsv, lower, upper)
cv2.imshow('color mask', mask)

# adapt the mask
kernel = np.ones((3, 3), np.uint8)
mask = cv2.erode(mask, kernel, iterations=1)
mask = cv2.dilate(mask, kernel, iterations=1)
opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
cv2.imshow('color mask + morphological operations', mask)

# Bitwise-AND mask and original image
blob = cv2.bitwise_and(img, img, mask=mask)

# detect contours
contours, hierarchy = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
print(contours)

cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
cv2.imshow('image', img)

# show all images
cv2.waitKey(0)
cv2.destroyAllWindows()
