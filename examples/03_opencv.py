import cv2
import numpy as np
import copy
from matplotlib import pyplot as plt

# Oeffne das Bild
img = cv2.imread('robocup.png')

# nutze matplotlib um das Bild anzuzeigen
plt.imshow(img)
print(img.size)  # Anzahl aller Elemente
print(img.shape)  # Anzahl aller Elemente (480(zeilen), 640(spalten), 3(tiefe-kanäle))
# plotte die Farben der 100-ten Zeile
plt.plot(img[100, :, :])
plt.show()

# nutze opencv um das Bild anzuzeigen
cv2.imshow('image', img)
cv2.waitKey(0)  # drücke irgendeine taste -> bild fenster verschwindet

red = img[:, :, 0]  # -> nehme nur ersten kanal
sobel_filter = np.array([[-1, 0, 1],
                         [-2, 0, 2],
                         [-1, 0, 1]])

res = np.zeros(red.shape)
for x in range(1, red.shape[1] - 1):
    for y in range(1, red.shape[0] - 1):
        # nehme Umgebung von Punkt x
        v = np.sum(red[(y - 1):(y + 2), (x - 1):(x + 2)] * sobel_filter)
        res[y, x] = v
res = res / res.max() * 255  # -> normieren
cv2.imshow('sobel', res)
cv2.waitKey(0)

# red = np.array(img)
# red[:, :, [1, 2]] = 0
# cv2.imshow('red', red)

# green = np.array(img)
# green[:, :, [0, 2]] = 0
# cv2.imshow('green', green)
#
# blue = np.array(img)
# blue[:, :, [0, 1]] = 0
# cv2.imshow('blue', blue)
# cv2.waitKey(0)

# cv2.imshow('image', img[:, :, 0])  # -> zeige bild nochmal mit nur kanal 0
# cv2.waitKey(0)
cv2.destroyAllWindows()
