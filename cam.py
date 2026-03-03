import cv2
from tkinter import *


#cam = cv2.VideoCapture('http://169.254.168.181:8081/')
cam = cv2.VideoCapture('http://192.168.8.149:8081/') # ввести ip адрес расбери

scale = 50

while True:
    ret, image = cam.read()
    #cv2.imshow("camera", image)

# Логика зума изображения

    height, width, channels = image.shape
    # prepare the crop
    centerX, centerY = int(height / 2), int(width / 2)
    radiusX, radiusY = int(scale * height / 100), int(scale * width / 100)

    minX, maxX = centerX - radiusX, centerX + radiusX
    minY, maxY = centerY - radiusY, centerY + radiusY

    cropped = image[minX:maxX, minY:maxY]
    resized_cropped = cv2.resize(cropped, (1920, 1080)) # Разрешение окна вывода изображения

    cv2.imshow('my webcam', resized_cropped)
    #cv2.imshow('crop', cropped)



    kk = cv2.waitKey(1)
    #print(kk, scale)

# Управление

    if kk == 27: # Клавиша Esc
        break
    elif kk == 61 and scale>2:
            scale -= 2  # +5
    elif kk == 45 and scale<50:
            scale += 2  # +5
        # add + or - 5 % to zoom

    #if cv2.waitKey(10) == 0:
      #      scale += 5  # +5

    #if cv2.waitKey(10) == 1:
        #    scale = 5  # +5
cam.release()
cv2.destroyAllWindows()

