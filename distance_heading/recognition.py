import numpy as np
import cv2 as cv
from common import draw_str, StatValue

sensor_width = 0.00367
sensor_height = 0.00276
face_height = 0.0205
face_width = 0.015
f = 0.00304
body_classifier = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

def recognition(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    bodies = body_classifier.detectMultiScale(gray, 1.1, 3)
    distance_x = 0
    distance_y = 0
    for (x, y, w, h) in bodies:
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        image_height, image_width = frame.shape[:2]
        distance_x = distance_calculation(w, image_width, sensor_width, face_height)
        distance_y = distance_calculation(h, image_height, sensor_height, face_height)

    return frame, distance_x, distance_y

def distance_calculation(box_dim, img_dim, sensor_dim, face_dim):
    real_dim = face_dim * (img_dim/box_dim)

    sensor_side = np.sqrt((sensor_dim/2)*(sensor_dim/2) + f*f)

    pic_side = (real_dim/sensor_dim)*sensor_side

    distance = np.sqrt((pic_side*pic_side)-((real_dim/2)*(real_dim/2)))

    return distance