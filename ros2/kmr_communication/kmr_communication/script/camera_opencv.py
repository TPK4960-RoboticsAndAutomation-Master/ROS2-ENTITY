import numpy as np
from cv2 import cv2 as cv

class Camera(object):

    def __init__(self):
        cap = cv.VideoCapture(0)

        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        
        self._camera = cap

    def capture(self):
        ret, frame = self._camera.read()

        if not ret:
            print("Can't receive frame...")
            exit()

        return frame

    def __del__(self):
        self._camera.release()
        cv.destroyAllWindows()

"""
cam = Camera()

while True:
    cv.imshow('frame', cam.capture())

    if cv.waitKey(1) == ord('q'):
        break

"""