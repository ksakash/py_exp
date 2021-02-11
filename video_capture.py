#!/usr/bin/env python3

import cv2
import numpy as np

url = "rtsp://192.168.43.1:8554/fpv_stream"
cap = cv2.VideoCapture (url)

if (cap.isOpened () == False):
    print ("error in opening video stream")

while (cap.isOpened()):
    ret, frame = cap.read ()
    if (ret == True):
        cv2.imshow ('Frame', frame)

        if (cv2.waitKey (25) & 0xFF == ord ('q')):
            break
    else:
        break

cap.release ()
cv2.destroyAllWindows ()

