#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import time
import datetime
import sys
import os
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
   sys.path.remove(ros_path)
import cv2 as cv
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/home/domistic/Vision/Function')

from Kinect import Camera

cam = Camera()
if not cam.openCamera():
    print("camera failure")
    cam.closeCamera()
    sys.exit(1)

print("sucessful open")

today = datetime.date.today()
floder_string = "/home/domistic/Vision/Data/" + str(today)
print("目录：" + floder_string)
if not os.path.exists(floder_string):
    os.mkdir(floder_string)

pic_num = 0
cv.namedWindow("colorImg")
while(1):
    frame = cam.transColorImg()
    frame = cv.resize(frame,(int(1920/2),int(1080/2)))
    cv.imshow("colorImg",frame)
    cam.Awaken()

    if cv.waitKey(1) == ord('c'):
        today = datetime.date.today()
        now = time.strftime("%H:%M:%S")
        string = floder_string + "/" + str(today) + "-" +  str(now) + ".jpg"
        cv.imwrite(string,frame)
        pic_num += 1
        print("You have taken " + str(pic_num) + " pictures")

    if cv.waitKey(1) == ord('q'):
        cv.destroyAllWindows()
        break

cam.closeCamera()

sys.exit(0)
