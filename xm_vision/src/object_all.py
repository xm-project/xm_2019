#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import time
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
import rospy
if ros_path in sys.path:
   sys.path.remove(ros_path)

import cv2 as cv
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/home/domistic/Vision/Function')

from Kinect import Camera,Cordinate3D
from Processor import Processor
import os

cam = Camera()

detect_thing = "object"

processor = Processor(detect_thing)

count = 1
obj_num = 0
frame = []
result_img = []
detect_name = []
result_name = []
detCordinate = []

if not cam.openCamera():
    print("camera failure")
    cam.closeCamera()
    sys.exit(1)

print("sucessful open")

while count<=10:
    count = count + 1
    #这里有个摄像头获取图片
    #frame = cv.imread("/home/domistic/ssd_pytorch/pic/00000.png")
    frame = cam.transColorImg()
    cv.imwrite("/home/domistic/Vision/Frame/frame_all.jpg",frame)
    frame = cv.imread("/home/domistic/Vision/Frame/frame_all.jpg")
    frame_path = "/home/domistic/Vision/Frame/frame_all.jpg"
    processor.passMat(frame,frame_path)

    detect_name = processor.getAllObject()
    obj_num = len(detect_name)
    print("the number of objects detected is " + str(obj_num))
    cam.Awaken()

cam.closeCamera()
result_name = processor.object_sort()
print("Result:")
print(result_name)

outfile_path = "/home/domistic/Vision/Result/out_all.txt"
with open(outfile_path,'w') as outfile:
    for name in result_name:
        outfile.write(name)
        outfile.write('\n')
processor.drawObjectAll()
result_img = processor.getImage()
cv.imwrite("/home/domistic/Vision/Result/object_all.jpg", result_img)
rows, cols = result_img.shape[0:2]
result_img = cv.resize(result_img, (int(cols/2), int(rows/2)))
cv.imshow("detect", result_img)
cv.waitKey(1000)
cv.destroyAllWindows()
