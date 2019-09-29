#!/usr/bin/env python3
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
import rospy
if ros_path in sys.path:
   sys.path.remove(ros_path)

import datetime
import time
import cv2 as cv
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/home/domistic/Vision/Function')

from xm_msgs.srv import *
from xm_msgs.msg import *
from Kinect import Camera,Cordinate3D
from Processor import Processor

if_detected = 0
def name_init():
    inf = open('/home/domistic/Vision/models/object/obj.names')
    all_lines = inf.readlines()
    inf.close()
    vec = [x.strip() for x in all_lines]
    print(vec)
    print("the number of objects is " + str(len(vec)))
    return vec

def callback(req):
    global if_detected
    cam = Camera()
    print("here")
    if req.people_id==0:
        processor = Processor("object")
    elif req.people_id==1:
        processor = Processor("person")
    elif req.people_id==2:
        processor = Processor("people")
    elif req.people_id==3:
        processor = Processor("phone")
    vec = name_init()
    # processor.setObjectName(vec)

    obj_name_tmp = req.object_name
    count = 1
    print("*************************")
    print(obj_name_tmp + " need to be found.")
    if obj_name_tmp not in vec and req.people_id==0:
        res = xm_ObjectDetectResponse()
        obj = []
        res.object.append(obj)
        print("the target object is not in the list")
    processor.setDetectName(obj_name_tmp)

    frame = []
    detCordinate = []
    targetCor =  Cordinate3D()

    if not cam.openCamera():
        print("camera failure")
        cam.closeCamera()
        sys.exit(1)

    print("sucessful open")

    while count<=10:
        print(str(count))
        count = count + 1
        targetCor.clear()
        detCordinate.clear()
        # frame = cv.imread("/home/domistic/ssd_pytorch/pic/frame.jpg")
        frame = cam.transColorImg()
        cv.imwrite("/home/domistic/Vision/Frame/frame_detect.jpg",frame)
        print("ddddddddddddd")
        frame = cv.imread("/home/domistic/Vision/Frame/frame_detect.jpg")
        frame_path = "/home/domistic/Vision/Frame/frame_detect.jpg"
        processor.passMat(frame,frame_path)
        if req.people_id==3:
            print("rrrrrrrrrrrrr")
            detCordinate = processor.Find_Phone()
        else:
            detCordinate = processor.getBoundingBox()
            if obj_name_tmp == "Green_tea":
                detCordinate[0][1] = detCordinate[0][1]+(detCordinate[0][3]-detCordinate[0][1])*2/7
            print(detCordinate)
        print(detCordinate)
        
        if len(detCordinate)==0 or len(detCordinate[0])==0:
            print("no object")
            cam.Awaken()
            continue
        if len(detCordinate)!=0:
            # area = (detCordinate[0][2] - detCordinate[0][0])*(detCordinate[0][3] - detCordinate[0][1])
            cam.Awaken()
            break
        if cv.waitKey(1) == ord("q"):
            break

    targetCor = cam.getCordinate3D(detCordinate)
    cam.closeCamera()
    if req.people_id!=3:
        processor.drawRect()
    result = processor.getImage()
    curr_time = datetime.datetime.now()
    name = "/home/domistic/Vision/Result/object_detect"+str(curr_time.hour)+"_"+str(curr_time.minute)+".jpg"
    # cv.imwrite("/home/domistic/Vision/Result/object_detect.jpg", result)
    cv.imwrite(name, result)
    rows, cols = result.shape[0:2]
    result = cv.resize(result, (int(cols/2), int(rows/2)))
    cv.imshow("windows", result)
    cv.waitKey(1000)
    cv.destroyAllWindows()
    print('picture~~~~~')

    res = xm_ObjectDetectResponse()
    print("DetCordinate size: " + str(len(detCordinate)))
    for i in range(0,len(detCordinate)):
        obj_tmp = xm_Object()
        print("X:"+str(targetCor[i].x)+" Y:"+str(targetCor[i].y)+" Z:"+str(targetCor[i].z))
        obj_tmp.pos.point.x = targetCor[i].x
        obj_tmp.pos.point.y = targetCor[i].y
        obj_tmp.pos.point.z = targetCor[i].z
        obj_tmp.pos.header.frame_id = "camera_link"
        obj_tmp.pos.header.stamp = rospy.Time(0)
        if (detCordinate[i][3]-detCordinate[i][1])<150:
            print("&&&&&&&&&&Small thing&&&&&&&&&&")
            obj_tmp.state = 0
        else:
            obj_tmp.state = 1
        res.object.append(obj_tmp)
    # print(res)
        
    if_detected = 1

    return res
    
rospy.init_node('object_detect')
service = rospy.Service('get_position', xm_ObjectDetect, callback)
rospy.loginfo('object_detect')
print("time out")
rospy.spin()
