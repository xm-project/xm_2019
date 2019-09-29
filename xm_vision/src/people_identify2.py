#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
import rospy
if ros_path in sys.path:
   sys.path.remove(ros_path)

import os
import math
import time
import face_recognition
import cv2 as cv
import numpy as np
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/home/domistic/Vision/Function')

from xm_msgs.srv import *
from xm_msgs.msg import *
from Kinect import Camera,Cordinate3D
from Find_People import face_segmentation,recognize, recognize2

if_detected = 0
wrong_flag = 0
name_id = []

def get_person_location(command_num, search_id):
    global if_detected
    global wrong_flag
    if command_num==1:
        #这里有个摄像头
        # frame = cv.imread("/home/domistic/Vision/frame_people.jpg")
        cam = Camera()
        if not cam.openCamera():
            print("camera failure")
            cam.closeCamera()
            sys.exit(1)

        print("sucessful open")
        
        for i in range(0,10):
            continue
            # print(str(i))
            # frame = cam.transColorImg()
            # s = "/home/domistic/Vision/" + str(i) + ".jpg"
            # cv.imwrite(s,frame)
            # cam.Awaken()
        time.sleep(1)

        frame = cam.transColorImg()
        rows, cols = frame.shape[0:2]
        frame = cv.resize(frame,(int(cols/2), int(rows/2)))
        cv.imwrite("/home/domistic/Vision/Frame/frame_people.jpg",frame)
        # frame = cv.imread("/home/domistic/Vision/frame_people.jpg")
        
        os.system("rm -rf /home/domistic/Vision/Result/people_identify/")
        os.system("mkdir /home/domistic/Vision/Result/people_identify/")
        os.system("rm -rf /home/domistic/Vision/Result/people_identify_final/")
        os.system("mkdir /home/domistic/Vision/Result/people_identify_final/")
        
        # 如果测试出来的人脸多于三个退出循环，靠深度距离筛选，如果识别出了人脸但是数量少于三个，先进行保存，如果十次之后还是没识别出三个人脸
        # 那就返回此前有人脸的个数，如果十次之后仍然一张人脸都没识别出来，最终结果返回10 10 10
        detect_num = 0
        while detect_num<=2:
            detect_num += 1
            people_position_tmp = face_segmentation()
            print("The number of people is " + str(len(people_position_tmp)))
            if len(people_position_tmp)>=3:
                break
            else:
                cam.Awaken()
                frame = cam.transColorImg()
                rows, cols = frame.shape[0:2]
                frame = cv.resize(frame,(int(cols/2), int(rows/2)))
                cv.imwrite("/home/domistic/Vision/Frame/frame_people.jpg",frame)
                continue
        if len(people_position_tmp)==0:
            tmp = [0,0,0,0]
            people_position_tmp.append(tmp)
        print(people_position_tmp)

        cam.Awaken()
        targetCor_tmp = cam.getCordinate3D(people_position_tmp)
        cam.closeCamera()

        if len(people_position_tmp)==0:
            return targetCor_tmp

        targetCor = []

        for i in range(0,len(targetCor_tmp)):
            if targetCor_tmp[i].z <= 2.5 and targetCor_tmp[i].z >= 0.35:
                targetCor.append(targetCor_tmp[i])
                top = int(people_position_tmp[i][1]/2)
                bottom = int(people_position_tmp[i][3]/2)
                left = int(people_position_tmp[i][0]/2)
                right = int(people_position_tmp[i][2]/2)
                tmp = frame[top:bottom,left:right]
                rows, cols = tmp.shape[0:2]
                tmp = cv.resize(tmp,(int(cols*2), int(rows*2)))
                cv.imwrite("/home/domistic/Vision/Result/people_identify_final/"+str(i)+".jpg",tmp)

        if len(targetCor) == 0:
            targetCor = targetCor_tmp[:]
        
        if_detected = 1
        return targetCor

    elif command_num == 2:
        #这里有个摄像头
        # frame = cv.imread("/home/domistic/Vision/frame_pscene.jpg")
        global name_id
        cam = Camera()
        if not cam.openCamera():
            print("camera failure")
            cam.closeCamera()
            sys.exit(1)

        print("sucessful open")

        for i in range(0,10):
            continue
            # print(str(i))
        time.sleep(0.5)

        frame = cam.transColorImg()
        cv.imwrite("/home/domistic/Vision/Frame/frame_pscene.jpg",frame)
        #os.system("rm /home/domistic/Vision/data/people_identify/id.txt")
        #os.system("rm /home/domistic/Vision/data/people_identify/message.txt")
        face_location_all = []
        name_id = []
        print("Start!!!")
        for i in range(0,3):
            search_id = i
            print("People ID: "+str(search_id))
            test_num = 0
            face_threshold = 0.45
            face_location = []
            while test_num < 8:
                print("Threshold:::"+str(face_threshold))
                test_num += 1
                face_location = recognize(search_id,face_threshold)
                print(face_location)
                if len(face_location)==4 and face_location[0]!=-1 and face_location[0]!=-2:
                    print("I find you!!!")
                    print(face_location)
                    face_location_all.append(face_location)
                    name_id.append(search_id)
                    break
                elif len(face_location)==4 and face_location[0]==-1:         #阈值低了，没有识别到人脸
                    print("No target.Try again.")
                    face_threshold += 0.02
                    cam.Awaken()
                    frame = cam.transColorImg()
                    cv.imwrite("/home/domistic/Vision/Frame/frame_pscene.jpg",frame)
                elif len(face_location)>4:       #阈值高了，识别过多人脸
                    print("No target.Try again.")
                    face_threshold -= 0.01
                    cam.Awaken()
                    frame = cam.transColorImg()
                    cv.imwrite("/home/domistic/Vision/Frame/frame_pscene.jpg",frame)
                else:                               
                    print("No target.Try again.")
                    face_threshold += 0.02
                    cam.Awaken()
                    frame = cam.transColorImg()
                    cv.imwrite("/home/domistic/Vision/Frame/frame_pscene.jpg",frame)

        if 0 not in name_id:
            test_num = 0
            while test_num<5:
                test_num += 1
                face_location = recognize2(0)
                if len(face_location)!=0:
                    print("Find people0")
                    name_id.insert(0,0)
                    face_location_all.insert(0,face_location)
                    break
        if 1 not in name_id:
            test_num = 0
            while test_num<5:
                test_num += 1
                face_location = recognize2(1)
                if len(face_location)!=0:
                    print("Find people1")
                    if 2 in name_id and len(name_id)==1:
                        name_id.insert(0,1)
                        face_location_all.insert(1,face_location)
                    else:
                        name_id.insert(1,1)
                        face_location_all.insert(1,face_location)
                    break
        if 2 not in name_id:
            test_num = 0
            while test_num<5:
                test_num += 1
                face_location = recognize2(2)
                if len(face_location)!=0:
                    print("Find people2")
                    name_id.insert(2,2)
                    face_location_all.insert(2,face_location)
                    break

        single_position = []
        if face_location_all[0][0]==-1 or face_location_all[1][0]==-1 or face_location_all[2][0]==-1 :
            single_position_ = [-1, -1, -1]
            wrong_flag = -1
            if_detected = 1
            cam.Awaken()
            cam.closeCamera()
            single_position.append(single_position_)
            return single_position
        elif face_location_all[0][0]==-2 or face_location_all[1][0]==-2 or face_location_all[2][0]==-2 :
            single_position_ = [-1, -1, -1]
            wrong_flag = -2
            if_detected = 1
            cam.Awaken()
            cam.closeCamera()
            single_position.append(single_position_)
            return single_position
        cam.Awaken()
        print("Final result:")
        print(face_location_all)
        targetCor = cam.getCordinate3D(face_location_all)
        cam.closeCamera()
        single_position1 = [targetCor[0].x, targetCor[0].y, targetCor[0].z]
        single_position2 = [targetCor[1].x, targetCor[1].y, targetCor[1].z]
        single_position3 = [targetCor[2].x, targetCor[2].y, targetCor[2].z]
        if math.isnan(single_position1[0]) or math.isnan(single_position1[1]) or math.isnan(single_position1[2]) or math.isnan(single_position2[0]) or math.isnan(single_position2[1]) or math.isnan(single_position2[2]) or math.isnan(single_position3[0]) or math.isnan(single_position3[1]) or math.isnan(single_position3[2]):
            wrong_flag = -1
            if_detected = 1
            cam.Awaken()
            cam.closeCamera()
            return [[-1, -1, -1]]
        if single_position1[0]==-10 or single_position2[0]==-10 or single_position3[0]==-10:
            wrong_flag = -2
            if_detected = 1
            cam.Awaken()
            cam.closeCamera()
            return [[-1, -1, -1]]
        if single_position1[0]==10 or single_position2[0]==10 or single_position3[0]==10:
            wrong_flag = -4
            if_detected = 1
            cam.Awaken()
            cam.closeCamera()
            return [[-1, -1, -1]]
    
        if_detected = 1
        
        print("I find you")
        return targetCor
        

def callback(req):
    obj_name_tmp = req.object_name
    if obj_name_tmp == "person":
        if req.people_id == -1:
            print("Find all people")
            targetCor = get_person_location(1,-1)
            count = 0
            res = xm_ObjectDetectResponse()
            for cor in targetCor:
                obj_tmp = xm_Object()
                obj_tmp.name = count
                count += 1
                print(str(count)+":"+"  X:"+str(cor.x)+"  Y:"+str(cor.y)+"  Z:"+str(cor.z))
                obj_tmp.pos.point.x = cor.x
                obj_tmp.pos.point.y = cor.y
                obj_tmp.pos.point.z = cor.z
                obj_tmp.pos.header.frame_id = "camera_link"
                obj_tmp.pos.header.stamp = rospy.Time(0)
                res.object.append(obj_tmp)

            print("**********Print_Res**********")
            for r in res.object:
                print(str(r.name)+":"+"   X:"+str(r.pos.point.x)+"  Y:"+str(r.pos.point.y)+"  Z:"+str(r.pos.point.z))
            print("**********Finish**********")
            print(str(len(res.object)))
            return res

        else:
            # print("People ID:" + str(req.people_id))
            targetCor = get_person_location(2,req.people_id)
            print("Result:")
            print("obj_tempX、obj_tempY、obj_tempZ")
            print(targetCor)

            res = xm_ObjectDetectResponse()
            if len(targetCor)==1:
                obj_tmp = xm_Object()
                obj_tmp.name = -1
                obj_tmp.pos.point.x = targetCor[0][0]
                obj_tmp.pos.point.y = targetCor[0][1]
                obj_tmp.pos.point.z = targetCor[0][2]
                obj_tmp.pos.header.frame_id = "camera_link"
                obj_tmp.pos.header.stamp = rospy.Time(0)
                obj_tmp.state = wrong_flag
                res.object.append(obj_tmp)
            else:
                id_num = 0
                for cor in targetCor:
                    obj_tmp = xm_Object()
                    obj_tmp.name = name_id[id_num]
                    obj_tmp.pos.point.x = cor.x
                    obj_tmp.pos.point.y = cor.y
                    obj_tmp.pos.point.z = cor.z
                    obj_tmp.pos.header.frame_id = "camera_link"
                    obj_tmp.pos.header.stamp = rospy.Time(0)
                    obj_tmp.state = wrong_flag
                    res.object.append(obj_tmp)
                    id_num += 1
            print("**********Finish**********")
            return res

rospy.init_node('people_identify')
service = rospy.Service('get_position', xm_ObjectDetect, callback)
rospy.loginfo('*********people_identify*********')
rospy.spin()
