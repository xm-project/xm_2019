# -*- coding: UTF-8 -*-
import sys
import cv2 as cv

import numpy as np
import pygame
import detect

class Vision:
    class Processor:
        detName = ''    #目标物体
        name = []       #总物体标签
        img = []        #当前图片
        object_all_img = []         #所有物体的绘制图片
        detCordinate = []           #２Ｄ坐标信息存储
        threshold = 0.2             #阈值
        object_name_all = []        #识别到的所有物体的标签
        object_rect_all = []        #识别到的所有物体的坐标

        detector = ''
        objmodel = "/home/domestic/ssd_pytorch/weights/ssd300_VOC_60000.pth"

        min_y = 1060                #用于对识别到的所有物体进行排序
        max_y = 0

        def __init__(self, detType):
            if detType == "object":
                self.detector = detect.Detector(self.objmodel)
            else:
                print("Passing parameter error!!!")
            # print(detType)

        #传目标物体标签
        def setDetectName(self, detectName):
            self.detName = detectName

        #传所有物体标签
        def setObjectName(self, objNames):
            self.name = objNames[:]

        #传当前图片
        def passMat(self, image):
            self.img = image.copy()
            #cv.imshow('image',self.img)
            #cv.waitKey(0)

        #返回识别到的目标物体坐标
        def getBoundingBox(self):
            if len(self.detCordinate)!=0:
                self.detCordinate.clear()
            #这里有个找物体的结果，返回为一个detections列表
            #detections = [[0,1,0.9,2,2,4,5],[0,4,0.7,4,5,6,7],[0,1,0.92,100,150,250,300]]
            detections = self.detector.Detect(self.img)
            print(detections)
            if len(detections)==0:
                return self.detCordinate

            max_threshold = 0
            detect_detection = []
            for detection in detections:
                #id = detection[1]
                #if self.name[id-1]==self.detName and detection[2]>=self.threshold:
                if detection[0]==self.detName and detection[1]>=self.threshold:
                    x1 = detection[2]
                    y1 = detection[3]
                    x2 = detection[4]
                    y2 = detection[5]
                    detect_detection = [x1,y1,x2,y2]
                    # if detection[1]>max_threshold:
                    #     max_threshold = detection[1]
                    #     width = x2 - x1
                    #     height = y2 - y1
                    #     rect = pygame.Rect(x1, y1, width, height)
            if max_threshold==0:
                return self.detCordinate

            self.detCordinate.append(detect_detection)
            
            if self.detName=="people":
                area = 0
                for det in self.detCordinate:
                    width = det[2] - det[0]
                    height = det[3] - det[1]
                    temp_area = width * height
                    if temp_area>area:
                        area = temp_area
                        detect_detection = det
                
                self.detCordinate.clear()
                self.detCordinate.append(rect)

            return self.detCordinate

        #绘制识别到的目标物体
        def drawRect(self):
            if len(self.detCordinate)==0:
                return 
            
            for det in self.detCordinate:
                x = det[0]
                y = det[1]
                width = det[2] - det[0]
                height = det[3] - det[1]
                center_x = x + width / 2
                center_x = int(center_x)
                center_y = y + height / 2
                center_y = int(center_y)
                cv.rectangle(self.img, (x, y), (x+width, y+height), (0, 255, 0), 2)
                cv.putText(self.img, self.detName, (x, y-10), 5, 4, (255, 0, 0), 3)
                cv.circle(self.img, (center_x, center_y), 5, (0,0,255), -1, 8)
            cv.imwrite("//home/domestic/Pictures/result.jpg", self.img)

        #识别五个人
        def five_people(self):
            if len(self.detCordinate)!=0:
                self.detCordinate.clear()
            #detections = [[0,1,0.9,20,20,40,50],[0,4,0.7,4,5,6,7],[0,1,0.92,40,60,80,70]]
            detections = self.detector.Detect(self.img)
            if len(detections)==0:
                print("the number is " + str(len(detections)))
                return self.detCordinate
            
            people_i = 0
            for detection in detections:
                #id = detection[1]
                if detection[0]=="person" and detection[1]>=self.threshold:
                #if id==1 and detection[2]>=self.threshold:      #coco数据集里的person的id为１
                    x1 = detection[2]
                    y1 = detection[3]
                    x2 = detection[4]
                    y2 = detection[5]
                    face = np.ones((x2-x1,y2-y1,3))
                    face = self.img[x1:x2,y1:y2]
                    single_face_path = "//home/domestic/Pictures/" + str(people_i) + ".jpg"
                    cv.imwrite(single_face_path, face)
                    rect = [x1, y1, x2, y2)
                    self.detCordinate.append(rect)
                    people_i = people_i + 1
            person_num = len(self.detCordinate)
            print("the number of people detected is :" + str(person_num))
            if person_num>5:
                person_num = 5

            for i in range(0, person_num):
                for j in range(person_num-1, i, -1):
                    #temp_area = self.detCordinate[i].height * self.detCordinate[i].width
                    area1 = (self.detCordinate[j][3]-self.detCordinate[j][1]) * (self.detCordinate[j][2]-self.detCordinate[j][0])
                    area2 =  (self.detCordinate[j-1][3]-self.detCordinate[j][1]) * (self.detCordinate[j-1][2]-self.detCordinate[j][0])
                    if area1 > area2:
                        rect = self.detCordinate[j]
                        self.detCordinate[j-1] = self.detCordinate[j]
                        self.detCordinate[j] = rect
            position_five_people = []
            for i in range(0, person_num):
                position_five_people.append(self.detCordinate[i])
            return position_five_people

        #识别所有的物体，返回标签
        def getAllObject(self):

            detected_name = []
            detected_rect = []
            detected_confidence = []
            object_confidence = []

            #detections = [[0,1,0.8,20,60,40,80],[0,4,0.7,140,255,266,307],[0,1,0.92,240,60,280,70]]
            detections = self.detector.Detect(self.img)
            if len(detections)==0:
                return detected_name
            
            for detection in detections:
                if detection[1]>self.threshold:
                    x1 = detection[2]
                    y1 = detection[3]
                    x2 = detection[4]
                    y2 = detection[5]
                    rect = [x1, y1, x2, y2]
                    detected_rect.append(rect)
                    detected_confidence.append(detection[1])
                    detected_name.append(detection[0])
                    if y1<self.min_y:
                        self.min_y = y1
                    if y1>self.max_y:
                        self.max_y = y1
            #print(detected_confidence)

            for i in range(0, len(detected_name)):
                #print(detected_name[i])
                #print(self.object_name_all)
                if detected_name[i] not in self.object_name_all:
                    self.object_name_all.append(detected_name[i])
                    self.object_rect_all.append(detected_rect[i])
                    object_confidence.append(detected_confidence[i])
                else:
                    position = self.object_name_all.index(detected_name[i])
                    if object_confidence[position]<detected_confidence[i]:
                        #self.object_name_all[position] = detected_name[i]
                        self.object_rect_all[position] = detected_rect[i]
                        object_confidence[position] = detected_confidence[i]
            # print(object_confidence)
            # print(self.object_rect_all)

            return self.object_name_all

        #绘制识别到的所有物体
        def drawObjectAll(self):
            self.object_all_img = self.img.copy()
            i = 0
            print(self.object_rect_all)
            for rect in self.object_rect_all:
                x = rect[0]
                y = rect[1]
                width = rect[2] - rect[0]
                height = rect[3] - rect[1]
                center_x = x + width / 2
                center_x = int(center_x)
                center_y = y + height / 2
                center_y = int(center_y)
                cv.rectangle(self.object_all_img, (x, y), (x+width, y+height), (0, 255, 0), 2)
                cv.putText(self.object_all_img, self.object_name_all[i], (x, y-10), 5, 4, (255, 0, 0), 3)
                cv.circle(self.object_all_img, (center_x, center_y), 5, (0,0,255), -1, 8)
                i = i + 1
            #cv.imwrite("/home/domestic/Vision/Processor/lib/libprocessor/result.jpg", self.object_all_img)

        #对识别到的所有物体进行排序（方便机械臂抓取）
        def object_sort(self):
            object_name_all_sort = [None]*len(self.object_name_all)
            differenct_y = self.max_y - self.min_y
            #print(str(differenct_y))
            if differenct_y<200:
                for i in range(0, len(self.object_name_all)):
                    x_place = 0
                    for j in range(0, len(self.object_name_all)):
                        if self.object_rect_all[j][0] < self.object_rect_all[i][0]:
                            x_place = x_place + 1
                    object_name_all_sort[x_place] = self.object_name_all[i]
            else:
                y_middle = (self.max_y + self.min_y) / 2
                up_number = 0
                for i in range(0, len(self.object_name_all)):
                    if self.object_rect_all[i][1] < y_middle:
                        up_number = up_number + 1
                        x_place = 0
                        for j in range(0, len(self.object_name_all)):
                            if self.object_rect_all[j][1] < y_middle and self.object_rect_all[j][0] < self.object_rect_all[i][0]:
                                x_place = x_place + 1
                        object_name_all_sort[x_place] = self.object_name_all[i]
                for i in range(0, len(self.object_name_all)):
                    if self.object_rect_all[i][1] > y_middle:
                        x_place = 0
                        for j in range(0, len(self.object_name_all)):
                            if self.object_rect_all[j][1] > y_middle and self.object_rect_all[j][0] < self.object_rect_all[i][0]:
                                x_place = x_place + 1
                        object_name_all_sort[x_place+up_number] = self.object_name_all[i]
            return object_name_all_sort

        def getAllImage(self):
            return self.object_all_img

        def getImage(self):
            return self.img

# pro = Processor("object")
# pro.setDetectName("person")
# print(pro.detName)
# objectname = ['car','person']
# pro.setObjectName(objectname)
# print(pro.name)
# image_ = cv.imread('/home/domestic/ssd_pytorch/pic/00000.png',1)
# pro.passMat(image_)
# det = pro.getBoundingBox()
# print(det[0].x)
# pro.drawRect()
# position = pro.five_people()
# print(position)
# object_name = pro.getAllObject()
# print(object_name)
# pro.drawObjectAll()
# object_sort = pro.object_sort()
# print(object_sort)
# cor = Cordinate2D()
# cor.x1 = 10
# cor.y1 = 10
# cor.x2 = 30
# cor.y2 = 40
# print(str(cor.width()))