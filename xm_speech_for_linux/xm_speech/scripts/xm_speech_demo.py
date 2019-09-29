#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Created on Thu Jan 19 09:37:01 2017
altered on 2018& 2019

@author: fanfan

This project is to build an interesting chatting robot and aims to serve intelligent home

newest change in 2019.8.29
"""

import sys
import time
import math
import json
import collections
import audioop
import pyaudio
import requests
import random
import rospy
import threading
import time
from std_msgs.msg import String
from xm_speech.srv import *
from ctypes import *
from speech_recognition import *

import nltk
import os
import re
import codecs

from nltk import CFG
from nltk.parse import RecursiveDescentParser

import subprocess
import time

pause = 0
xm_speech_req = xm_Speech_meaningRequest()
xm_speech_res = xm_Speech_meaningResponse()
object = ["Bath_cream","Milk_tea","Biscuits","Chips","Napkin","Oreo","Green_tea","Pepsi","Herbal_tea","Porridge","Jelly"]
objectno = ["Bath cream","Milk tea","Biscuits","Chips","Napkin","Oreo","Green tea","Pepsi","Herbal tea","Porridge","Jelly"]
double_words =["Bath_cream","Milk_tea","Green_tea","Herbal_tea","kitchen_table","kitchen_counter","couch_table","dinner_table","left_bedside_table","right_bedside_table","hallway_table"]
kitchen = ["fridge","kitchen table","kitchen counter","cupboard","trashbin"]
livingroom = ["bar","couch table","dinner table","sofa"]
bedroom = ["left bedside table","right bedside table","desk"]
hallway = ["bookcase","hallway table"]
people = ["Adam","Daniel","Fisher","Michael","Jack","John","Mary","Tom","Kevin","Rose"]
text_last = ''
sentences = 'join'

grammar=CFG.fromstring("""
S ->Mission1|Mission1 Mission2|Mission1 Mission2 Mission3

Mission1 ->Val VP
Mission2 ->Val VP
Mission3 ->Val VP

VP ->V NP|V NP PP|V PP|V NP NP|V PP PP|V NP NP PP|V WHH|V NP NP WHH PP|TAKE NP NP WHH PP|TAKE NP|TAKE NP PP|TAKE PP|TAKE NP NP|TAKE PP PP|TAKE NP NP PP|
NP ->Det Nom|PropN|Nom|P N|P Nom
Nom ->Adj N|N|N NP
PP ->P NP
WHH ->WH N V PropN|WH N V N|WH V

Val ->"and"|"then"|
Det ->"a"|"an"|"the"|"your"|"my"|
PropN ->"her"|"him"|"me"|"it"
P ->To|In|For|Of
Adj ->"calling"
WH -> "what"|"which"

N ->Location|Object|Topic|Person
V ->Find|Go|Deliver|Talk|Follow|Grasp|Remember|Stop|"is"
TAKE ->"take"

Location ->"kitchen"|"diningroom"|"bedroom"|"livingroom"|"here"|"fridge"|"kitchen_table"|"kitchen_counter"|"cupboard"|"trashbin"|"bar"|"couch_table"|"dinner_table"|"sofa"|"left_bedside_table"|"right_bedside_table"|"desk"|"bed"|"bookcase"|"hallway_table"|"hallway"|"shelf"
Object ->"Bath_cream"|"Milk_tea"|"Biscuits"|"Chips"|"Napkin"|"Oreo"|"Green_tea"|"Pepsi"|"Herbal_tea"|"Porridge"|"Jelly"
Person ->"person"|"her"|"Adam"|"Daniel"|"Fisher"|"Michael"|"Jack"|"John"|"Mary"|"Tom"|"Kevin"|"Rose"
Topic ->Time|Date|Name|Day|Question|Sentence
Time ->"time"
Date ->"date"|"today"|"day"|"tomorrow"
Name ->"name"|"team"
Day ->"week"|"month"|"whatdayistoday"
Question ->"question"
Sentence ->"sentence"


Find ->"find"|"look"|"search"
Go ->"go"|"navigate"|"reach"|"get"
Deliver ->"deliver"|"bring"|"carry"
Talk ->"tell"|"say"|"speak"|"ask"|"answer"
Follow ->"follow"
Grasp ->"grasp"|"get"|"put"
Remember ->"remember"

To ->"to"|"into"|"back"
In ->"in"|"from"|"on"|"at"
Of ->"of"
For ->"for"

""")

def search(tree):
    global xm_speech_res
    if tree.label()=="S":
        if len(tree)==1:
            tr=tree[0]
            search(tr)
            
        elif len(tree)==2:
            tr=tree[0]
            search(tr)
            tr=tree[1]
            search(tr)
            
        elif len(tree)==3:
            tr=tree[0]
            search(tr)
            tr=tree[1]
            search(tr)
            tr=tree[2]
            search(tr)
#	    
    if tree.label()=="Mission1"or tree.label()=="Mission2" or tree.label()=="Mission3":
        if len(tree)==2:
            tr=tree[1]
            search(tr)
#	    
    if tree.label()=="VP":
        if len(tree)==2:
            if (tree[0].label()=="V" or tree[0].label == "TAKE") and tree[1].label()=="NP":
                tr=tree[0]
                search(tr)
                tr=tree[1]
                search(tr)
                
            elif (tree[0].label()=="V" or tree[0].label == "TAKE") and tree[1].label()=="PP":
                tr=tree[0]
                search(tr)
                tr=tree[1]
                search(tr)

            elif (tree[0].label()=="V" or tree[0].label == "TAKE") and tree[1].label()=="WHH" : 
                tr = tree[0]
                search(tr)
                tr = tree[1]
                search(tr)  
        
        elif len(tree)==3:
            if tree[0].label()=="TAKE" and tree[1].label()=="NP" and tree[2].label()=="PP":
                if len(tree[2])>0:             #先分析介词短语
                    if tree[2][0].label()=="P":
                        if len(tree[2][0])>0:
                            if tree[2][0][0].label()=="To":  #
                                xm_speech_res.action.append('go')
                                tr=tree[1]
                                search(tr)
                                print("go ")
                                tr=tree[2]
                                search(tr)
                                xm_speech_res.action.append('put')
                                xm_speech_res.target.append('here')
                                xm_speech_res.num += 3
                            elif tree[2][0][0].label()=="In":  #
                                print("go ")
                                xm_speech_res.action.insert(0,'go')
                                xm_speech_res.num+=1
                                tr=tree[2]
                                search(tr)
                                tr=tree[0]
                                search(tr)
                                tr=tree[1]
                                search(tr)
                            else :              #exp：tell the date of today;
                                tr=tree[0]      #形式为talk topic;忽略修饰
                                search(tr)
                                tr=tree[1]
                                search(tr)

            if (tree[0].label()=="V" or tree[0].label()!="TAKE" )and tree[1].label()=="NP" and tree[2].label()=="PP":
                if len(tree[2])>0:             #先分析介词短语
                    if tree[2][0].label()=="P":
                        if len(tree[2][0])>0:
                            if tree[2][0][0].label()=="To":  #
                                tr=tree[0]
                                search(tr)
                                tr=tree[1]
                                search(tr)
                                print("go ")
                                tr=tree[2]
                                search(tr)
                                xm_speech_res.action.append('put')
                                xm_speech_res.target.append('here')
                                xm_speech_res.num += 3
                            elif tree[2][0][0].label()=="In":  #
                                print("go ")
                                xm_speech_res.action.insert(0,'go')
                                xm_speech_res.num+=1
                                tr=tree[2]
                                search(tr)
                                tr=tree[0]
                                search(tr)
                                tr=tree[1]
                                search(tr)
                            else :              #exp：tell the date of today;
                                tr=tree[0]      #形式为talk topic;忽略修饰
                                search(tr)
                                tr=tree[2]
                                search(tr)        
                                 
                                                  
            elif tree[0].label()=="V" and tree[1].label()=="NP" and tree[2].label()=="NP": 
                tr=tree[0]
                search(tr)
                tr=tree[2]
                search(tr)            

            elif tree[0].label()=="V" and tree[1].label()=="PP" and tree[2].label()=="PP":
                if len(tree[2])>0:
                    if tree[2][0].label()=="P":
                        if len(tree[2][0])>0:
                            if tree[2][0][0].label()=="In":
                                print("go ")
                                xm_speech_res.action.insert(0,'go')
                                xm_speech_res.num+=1
                                tr=tree[2]
                                search(tr)
                                tr=tree[0]
                                search(tr)
                                tr=tree[1]
                                search(tr)
        
        elif len(tree) == 4:
            tr = tree[0]
            search(tr)
            tr = tree[3]
            search(tr)
            xm_speech_res.action.append('find') 
            tr = tree[2]
            search(tr)                  
            xm_speech_res.action.append('put')
            xm_speech_res.target.append('here')
            xm_speech_res.num += 3

        elif len(tree) == 5:
            tr = tree[0]
            search(tr)
            tr = tree[4]
            search(tr)
            xm_speech_res.action.append('find') 
            tr = tree[2]
            search(tr)
            xm_speech_res.action.append('put')
            xm_speech_res.target.append('here')
            xm_speech_res.num += 3
                

    if tree.label()=="NP":
        if len(tree)==1:
            tr=tree[0]
            search(tr)
        elif len(tree)==2:
            tr=tree[1]
            search(tr)
                		
    if tree.label()=="PP":
        if len(tree)==2:
            tr=tree[1]
            search(tr)
                                
    if tree.label()=="V":
        if len(tree)>0:
            if tree[0].label()=="Find":
                xm_speech_res.action.append('find')
                xm_speech_res.num += 1
                print("find")
            if tree[0].label()=="Go":
                xm_speech_res.action.append('go')
                xm_speech_res.num += 1
                print("go")
            if tree[0].label()=="Deliver":
                print( tree[0] )
                #if str(tree[0]) == "(Deliver deliver)":
                xm_speech_res.action.append('go')
                xm_speech_res.num += 1
                print("go")
            if tree[0].label()=="Talk":
                print( tree[0] )
                if str(tree[0]) == "(Talk answer)":
                    xm_speech_res.action.append('talk')
                    xm_speech_res.num+=1
                    print("talk")
                else:
                    xm_speech_res.action.append('tell')
                    xm_speech_res.num+=1
                    print("tell")
            if tree[0].label()=="Follow":
                xm_speech_res.action.append('follow')
                xm_speech_res.num += 1
                print( tree[0] )
                print("follow")
            if tree[0].label()=="Grasp":
                xm_speech_res.action.append('grasp')
                xm_speech_res.num += 1
                print("grasp")
            if tree[0].label()=="Remember":
                xm_speech_res.action.append('remember')
                xm_speech_res.num += 1
                print("remember")
    if tree.label()=="TAKE":
        xm_speech_res.action.append('grasp')
        xm_speech_res.num += 1
        print("grasp")

    if tree.label()=="PropN":
        if len(tree)>0:
            if tree[0] == "me":
                xm_speech_res.target.append('speaker')
                print("speaker")
            elif tree[0] == "it":
                print("person")
            else:
                xm_speech_res.target.append('person')
                print("person")
		
    if tree.label()=="Nom":
        if len(tree)==1:
            tr=tree[0]
            search(tr)
        if len(tree)==2:
            xm_speech_res.target.append('calling_people')


             
    if tree.label()=="N":
        if len(tree)==1:
            tr=tree[0]
            search(tr)
            
    if tree.label()=="Location":
        print(tree[0])
        if tree[0] == "bedroom":
            xm_speech_res.target.append('bedroom')
            print("bedroom")
        elif tree[0] == "livingroom":
            xm_speech_res.target.append('livingroom')
            print("livingroom")
        elif tree[0] == "kitchen":
            xm_speech_res.target.append('kitchen')
            print("kitchen")
        elif tree[0] == "here":
            xm_speech_res.target.append('speaker')
            print("speaker")
        elif tree[0] == "hallway":
            xm_speech_res.target.append('hallway')
        else:
            xm_speech_res.target.append(tree[0])
                

    if tree.label()=="Object":
        if len(tree)==1:
            if tree[0] in object:
                xm_speech_res.target.append(tree[0])
                print(tree[0])
	
    if tree.label()=="Person":
        if len(tree)==1:
            xm_speech_res.target.append('person')
            print("person")
	
    if tree.label()=="Topic":
        if len(tree)==1:
            tr=tree[0]
            search(tr)
            
    if tree.label()=="Time":
        if len(tree)==1:
            xm_speech_res.target.append('time')
            print(tree[0]+" ")
    if tree.label()=="Date":
        if tree[0] == 'tommorrow':
            xm_speech_res.target.append('tomo')
        else:
            xm_speech_res.target.append('date')
            print(tree[0]+" ")
    if tree.label()=="Name":
        if len(tree)==1 and tree == 'name':
            xm_speech_res.target.append('name')
            print(tree[0]+" ")
        else: xm_speech_res.target.append('team')
    if tree.label()=="Day":
        if len(tree)==1:
            xm_speech_res.target.append('day')
            print( tree[0] )
    if tree.label()=="Question":
        if len(tree)==1:
            if tree[0] == "question":
                xm_speech_res.target.append('question')
                print("question")
    if tree.label()=="WHH":
        print("1",len(tree),tree[1].label())
        if len(tree)== 4 and tree[1].label() == "N":
            print('here')
            tr = tree[1]
            search(tr)
        else :
            tr = tree[3]
            search(tr)
    if tree.label()=="Sentence":
        if len(tree)==1:
            xm_speech_res.target.append(sentences)
            print( tree[0] )


#-------------------------------------------------------------------------------
class XM(Recognizer):
    def __init__(self):
        """Class XM inherits from class Recognizer in speech_recognition"""

        Recognizer.__init__(self)

    def tts(self,text):
        tts_test = rospy.ServiceProxy('tts',xm_Speech_tts)
        tts_res = tts_test.call(text)
        if tts_res.flag == 1:
            subprocess.call(["play","tts_sample.wav"])
        else:
            subprocess.call(["espeak","-v","f3+en_us","-s","130",text])

    def isr(self,audio_site):
        isr_step = rospy.ServiceProxy('isr',xm_speech_isr)
        isr_res = isr_step.call(audio_site)
        raw_result = ''
        with codecs.open('/home/domistic/text.txt','r') as f: raw_result = f.read()
        result = re.findall(r'(<rawtext>)(.*)(</rawtext>)',raw_result)
        print(result)
        if len(result) != 1:
            print('get nothing')
            return ''
            print(result[0][1])
        if result[0][1] != "":
            return result[0][1]

    def iat(self,audio_site):
        iat_step = rospy.ServiceProxy('iat',xm_speech_iat)
        iat_res = iat_step.call(audio_site)
        result = ''
        with codecs.open('/home/domistic/text.txt','r') as f: result += f.read()
        print(result)
        if result == "":
            print('get nothing')
            return ''
        if result != '':
            return result


    def callback(self, text):
        """this function is to process the recognized text and send the processed information to Arduino which controls some devices"""
        global text_last
        global xm_speech_req
        global xm_speech_res
        global sentences
        #if len(text) != 0 :
         #   if text[28] == '=':
          #      text = text[29:]
           # elif text[29] == '=':
            #    text = text[30:]
        if text != "you are right" and text != "no" and len(text) != 0:
            text_last = text
        if type(text) == str and len(text) != 0:
            print("recognized : " + text)
            if xm_speech_req.command == 1:#Answer a question
                if text == "you are right":
                    print('question:  ' + text_last)
                    file_handle_c = open('/home/domistic/catkin_ws/src/xm_speech_for_linux/xm_speech/msc/bnf/c.txt',mode='rb')
                    file_handle_d = open('/home/domistic/catkin_ws/src/xm_speech_for_linux/xm_speech/msc/bnf/d.txt',mode='rb')
                    c_contents = file_handle_c.readlines()
                    d_contents = file_handle_d.readlines()
                    t = '\"' + text_last + '\"'+ '\n' 
                    if t in c_contents:
                        c_index = c_contents.index(t)
                        d = d_contents[c_index]
                        print('answer:  ' + d)
                        xm_speech_res.num = 1
                        self.tts(d)

            elif xm_speech_req.command == 2:#GPSR
                if text != "you are right" and text != "no" and text != "you can stop here":
                    if len(text_last) != 0:
                        self.tts(text_last)
                        self.tts("am i right")

                if text == "you are right":
                    if len(text_last) != 0:
                        self.tts("OK")
                        sen = text_last

                        if "tell the day of the  month" in sen:
                            sentences = "tell the day of the  month"
                            sen = sen.replace("tell the day of the  month","sentence")
                        if "tell the day of the  week" in sen:
                            sentences = "tell the day of the  week"
                            sen = sen.replace("tell the day of the  week","sentence")
                        if "tell the date" in sen:
                            sentences = "tell the date"
                            sen = sen.replace("tell the date","sentence")
                        for i in double_words:
                            sen = sen.replace(i.replace("_"," "),i)
                        #print(sen)
                        rd = RecursiveDescentParser(grammar)
                        t = rd.parse_one(sen.split())
                        #print(t)
                        search(t)
                        text_last = ''

    
            elif xm_speech_req.command == 3:#WhoIsWho
                if text != "you are right" and text != "no":
                    if len(text_last) != 0:
                        self.tts(text_last)
                    self.tts("am i right")   

                if text == "you are right":
                    if len(text_last) != 0:
                        print("k")
                        self.tts("OK")
                        print("text_last:"+text_last)
                        #name
                        for t in people:
                            if t in text_last:
                                xm_speech_res.name.append(t)
                                xm_speech_res.num += 1
                        for i in objectno:
                            if i in text_last:
                                xm_speech_res.object.append(i)
                                xm_speech_res.num += 1

                        text_last = ''
                
            elif xm_speech_req.command == 4:#SPR
                #print('question:   ' + text)
                file_handle_a = open('/home/domistic/catkin_ws/src/xm_speech_for_linux/xm_speech/msc/bnf/a.txt',mode='rb')
                file_handle_b = open('/home/domistic/catkin_ws/src/xm_speech_for_linux/xm_speech/msc/bnf/b.txt',mode='rb')
                a_contents = file_handle_a.readlines()
                b_contents = file_handle_b.readlines()
                t = text+'?\n'
                if t == a_contents[0]:
                    print('finally')
                if t in a_contents:
                    a_index = a_contents.index(t)
                    b = b_contents[a_index]
                    xm_speech_res.answer=b
                    print('answer:   ' + b)
                    xm_speech_res.num = 1
                    self.tts(b)
                elif text == "number of people standing":
                    xm_speech_res.num = 2
                    time.sleep(1)
                elif text == "number of people sitting":
                    xm_speech_res.num = 3
                    time.sleep(1)
                elif text == "what is the number of the people who waving arms":
                    xm_speech_res.num = 4
                    time.sleep(1)
            elif(xm_speech_req.command == 5):    #help me carry   
                if text != "you are right" and text != "no":
                    if len(text_last) != 0:
                        self.tts(text_last)
                    self.tts("am i right")

                if text == "you are right":
                    if len(text_last) != 0:

                        if text_last.find("stop") >= 0:
                            xm_speech_res.action.append('stop')
                        elif text_last.find("follow") >= 0:
                            xm_speech_res.action.append('follow')
                        elif text_last.find("take") >= 0:
                            xm_speech_res.action.append('take')

                        if text_last.find("kitchen") >= 0:
                            xm_speech_res.target.append('kitchen')
                        elif text_last.find("livingroom") >= 0:
                            xm_speech_res.target.append('livingroom')
                        elif text_last.find("bedroom") >= 0:
                            xm_speech_res.target.append('bedroom')
                        elif text_last.find("diningroom") >= 0:
                            xm_speech_res.target.append('diningroom')
                            
                        xm_speech_res.num = 1 
                        print("text_last:"+text_last)
                        text_last = ''
                if text == "you can stop here":
                    xm_speech_res.action.append('stop')

            
            elif(xm_speech_req.command == 6):       #shopping
                if text != "you are right" and text != "no":
                    if len(text_last) != 0:
                        self.tts(text_last)
                    self.tts("am i right")    

                if text == "you are right":
                    if len(text_last) != 0:
                        self.tts("OK")
                        print("text_last:"+text_last)
                        if text_last == "you can stop here":
                            xm_speech_res.action.append('stop')
                            xm_speech_res.num += 1

                        
                        #follow
                        elif text_last.find('follow') >= 0:
                            xm_speech_res.action.append('follow')
                            xm_speech_res.num += 1
                        else:
                            for i in objectno:
                                if i in text_last:
                                    xm_speech_res.object.append(i)
                                    xm_speech_res.num += 1

                        #方向
                        if text_last.find('right') >= 0:
                            xm_speech_res.target.append('right')
                            xm_speech_res.num += 1 
                        if text_last.find('left') >= 0:
                            xm_speech_res.target.append('left')
                            xm_speech_res.num += 1
                        if text_last.find('front') >= 0:
                            xm_speech_res.target.append('front')
                            xm_speech_res.num += 1
                        text_last = ''

    def save_wav_file(self,wav_file, audio, convert_rate=None, convert_width=None):
            #audio = wav_file.open()
            raw_data = audio.get_raw_data(convert_rate, convert_width)
            sample_rate = audio.sample_rate if convert_rate is None else convert_rate
            sample_width = audio.sample_width if convert_width is None else convert_width

            wav_writer = wave.open(wav_file, "wb")
            try: 
                wav_writer.setframerate(sample_rate)
                wav_writer.setsampwidth(sample_width)
                wav_writer.setnchannels(1)
                wav_writer.writeframes(raw_data)
            finally: 
                wav_writer.close()
            return wav_file

    def auto_listen(self, source, phrase_time_limit=None):
            assert isinstance(source, AudioSource), "Source must be an audio source"

            running = [True]
            timee = time.strftime('%m-%d-%H:%M',time.localtime(time.time()))
            t = '/home/domistic/catkin_ws/src/xm_speech_for_linux/xm_speech/msc/wavewv/' + timee + '.wav'
            try:
                print("a moment of silence, please...")
                with source as s:
                    self.adjust_for_ambient_noise(s) # we only need to calibrate once, before we start listening

                while pause == 0:
                        #try:  # listen for 1 second, then check again if the stop function has been called
                    r = Recognizer()
                    
                    subprocess.call(["play","/home/domistic/catkin_ws/src/xm_speech_for_linux/xm_speech/scripts/ring1.wav"])
                    print("listen")
                    
                    with source as s:audio = r.listen(s,None,5,None)
                    print("Got it! Now to recognize it...")
                
                    self.save_wav_file(t,audio)			
                    isr_result=self.isr(t)
                    print(isr_result)
                    if pause == 0: 
                        self.callback(isr_result)
			#time.sleep(3)
            except KeyboardInterrupt:
                running[0] = False

	
def xm_callback():
    global xm_speech_req
    if xm_speech_req.command >0 and xm_speech_req.command <=6:
        xm = XM()
        m = Microphone(sample_rate = 16000, chunk_size = 1024)
        xm.auto_listen(m)

def runiat(req):#req
    #ret = build_grammar(asr_data)
    global xm_speech_req
    global xm_speech_res
    global pause
    xm_speech_req = req
    
    running=[True]
    try:
        while running[0]:
            xm_speech_res.num = 0
            xm_speech_res.action = []
            xm_speech_res.target = []
            xm_speech_res.name = []
            xm_speech_res.object = []
            pause = 0
            while xm_speech_res.num == 0 :
                time.sleep(0.5)
                if xm_speech_res.num !=0:
                    pause = 1
                    '''if xm_speech_req.command == 2:
                        if xm_speech_res.action[2] == 'grasp':
                            if xm_speech_res.target[2] == 'livingroom' or xm_speech_res.target[2] == 'kitchen' or xm_speech_res.target[2] == 'diningroom' or xm_speech_res.target[2] == 'bedroom':
                                xm_speech_res.action[2] = 'go'
                                if xm_speech_res.num ==2:
                                    xm_speech_res.num += 1'''
                    xm_speech_req.command = -1
                    return xm_speech_res
    except KeyboardInterrupt:
        running[0] = False


#------------------------------------------------------------------------------

def xm_speech_demo ():

    rospy.init_node('xm_speech_demo')
    rospy.logwarn('FUck zjx')
    s = rospy.Service('xm_speech_meaning', xm_Speech_meaning, runiat)
    global pause
    global xm_speech_res
    running=[True]
    try:
        while running[0]:
            if pause == 0 :
                xm_callback()
    except KeyboardInterrupt:
        running[0] = False
    print(xm_speech_res)
    rospy.spin()

#------------------------------------------------------------------------------

if __name__ == "__main__":
    xm_speech_demo()    

#------------------------------------------------------------------------------

