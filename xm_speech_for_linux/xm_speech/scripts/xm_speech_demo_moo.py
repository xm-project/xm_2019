#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Created on Thu Jan 19 09:37:01 2017
altered on 2018& 2019

@author: fanfan

This project is to build an interesting chatting robot and aims to serve intelligent home
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

text_last = ''

grammar=CFG.fromstring("""
S ->Mission1|Mission1 Mission2|Mission1 Mission2 Mission3

Mission1 ->Val VP
Mission2 ->Val VP
Mission3 ->Val VP

VP ->V NP|V NP PP|V PP|V NP NP|V PP PP|V NP NP PP
NP ->Det Nom|PropN
Nom ->Adj N|N
PP ->P NP

Val ->"and"|"then"|
Det ->"a"|"an"|"the"|"your"|"my"|
PropN ->"her"|"him"|"me"|"it"
P ->To|In|For|Of
Adj ->"waving"

N ->Location|Object|Topic|Person
V ->Find|Go|Deliver|Talk|Follow|Grasp|Remember|Stop


Location ->"kitchen"|"diningroom"|"bedroom"|"livingroom"|"here"
Object ->"cola"|"milk"|"noodles"|"biscuit"|"chips"|"rollpaper"|"toothpaste"|"soap"|"water"|"coffee"|"cake"|" Beer "|" Ice tea "
Person ->"person"|"me"|"her"|"Tom"|"Paul"|"Green"|"Kevin"|"Tracy"|"John"|"Angel"|"Jamie"|"Fisher"|"Shirley"|"Robin"|"Daniel"|"Saber"
Topic ->Time|Date|Name|Day|Question
Time ->"time"|"what time is it"
Date ->"date"|"today"|"tomorrow"
Name ->"name"|"team"
Day ->"day"|"week"|"month"|"whatdayistoday"
Question ->"question"


Find ->"find"|"look"|"search"
Go ->"go"|"navigate"|"reach"|"go back"
Deliver ->"deliver"|"bring"|"carry"
Talk ->"tell"|"say"|"speak"|"ask"|"answer"
Follow ->"follow"
Grasp ->"grasp"|"get"|"take"|"put"
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
            if tree[0].label()=="V" and tree[1].label()=="NP":
                tr=tree[0]
                search(tr)
                tr=tree[1]
                search(tr)
                
            if tree[0].label()=="V" and tree[1].label()=="PP":
                tr=tree[0]
                search(tr)
                tr=tree[1]
                search(tr)
        
        elif len(tree)==3:
            if tree[0].label()=="V" and tree[1].label()=="NP" and tree[2].label()=="PP":
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
                                 
                                                  
            elif tree[0].label()=="V" and tree[1].label()=="NP" and tree[2].label()=="NP": #
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
            tr = tree[1]
            search(tr)                    
            tr = tree[2]
            search(tr)
            tr = tree[3]
            search(tr) 
                

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

    if tree.label()=="PropN":
        if len(tree)>0:
            if tree[0] == "me":
                xm_speech_res.target.append('speaker')
                print("speaker")
		
    if tree.label()=="Nom":
        if len(tree)==1:
            tr=tree[0]
            search(tr)
        if len(tree)==2:
            tr=tree[1]
            search(tr)#先不做形容词
             
    if tree.label()=="N":
        if len(tree)==1:
            tr=tree[0]
            search(tr)
            
    if tree.label()=="Location":
        if len(tree)==1:
            if tree[0] == "bedroom":
                xm_speech_res.target.append('bedroom')
                print("bedroom")
            if tree[0] == "diningroom":
                xm_speech_res.target.append('diningroom')
                print("diningroom")
            if tree[0] == "livingroom":
                xm_speech_res.target.append('livingroom')
                print("livingroom")
            if tree[0] == "kitchen":
                xm_speech_res.target.append('kitchen')
                print("kitchen")
            if tree[0] == "here":
                xm_speech_res.target.append('speaker')
                print("speaker")            

    if tree.label()=="Object":
        if len(tree)==1:
            if tree[0] == "cola":
                xm_speech_res.target.append('cola')
                print("cola")
            elif tree[0] == "coffee":
                xm_speech_res.target.append('coffee')
                print("coffee")
            elif tree[0] == "water":
                xm_speech_res.target.append('water')
                print("water")            
            elif tree[0] == "milk":
                xm_speech_res.target.append('milk')
                print("milk")
            elif tree[0] == "cake":
                xm_speech_res.target.append('cake')
                print("cake")
            elif tree[0] == "noodles":
                xm_speech_res.target.append('noodles')
                print("noodles")
            elif tree[0] == "bicuit":
                xm_speech_res.target.append('biscuit')
                print("biscuit")
            elif tree[0] == "rollpaper":
                xm_speech_res.target.append('roll paper')
                print("roll paper")
            elif tree[0] == "toothpaste":
                xm_speech_res.target.append('toothpaste')
                print("toothpaste")
            elif tree[0] == "chips":
                xm_speech_res.target.append('chips')
                print("chips")
            elif tree[0] == "soap":
                xm_speech_res.target.append('soap')
                print("soap")              
	
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
            print(tree[0]+" ")
    if tree.label()=="Date":
        if len(tree)==1:
            print(tree[0]+" ")
    if tree.label()=="Name":
        if len(tree)==1:
            print(tree[0]+" ")
    if tree.label()=="Day":
        if len(tree)==1:
            print( tree[0] )
            if tree[0] == "whatdayistoday":
                xm_speech_res.target.append('monday')
                print("whatdayistoday")
    if tree.label()=="Question":
        if len(tree)==1:
            if tree[0] == "question":
                xm_speech_res.target.append('question')
                print("question")


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
        with codecs.open('text.txt','r') as f: raw_result = f.read()
        result = re.findall(r'(<rawtext>)(.*)(</rawtext>)',raw_result)
	print(result)
        if len(result) != 1:
		print('get nothing')
            	return ''
	print(result[0][1])
        if result[0][1] != "":
		        return result[0][1]

    def callback(self, text):
        """this function is to process the recognized text and send the processed information to Arduino which controls some devices"""
        global text_last
        global xm_speech_req
        global xm_speech_res
        #if len(text) != 0 :
         #   if text[28] == '=':
          #      text = text[29:]
           # elif text[29] == '=':
            #    text = text[30:]
        if text != "you are right" and text != "no" and len(text) != 0:
            text_last = text
	    print(text)
	    t=type(text)
	    print(t)
        if type(text) == str and len(text) != 0:
            print("recognized : " + text)
            if xm_speech_req.command == 1:#Answer a question
                if text == "what is the capital of china":
                    print("Beijing")
                    self.tts("Beijing")
                    xm_speech_res.num = 1
                elif text == "how many hours in a day":
                    print("twenty four")
                    self.tts("twenty four")
                    xm_speech_res.num = 1
                elif text == "how many season are there in one year":
                    print("four")
                    self.tts("four")
                    xm_speech_res.num = 1
                elif text == "how many seconds in one minute":
                    print("sixty")
                    self.tts("sixty")
                    xm_speech_res.num = 1
                elif text == "what is the world biggest island":
                    print("green land")
                    self.tts("green land")
                    xm_speech_res.num = 1
                elif text == "what is the biggest province of china":
                    print("xin jiang")
                    self.tts("xin jiang")
                    xm_speech_res.num = 1
                elif text == "how large is the area of china":
                    print("nine million and six hundred thousand saquare kilometers")
                    self.tts("nine million and six hundred thousand saquare kilometers")
                    xm_speech_res.num = 1
                elif text == "who was the first president of the usa":
                    print("george washington")
                    self.tts("george washington")
                    xm_speech_res.num = 1
                elif text == "what is china's national animal":
                    print("panda")
                    self.tts("panda")
                    xm_speech_res.num = 1
                elif text == "how many children did queen victoria have":
                    print("nine children")
                    self.tts("nine children")
                    xm_speech_res.num = 1
                elif text == "what was the former name of new york":
                    print("new amsterdam")
                    self.tts("new amsterdam")
                    xm_speech_res.num = 1

            elif xm_speech_req.command == 2:#GPSR
                if text != "you are right" and text != "no" and text != "stop here now":
                    if len(text_last) != 0:
                        self.tts(text_last)
                    	self.tts("am i right") 
                       
                if text == "you are right":
                    if len(text_last) != 0:
			self.tts("OK")
                        sen = text_last
                        rd = RecursiveDescentParser(grammar)
                        t = rd.parse_one(sen.split())
                        search(t)
                elif text == "stop here now":
                    xm_speech_res.action.append('stop')
                    xm_speech_res.num = 1
                    self.tts("OK")
    
            elif xm_speech_req.command == 3:#WhoIsWho
                if text != "you are right" and text != "no":
                    if len(text_last) != 0:
                        self.tts(text_last)
                    self.tts("am i right")   

                if text == "you are right":
                    if len(text_last) != 0:
                        self.tts("OK")
                        print("text_last:"+text_last)
                        #name
                        if text_last.find('Tom') >= 0:
                            xm_speech_res.name.append('Tom')
                            xm_speech_res.num += 1
                        elif text_last.find('Micheal') >= 0:
                            xm_speech_res.name.append('Micheal')
                            xm_speech_res.num += 1
                        elif text_last.find('Jack') >= 0:
                            xm_speech_res.name.append('Jack')
                            xm_speech_res.num += 1
                        elif text_last.find('Kevin') >= 0:
                            xm_speech_res.name.append('Kevin')
                            xm_speech_res.num += 1
                        elif text_last.find('Rose') >= 0:
                            xm_speech_res.name.append('Rose')
                            xm_speech_res.num += 1
                        elif text_last.find('John') >= 0:
                            xm_speech_res.name.append('John')
                            xm_speech_res.num += 1
                        elif text_last.find('Mary') >= 0:
                            xm_speech_res.name.append('Mary')
                            xm_speech_res.num += 1
                        elif text_last.find('Fisher') >= 0:
                            xm_speech_res.name.append('Fisher')
                            xm_speech_res.num += 1
                        elif text_last.find('Adam') >= 0:
                            xm_speech_res.name.append('Adam')
                            xm_speech_res.num += 1
                        elif text_last.find('Daniel') >= 0:
                            xm_speech_res.name.append('Daniel')
                            xm_speech_res.num += 1
                        #object
                        if text_last.find('ice tea') >= 0:
                            xm_speech_res.object.append('ice tea')
                            xm_speech_res.num += 1
                        elif text_last.find('safeguard') >= 0:
                            xm_speech_res.object.append('safeguard')
                            xm_speech_res.num += 1
                        elif text_last.find('napkin') >= 0:
                            xm_speech_res.object.append('napkin')
                            xm_speech_res.num += 1
                        elif text_last.find('chips') >= 0:
                            xm_speech_res.object.append('chips')
                            xm_speech_res.num += 1
                        elif text_last.find('laoganma') >= 0:
                            xm_speech_res.object.append('laoganma')
                            xm_speech_res.num += 1
                        elif text_last.find('porridge') >= 0:
                            xm_speech_res.object.append('porridge')
                            xm_speech_res.num += 1
                        elif text_last.find('milk') >= 0:
                            xm_speech_res.object.append('milk')
                            xm_speech_res.num += 1
                        elif text_last.find('water') >= 0:
                            xm_speech_res.object.append('water')
                            xm_speech_res.num += 1
                        elif text_last.find('cola') >= 0:
                            xm_speech_res.object.append('cola')
                            xm_speech_res.num += 1 
                        elif text_last.find('sprite') >= 0:
                            xm_speech_res.object.append('sprite')
                            xm_speech_res.num += 1 

            elif xm_speech_req.command == 4:#SPR
                print(text)
                #ArenaQ
                if text == "How many bookshelf are in the living room":
                    xm_speech_res.num = 1
                    self.tts("three")
                elif text == "In which room is the fridge":
                    xm_speech_res.num = 1
                    self.tts("kitchen")
                elif text == "How many doors has the Arena":
                    xm_speech_res.num = 1
                    self.tts("2")
                elif text == "In which room is the dining table":
                    xm_speech_res.num = 1
                    tself.tts("diningroom")
                elif text == "Where is located the bed":
                    xm_speech_res.num = 1
                    self.tts("bedroom")
                elif text == "How many fireplace are in the bedroom":
                    xm_speech_res.num = 1
                    self.tts("zero")
                elif text == "In which room is the sofa":
                    xm_speech_res.num = 1
                    self.tts("livingroom")
                elif text == "Where is located the fridge":
                    xm_speech_res.num = 1
                    self.tts("kitchen")
                elif text == "Where is located the cupboard":
                    xm_speech_res.num = 1
                    self.tts("diningroom")
                elif text == "In which room is the bookshelf":
                    xm_speech_res.num = 1
                    self.tts("bedroom")

                #ObjectQ
                elif text == "how many hours in a day":
                    xm_speech_res.num = 1
                    self.tts("twenty four")
                elif text == "how many season are there in one year":
                    xm_speech_res.num = 1
                    self.tts("four")
                elif text == "how many seconds in one minute":
                    xm_speech_res.num = 1
                    self.tts("sixty")
                elif text == "what is the world biggest island":
                    xm_speech_res.num = 1
                    self.tts("green land")
                elif text == "what is the biggest province of china":
                    xm_speech_res.num = 1
                    self.tts("shin junk")
                elif text == "how large is the area of china":
                    xm_speech_res.num = 1
                    self.tts("Nine million and six hundred thousand saquare kilometers")
                elif text == "Who was the first president of the USA":
                    xm_speech_res.num = 1
                    self.tts("George Washington")
                elif text == "What is China's national animal":
                    xm_speech_res.num = 1
                    self.tts("panda")
                elif text == "How many children did Queen Victoria have":
                    xm_speech_res.num = 1
                    self.tts("Nine children")
                elif text == "What was the former name of New York":
                    xm_speech_res.num = 1
                    self.tts("New Amsterdam")

                #PredefinedQ
                elif text == "How many people live in the Japan":
                    xm_speech_res.num = 1
                    self.tts("A little over 80 million")
                elif text == "what is the capital of the united states":
                    xm_speech_res.num = 1
                    self.tts("Washington DC")
                elif text == "what day is today":
                    xm_speech_res.num = 1
                    self.tts("Saturday")
                elif text == "What city is the capital of the Japan":
                    xm_speech_res.num = 1
                    self.tts("Tokyo")
                elif text == "Where does the term computer bug come from":
                    xm_speech_res.num = 1
                    self.tts("From a moth trapped in a relay")
                elif text == "When was invented the B programming language":
                    xm_speech_res.num = 1
                    self.tts("B was developed circa 1969 at Bell Labs")
                elif text == "Who invented the C programming language":
                    xm_speech_res.num = 1
                    self.tts("Ken Thompson and Dennis Ritchie")
                elif text == "What is the highest point in Japan":
                    xm_speech_res.num = 1
                    self.tts("The highest point in Japan is Mount Fuji")
                elif text == "What are the colours of the Japanese flag":
                    xm_speech_res.num = 1
                    self.tts("Japanese flag is a red circle centred over white")
                elif text == "Who invented the first compiler":
                    xm_speech_res.num = 1
                    self.tts("Grace Brewster Murray Hopper invented it")

                #CrowdOperatorQ
                '''elif text == "number of male people sitting":
                    xm_speech_res.num = 2
                    time.sleep(1)
                elif text == "number of female people sitting":
                    xm_speech_res.num = 3
                    time.sleep(1)
                elif text == "number of male people standing":
                    xm_speech_res.num = 4
                    time.sleep(1)
                elif text == "number of female people standing":
                    xm_speech_res.num = 5
                    time.sleep(1)
                elif text == "number of people standing":
                    xm_speech_res.num = 6
                    time.sleep(1)
                elif text == "number of people sitting":
                    xm_speech_res.num = 7
                    time.sleep(1)
		        '''
                if text == 'number of male people':
                    xm_speech_res.num = 2
                    time.sleep(1)
                if text == 'number of female people':
                    xm_speech_res.num = 3
                    time.sleep(1)
                if text == 'number of people':
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

			if text_last.find("bag") >= 0:
                            xm_speech_res.object.append('bag')

                        self.tts("OK")
                            
                        xm_speech_res.num = 1 
                        print("text_last:"+text_last)

            
            elif(xm_speech_req.command == 6):       #shopping
                if text != "you are right" and text != "no":
                    if len(text_last) != 0:
                        self.tts(text_last)
                    self.tts("am i right")    

                if text == "you are right":
                    if len(text_last) != 0:
                        self.tts("OK")
                        print("text_last:"+text_last)

                        num = 0
                        #follow
                        if text_last.find('follow') >= 0:
                            xm_speech_res.action.append('follow')
                            num += 1
                        
                        #object
                        if text_last.find('ice tea') >= 0:
                            xm_speech_res.object.append('ice tea')
                            num += 1
                        if text_last.find('safeguard') >= 0:
                            xm_speech_res.object.append('safeguard')
                            num += 1
                        if text_last.find('napkin') >= 0:
                            xm_speech_res.object.append('napkin')
                            num += 1
                        if text_last.find('chips') >= 0:
                            xm_speech_res.object.append('chips')
                            num += 1
                        if text_last.find('laoganma') >= 0:
                            xm_speech_res.object.append('laoganma')
                            num += 1
                        if text_last.find('milk') >= 0:
                            xm_speech_res.object.append('milk')
                            num += 1
                        if text_last.find('water') >= 0:
                            xm_speech_res.object.append('water')
                            num += 1
                        if text_last.find('cola') >= 0:
                            xm_speech_res.object.append('cola')
                            num += 1
                        if text_last.find('porridge') >= 0:
                            xm_speech_res.object.append('porridge')
                            num += 1 
                        if text_last.find('sprite') >= 0:
                            xm_speech_res.object.append('sprite')
                            num += 1
                        #方向
                        if text_last.find('right') >= 0:
                            xm_speech_res.target.append('right')
                            num += 1 
                        if text_last.find('left') >= 0:
                            xm_speech_res.target.append('left')
                            num += 1

                        xm_speech_res.num = num

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
            try:
                print("a moment of silence, please...")
                with source as s:
                    self.adjust_for_ambient_noise(s) # we only need to calibrate once, before we start listening

                while pause == 0:
                        #try:  # listen for 1 second, then check again if the stop function has been called
                    r = Recognizer()
                    print("listen")
                    with source as s:audio = r.listen(s)
                    print("Got it! Now to recognize it...")
                
                    self.save_wav_file('/home/domestic/catkin_ws/src/xm_speech_for_linux/xm_speech/msc/wave/cc.wav',audio)			
                    isr_result=self.isr('/home/domestic/catkin_ws/src/xm_speech_for_linux/xm_speech/msc/wave/cc.wav')
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

    rospy.spin()

#------------------------------------------------------------------------------

if __name__ == "__main__":
    xm_speech_demo()    

#------------------------------------------------------------------------------

