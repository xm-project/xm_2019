#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.speech_reco_lib import *
from subprocess import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math
from math import pi


class turn():
    def __init__(self):
        rospy.init_node('turn')
        rospy.logwarn('ssssss')
        self.Turn = StateMachine( outcomes = ['succeeded', 'aborted', 'error'])
        
        with self.Turn:
            self.Turn.userdata.degree = pi
            self.Turn.userdata.sentences = 'I want play a riddle game'
            StateMachine.add('SPEAK' ,
                                Speak() , 
                                transitions = {'succeeded':'turn' , 'aborted':'aborted','error':'error'},
                                remapping = {'sentences':'sentences'})
            StateMachine.add('turn',
                                TurnDegree(),
                            	transitions ={'succeeded':'succeeded' , 'aborted':'aborted' , 'error':'error'} ,
                            	remapping = {'degree':'degree'})
            

                    
        
        self.Turn.execute()


if __name__ == "__main__":
    try:
	#rospy.logerr('F')
        turn()
    except:
        rospy.logerr('test wrong!')

