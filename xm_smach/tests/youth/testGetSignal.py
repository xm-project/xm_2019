#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.shopping_lib import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
from new_lib.basic_voice import *
import math
import subprocess


class test():
    def __init__(self):
        rospy.init_node('go_and_follow')
        self.Test = StateMachine(outcomes=['succeeded', 'aborted', 'error'])
        self.Test.userdata.PT_LIST = {}
        self.Test.userdata.mission = {}
        with self.Test:
            self.Test.userdata.targets = list()
            self.Test.userdata.actions = list()
            self.Test.userdata.people_condition = {}
            self.Test.userdata.command = 2
            # StateMachine.add('GET_TARGET',
            #                  CheckStop2(),
            #                  transitions={'stop': 'succeeded', 'aborted': 'GET_TARGET', 'remeber':'GET_TARGET'})
            
            StateMachine.add('GET_TASK', GeneralAnswer(), transitions={'succeeded': 'GET_TASK',
                                                                       'aborted': 'aborted'},
                             remapping={'targets': 'targets',
                                        'actions': 'actions',
                                        #'task_num':'tasksNum'
                                        })


   
        self.Test.execute()
   
if __name__ == "__main__":
    test()
