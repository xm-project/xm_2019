#!/usr/bin/env python
#encoding:utf8
import rospy
from smach import State, StateMachine,UserData
from xm_msgs.srv import *
from xm_msgs.msg import *
import math
import tf
from geometry_msgs.msg import *
from control_msgs.msg import *
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level
from xm_arm_moveit_level import xm_arm_moveit_level, xm_arm_moveit_name,xm_gripper
from std_srvs.srv import Empty,EmptyRequest
import subprocess
from copy import deepcopy


class xm_arm_smach():
    def __init__(self):
        rospy.init_node('xm_arm_smach')
        rospy.on_shutdown(self.shutdown)
       
        xm_arm_moveit_name('nav_pose')
        rospy.logwarn('nav_pose!')
        rospy.sleep(2)

        xm_gripper('kai')
        rospy.logwarn('kai!')
        rospy.sleep(2)
        
        xm_gripper('guan')
        rospy.logwarn('guan!')
        rospy.sleep(2)

        '''
        xm_arm_moveit_name('nav_right')
        rospy.logwarn('turn_right!')
        rospy.sleep(2)

        xm_arm_moveit_name('na_guoqi')
        rospy.logwarn('naguoqi!')
        rospy.sleep(2)

        xm_gripper()
        rospy.logwarn('kai!')
        rospy.sleep(2)

        #xm_gripper_moveit_name('guan')
        rospy.logwarn('guan!')
        rospy.sleep(2)

        xm_arm_moveit_name('miandui')
        rospy.logwarn('miandui!')
        rospy.sleep(2)

        xm_arm_moveit_name('zhuan')
        rospy.logwarn('zhuan!')
        rospy.sleep(2)

        #xm_finger_moveit_name('kai')
        rospy.logwarn('kai!')
        rospy.sleep(2)

        xm_arm_moveit_name('salute')
        rospy.logwarn('salute!')
        rospy.sleep(2)
'''
    def shutdown(self):
        rospy.logwarn('byebye^_^')


if __name__ =="__main__":
    try:
        xm_arm_smach()
    except KeyboardInterrupt:
        pass
