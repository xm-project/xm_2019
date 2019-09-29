
#! /usr/bin/env/ python
# encoding:utf8
import rospy
from smach import State,UserData,StateMachine
from xm_msgs.srv import *
from geometry_msgs.msg import *
from xm_msgs.msg import *
from math import *
import subprocess
from xm_smach.common_lib import *
import tf
from xm_smach.target_gpsr import *
from xm_arm_nav.xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level
from xm_arm_nav.xm_arm_moveit_level import *


class simple_turn(State):
    def __init__(self):
        State.__init__(outcome=['succeeded', 'aborted', 'preempted'])
        self.cmd_vel = rospy.Publisher(
        '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.twist_xm = Twist()

    def execute(self , userdata):
        if self.preempt_requested():
            return 'preempt'

        self.twist_xm.angular.z = 3.1415/2

        try:
            self.cmd_vel.publish(self.twist_xm)
            return 'succeeded'
        except Exception,e:
            rospy.logerr(e)
            rospy.logerr("Simple_Turn Error")
            return 'aborted'
