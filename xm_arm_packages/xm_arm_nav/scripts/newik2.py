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
from xm_arm_moveit_level import xm_arm_moveit_level, xm_arm_moveit_name
from std_srvs.srv import Empty,EmptyRequest
import subprocess
from copy import deepcopy

class DirectIK(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','error'],
                        input_keys = ['arm_ps','arm_mode','fgm'])
        self.tf_listener = tf.TransformListener()

    def execute(self,userdata):
        try:
            getattr(userdata,'arm_ps')
            getattr(userdata,'arm_mode')
	    getattr(userdata,'fgm')
        except:
            rospy.logerr('No params specified!')
            return 'error'
        if userdata.arm_mode is 0:
            #xm_arm_moveit_name('nav_pose')
            return 'succeeded'
        else:
            pass
        
        gripper_service_level(True)
        base_ps = self.tf_listener.transformPoint('base_link',userdata.arm_ps)
        solution_list=list()
        solution_list1=list()
        self.haha=0
	rospy.logerr(userdata.fgm)
        if userdata.fgm==2:
            xm_arm_moveit_name('nav_p')
            lifting_value = 0.02
            solution_list1.append(0.02)
            solution_list1.append(0.0)
            solution_list1.append(-0.3796)
            solution_list1.append(1.4308)
            solution_list1.append(0.6768)
            solution_list1.append(0.0)
            lifting_controller_level(lifting_value)
            joint_value = solution_list1[1:6]
            arm_controller_level(joint_value)
            rospy.sleep(3)
        
        solution_list.append(base_ps.point.z-0.66)
        solution_list.append(math.atan2(base_ps.point.y, 0.85))
        distance = math.sqrt(base_ps.point.y*base_ps.point.y + base_ps.point.x * base_ps.point.x)
        if base_ps.point.x>=1.05:
            distance -= 0.70
        elif base_ps.point.x>=0.99:
            distance -= 0.63
        elif base_ps.point.x>=0.93:
            distance -= 0.58
        else:
            distance -= 0.53
        solution_list.append((-1)*math.acos((distance*distance + 0.28*0.28 - 0.15*0.15)/2/distance/0.28))
            #solution_list.append((-1)*solution_list[2]+math.acos((distance*distance + 0.15*0.15 - 0.28*0.28)/2/distance/0.15))
        solution_list.append(3.14-math.acos(((-1)*distance*distance + 0.15*0.15 + 0.28*0.28)/2/0.28/0.15))
        solution_list.append((-1)*math.acos((distance*distance + 0.15*0.15 - 0.28*0.28)/2/distance/0.15))
        solution_list.append(0.0)
        lifting_value = solution_list[0]
        joint_value = solution_list[1:6]
        lifting_controller_level(lifting_value)
        joint_value_turn = solution_list[1:6]
        joint_value_turn[1] = -1.56999
        joint_value_turn[2] = 1.61619
        joint_value_turn[3] = 1.5492333
        joint_value_turn[4] = 0.0
        #arm_controller_level(joint_value_turn)
        #rospy.sleep(2)
        arm_controller_level(joint_value)
        rospy.sleep(10)
        gripper_service_level(False)
            #0.000261899782345 -1.56999063492 1.6161891222 1.54923319817 0.0

        lifting_value = solution_list[0]+0.1
        lifting_controller_level(lifting_value)
        rospy.sleep(2)
        joint_value[1]=-1
        joint_value[2]=1.4
        joint_value[3]=1
        arm_controller_level(joint_value)
        rospy.sleep(3)
        joint_value_turn[1] = -0.3968
        joint_value_turn[2] = 0.9462
        joint_value_turn[3] = 1.4022
        joint_value_turn[4] = 0.0
        #arm_controller_level(joint_value_turn)      
        rospy.sleep(3)
        lifting_controller_level(-0.0568)
        joints3 = [0,-1.4700, 1.3692, 1.5472, 0]
        arm_controller_level(joints3)
        xm_arm_moveit_name('nav_pose')#抓完之后的位置
        return 'succeeded'

class xm_arm_smach():
    def __init__(self):
        rospy.init_node('xm_arm_smach')
        rospy.on_shutdown(self.shutdown)
        xm_arm_moveit_name('nav_pose')
        rospy.logerr('Link Start!')
        self.arm_stack_service  = rospy.Service('arm_stack',xm_PickOrPlace,self.callback)
        self.sm_ArmStack = StateMachine(outcomes=['succeeded','error'],
                                            input_keys=['arm_point','arm_mode','fgm'])
        with self.sm_ArmStack:
            StateMachine.add('PICK',
                                DirectIK(),
                                remapping={'arm_ps':'arm_point','arm_mode':'arm_mode','fgm':'fgm'},
                                transitions={'succeeded':'succeeded','error':'error'})
            rospy.spin()
    
    def shutdown(self):
        rospy.logwarn('byebye^_^')

    def callback(self,req):
        self.arm_point = req.goal_position
        self.arm_mode  = req.action
	self.fgm = req.fgm
        sm_result = self.sm_ArmStack.execute(parent_ud ={'arm_point':self.arm_point,'arm_mode':self.arm_mode,'fgm':self.fgm})
        res = xm_PickOrPlaceResponse()        
        if sm_result == 'succeeded':
            rospy.logwarn('arm_stack succeeded')
            res.result =True
        else:
            rospy.logwarn('moveit failed, please justfy the position of robot')
            res.result =False
        return res

if __name__ =="__main__":
    try:
        xm_arm_smach()
    except KeyboardInterrupt:
        pass
