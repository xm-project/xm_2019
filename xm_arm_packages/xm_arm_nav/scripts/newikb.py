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
                        input_keys = ['arm_ps','arm_mode'])
        self.tf_listener = tf.TransformListener()

    def execute(self,userdata):
        try:
            getattr(userdata,'arm_ps')
            getattr(userdata,'arm_mode')
        except:
            rospy.logerr('No params specified!')
            return 'error'
        if userdata.arm_mode is 0:
            xm_arm_moveit_name('nav_poseb')
            return 'succeeded'
        else:
            pass
        if userdata.arm_mode is 1:
            gripper_service_level(True)
	gripper_service_level(True)
        xm_arm_moveit_name('nav_poseb')
        base_ps = self.tf_listener.transformPoint('base_link',userdata.arm_ps)
        solution_list=list()
        self.haha=0
        solution_list.append(base_ps.point.z-0.92)
        solution_list.append(math.atan2(base_ps.point.y, base_ps.point.x))
        distance = math.sqrt(base_ps.point.y*base_ps.point.y + base_ps.point.x * base_ps.point.x)
        distance -= 0.33
        solution_list.append((-1)*math.acos((distance*distance + 0.28*0.28 - 0.15*0.15)/2/distance/0.28))
        solution_list.append((-1)*solution_list[2]+math.acos((distance*distance + 0.15*0.15 - 0.28*0.28)/2/distance/0.15))
        solution_list.append((-1)*math.acos((distance*distance + 0.15*0.15 - 0.28*0.28)/2/distance/0.15))
        solution_list.append(-1.57)
	#if solution_list[3]>1.57:
	#    solution_list[3]=1.57
	#    solution_list[2]=0.1
        lifting_value = solution_list[0]
        joint_value = solution_list[1:6]
        lifting_controller_level(lifting_value)
	rospy.sleep(10)
        #joint_value_turn = solution_list[1:6]
        #joint_value_turn[1] = -1.56999
        #joint_value_turn[2] = 1.61619
        #joint_value_turn[3] = 1.23
        #joint_value_turn[4] = -1.57
        #arm_controller_level(joint_value_turn)
        
        arm_controller_level(joint_value)
	rospy.sleep(5)
        #gripper_service_level(False)
        #0.000261899782345 -1.56999063492 1.6161891222 1.54923319817 0.0

        joint_value[3]=1.5472
        joint_value[2]=1.6154
	joint_value[1]=-1.57
        #joint_value[3]=0.0
        arm_controller_level(joint_value)
        rospy.sleep(1)
        #if self.haha == 159:
        #    xm_arm_moveit_name('after_high')
        #xm_arm_moveit_name('grasp_pose')
        #xm_arm_moveit_name('nav_poseb')#抓完之后的位置
        return 'succeeded'

class xm_arm_smach():
    def __init__(self):
        rospy.init_node('xm_arm_smach')
        rospy.on_shutdown(self.shutdown)
        xm_arm_moveit_name('nav_pose')
        rospy.logerr('Link Start!')
        self.arm_stack_service  = rospy.Service('arm_stack',xm_PickOrPlace,self.callback)
        self.sm_ArmStack = StateMachine(outcomes=['succeeded','error'],
                                            input_keys=['arm_point','arm_mode'])
        with self.sm_ArmStack:
            StateMachine.add('PICK',
                                DirectIK(),
                                remapping={'arm_ps':'arm_point','arm_mode':'arm_mode'},
                                transitions={'succeeded':'succeeded','error':'error'})
            rospy.spin()
    
    def shutdown(self):
        rospy.logwarn('byebye^_^')

    def callback(self,req):
        self.arm_point = req.goal_position
        self.arm_mode  = req.action
        sm_result = self.sm_ArmStack.execute(parent_ud ={'arm_point':self.arm_point,'arm_mode':self.arm_mode})
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
