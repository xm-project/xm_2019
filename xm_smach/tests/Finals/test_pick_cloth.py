#!/usr/bin/env python
# encoding:utf8

import rospy
from smach import *
from xm_msgs.msg import *
from xm_msgs.srv import *
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import *
from xm_smach.help_me_carry_lib import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import subprocess
from smach_ros import *


class Pick_up():
    def __init__(self):
        rospy.init_node('Pick_up')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        

        self.sm_Pick = StateMachine(outcomes =['succeeded','aborted','error'])
        
        with self.sm_Pick:
            
            self.sm_Pick.userdata.name ='T-shirt'  #NotSure
            self.sm_Pick.userdata.target_mode =0
            self.sm_Pick.userdata.objmode = -1
            self.sm_Pick.userdata.place_ps = PointStamped()
            self.sm_Pick.userdata.place_ps.header.frame_id ='base_link'
            self.sm_Pick.userdata.place_ps.point.x =0.8  #Place On The Table 
            self.sm_Pick.userdata.place_ps.point.y =0.0
            self.sm_Pick.userdata.place_ps.point.z =0.6 
            self.sm_Pick.userdata.Object_Num = 1
            self.sm_Pick.userdata.objmode = 2
            # without moveit, if is just place it in a open space
           

            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_CLOTH','aborted':'aborted'})


            # StateMachine.add('GETNAME',
            #                    remapping ={'target':'target','current_task':'current_task','mode':'target_mode','current_target':'name'},
            #                   transitions={'succeeded':'FIND_OBJECT','aborted':'aborted','error':'error'})
            
            self.sm_Pick.userdata.object_pos = PointStamped()
            # StateMachine.add('FIND_OBJECT',
            #                     FindObject(),
            #                     transitions={'succeeded':'PICK', 'aborted':'FIND_OBJECT', 'error':'error'},
            #                     remapping = {'name':'name', 'object_pos':'object_pos', 'objmode':'objmode'})
            StateMachine.add('FIND_CLOTH',
                                FindClothes(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'FIND_CLOTH','error':'error'},
                                remapping ={'clothes_name':'name','clothes_pos':'object_pos'})
            
            #making the xm foreward the object may make the grasping task easier  
            self.sm_Pick.userdata.pose = Pose()
            StateMachine.add('POS_JUSTFY',
                        
                                PosJustfy(),
                                remapping={'object_pos':'object_pos','pose':'pose'},
                                transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
            StateMachine.add('NAV_TO',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE_IMG2','aborted':'NAV_TO','error':'error'},
                                remapping ={"pos_xm":'pose'})
            StateMachine.add('RUNNODE_IMG2',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_TWICE','aborted':'aborted'})                    
            StateMachine.add('FIND_TWICE',
                                FindClothes(),
                                transitions ={'succeeded':'PICK','aborted':'NAV_TO','error':'error'},
                                remapping ={'clothes_name':'name','clothes_pos':'object_pos'})
            self.sm_Pick.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'PLACE','aborted':'succeeded','error':'error'},
                             remapping={'arm_ps': 'object_pos', 'mode': 'arm_mode_1'})
            StateMachine.add('PLACE',
                                PlaceCloth(),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
            # self.sm_Pick.userdata.arm_mode_2 =2
            # StateMachine.add('PLACE',
            #                     ArmCmd(),
            #                     transitions= {'succeeded':'succeeded', 'aborted':'aborted', 'error':'error'},
            #                     remapping= {'arm_ps':'place_ps' ,'mode':'arm_mode_2'})
            # self.sm_Place = StateMachine(
            # outcomes=['succeeded', 'aborted', 'error'])
            # with self.sm_Place:
            # # # place_ps please specified due to the scene
            # # self.sm_Place.userdata.place_ps = PointStamped()
            # # self.sm_Place.userdata.place_ps.header.frame_id ='base_link'
            # # self.sm_Place.userdata.place_ps.point.x =0.8
            # # self.sm_Place.userdata.place_ps.point.y =0.0
            # # self.sm_Place.userdata.place_ps.point.z =0.6
            # # self.sm_Place.userdata.objmode = 2
            # # # without moveit, if is just place it in a open space
            # # self.sm_Place.userdata.arm_mode_1 =2
               

            self.sm_Pick.userdata.sentences = 'xiao meng can not find things'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})


        intro_server = IntrospectionServer('sm_Pick' , self.sm_Pick , '/SM_ROOT')
        intro_server.start()
        out = self.sm_Pick.execute()
        print out
        intro_server.stop()
        self.smach_bool =True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn("DONE")
        else:
            rospy.logwarn('FUCK THE ERROE')

if __name__ == '__main__':
    try:
	    Pick_up()
    except Exception , e:
        rospy.logerr(e)
