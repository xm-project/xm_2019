#! /usr/bin/env python
# encoding:utf8

import rospy
from smach import *
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.help_me_carry_lib import *
from xm_smach.common_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import subprocess
import math


class find():
    def __init__(self):
        rospy.init_node('find')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        rospy.logwarn('Fuck')

        self.FindObj = StateMachine(outcomes=['succeeded', 'aborted', 'error'])

        with self.FindObj:
            self.FindObj.userdata.pose = gpsr_target['livingroom_table_1']['pos']
            self.FindObj.userdata.mode_1 = 1
            

            StateMachine.add('NAV_HEHE',
                             NavStack(),
                             remapping={'pos_xm': 'pose'},
                             transitions={'succeeded': 'FIND_OBJECT', 'aborted': 'NAV_HEHE', 'error': 'error'})
            self.FindObj.userdata.name = 'milk'
            self.FindObj.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                             FindObject(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'name': 'name', 'object_pos': 'object_pos'})

        self.Find = StateMachine(outcomes=['succeeded', 'aborted', 'error'])

        with self.Find:
            self.Find.userdata.sentences = 'I will find milk!'
            self.Find.userdata.sentences1 = 'I have found it! Fucking high!'
            self.Find.userdata.door_pos = Pose(Point(0,0,0),Quaternion(0,0,0,1))

            StateMachine.add('SPEAK', Speak(),
                             transitions={'succeeded': 'FINDSTH',
                                          'aborted': 'SPEAK', 'error': 'error'},
                             remapping={'sentences': 'sentences'})
            StateMachine.add('FINDSTH', self.FindObj,
                             transitions={'succeeded': 'SPEAK1', 'aborted': 'FINDSTH', 'error': 'error'})
            StateMachine.add('SPEAK1', Speak(),
                             transitions={'succeeded': 'BACK',
                                          'aborted': 'SPEAK1', 'error': 'error'},
                             remapping={'sentences': 'sentences1'})
            StateMachine.add('BACK',NavStack(),
                            transitions={'succeeded':'succeeded','aborted':'BACK','error':'error'},
                            remapping={'pos_xm':'door_pos'})

        Intro = IntrospectionServer('Find',self.Find,'/SM_ROOT')
        Intro.start()
        self.Find.execute()
        self.smach_bool = True
        Intro.stop()


    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn('Done')
        else:
            rospy.logwarn('Fuck')

if __name__ == "__main__":
    find()
