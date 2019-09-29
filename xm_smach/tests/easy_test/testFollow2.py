#! /usr/bin/env python


import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from new_lib.basic_move import *
from new_lib.basic_vision import * 
from new_lib.basic_voice import *
from geometry_msgs.msg import *
from xm_smach.gpsr_lib import RunNode
import math
import subprocess


class  TestFollow():
    def __init__(self):
        rospy.init_node('TestFollow')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Follow start')

        self.smach_bool = False

        self.newFollow = StateMachine(outcomes = ['succeeded' , 'aborted','error'])

        with self.newFollow:

            StateMachine.add('RUNNODE' , RunNode() , transitions = {'succeeded':'FOLLOW' , 
                                                                    'aborted':'RUNNODE',
                                                                    })

            StateMachine.add('FOLLOW' ,FollowTest() , transitions = {'succeeded':'succeeded' ,
                                                                    'error':'error',
                                                                    'aborted':'aborted'} )

                        
        intro_server = IntrospectionServer('sm_gpsr',self.newFollow,'/SM_ROOT')
        intro_server.start()
        out = self.newFollow.execute()
        rospy.logerr(out)
        intro_server.stop()
        self.smach_bool = True 

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')


if __name__ == "__main__":
    try:
        TestFollow()
    except Exception,e:
        rospy.logerr(e)   
    