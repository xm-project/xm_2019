#! /usr/bin/env python3
# encoding:utf8


import rospy
from smach import *
from smach_ros import *
from xm_smach.new_lib import *
from xm_smach.help_me_carry_lib import *
from xm_smach.gpsr_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from smach_ros import IntrospectionServer


class go_and_follow():
    def __init__(self):
        rospy.init_node('go_and_follow')
        
        rospy.logerr('GOGOGO')


        self.didi = StateMachine(['succeeded','aborted','preempted'])

        with self.didi:
            self.didi.userdata.sentences1 = 'Chen Shaw Rui is a little brother'
            
            StateMachine.add('SPEAK',SpeakSentence(),
                            transitions={'succeeded':'SPEAK','aborted':'SPEAK','error':'aborted'},
                            remapping={'sentences':'sentences1'})


        self.didi.execute()




if __name__ == "__main__":
    go_and_follow()
    




