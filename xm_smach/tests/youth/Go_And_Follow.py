#! /usr/bin/env python
# encoding:utf8

import rospy
from smach import *
from smach_ros import *
from xm_smach.common_lib import *
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
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('GOGOGO')

        self.meta_follow = StateMachine(['succeeded','aborted','preempted'])
        with self.meta_follow:
            StateMachine.add('FIND',
                                FindPeople().find_people_,
                                transitions = {'invalid':'META_NAV','valid':'FIND','preempted':'preempted'},
                        
                                remapping={'pos_xm':'pos_xm'})
            self.meta_nav = Concurrence(outcomes=['time_over','get_pos','aborted'],
                                            default_outcome  = 'aborted',
                                            outcome_map={'time_over':{'WAIT':'succeeded'},
                                                            'get_pos':{'NAV':'succeeded'},
                                                            'aborted':{'NAV':'aborted'}},
                                            child_termination_cb=self.nav_child_cb,
                                            input_keys=['pos_xm'])
            with self.meta_nav:
                Concurrence.add('NAV',NavStack(),remapping={'pos_xm':'pos_xm'})
                Concurrence.add('WAIT',Wait_trace())
            StateMachine.add('META_NAV',
                                self.meta_nav,
                                transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})

        
        self.sm_top = StateMachine(outcomes=['succeeded','aborted','error'])

        with self.sm_top:
            self.sm_top.userdata.sentences1 = 'You are didi,I am Big man;Go to bedroom'
            self.sm_top.userdata.pos_xm = gpsr_target['bedroom']['pos']
            StateMachine.add('SPEAK',Speak(),
                            transitions={'succeeded':'NAV1','aborted':'NAV1','error':'aborted'},
                            remapping={'sentences':'sentences1'})
            
            StateMachine.add('NAV1' , NavStack(),
                            transitions = {'succeeded':'SPEAK1','aborted':'NAV1','error':'error'})
            
            self.sm_top.userdata.sentences2 = "Van GoghMing is daddy! I will follow a person"

            StateMachine.add('SPEAK1', Speak(),
                            transitions={'succeeded':'RUNNODE','aborted':'RUNNODE','error':'error'},
                            remapping={'sentences':'sentences2'})

            StateMachine.add('RUNNODE',RunNode(),
                            transitions={'succeeded':'FOLLOW','aborted':'aborted'})
		

            StateMachine.add('FOLLOW',self.meta_follow,
                            transitions = {'succeeded':'GOODBYE','aborted':'GOODBYE','preempted':'GOODBYE'})

            self.sm_top.userdata.sentences3 = "Fucking goodbye"

            StateMachine.add('GOODBYE', Speak(),
                            transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                            remapping={'sentences':'sentences3'})


        out = self.sm_top.execute()
        rospy.logerr(out)


    def shutdown(self):
        rospy.logerr('byebye')


    def child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('get the stop signal, stop tracing ........')
            # subprocess.call('touch /home/ye/Recognition/kinect2/dummy_excution_final &', shell = True)
            pid = get_pid("people_tracking")
            subprocess.call('kill '+str(pid[0]),shell=True)
            return True
        elif outcome_map['STOP'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True
            
        if outcome_map['FOLLOW']:
            rospy.logerr('the follow state meet error!')
            return True
        return False

    def nav_child_cb(self,outcome_map):
        if outcome_map['WAIT'] == 'succeeded':
            rospy.logwarn('get the pos again')
            return True
        elif outcome_map['NAV'] == 'succeeded':
            return True
        elif outcome_map['NAV'] == 'aborted':
            return True
        else:
            print outcome_map
            return False

if __name__ == '__main__':
    try:
        go_and_follow()

    except Exception,e:
        rospy.logerr(e)


            


        


        
