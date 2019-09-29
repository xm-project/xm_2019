#! /usr/bin/env python

import rospy
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.shopping_lib import * 
from xm_smach.target_gpsr import *
from smach import State, StateMachine, Concurrence
from math import pi
from geometry_msgs.msg import *
import subprocess







class Follow():
    def __init__(self):
        rospy.init_node('follow')
        rospy.logerr('gogogo')

        self.trace = Concurrence(outcomes = ['remeber','stop','aborted'],
                                        default_outcome = 'stop',
                                        outcome_map={'remeber':{'STOP':'remeber'},
                                                    'stop':{'STOP':'stop'},
                                                    'aborted':{'FOLLOW':'aborted'}},
                                        child_termination_cb = self.trace_child_cb,
                                        input_keys = ['PT_LIST','mission'],
                                        output_keys= ['PT_LIST','mission'])
        with self.trace:
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
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop(),remapping = {'PT_LIST':'PT_LIST','mission':'mission'})


        self.shopping = StateMachine(outcomes=['succeeded','aborted' ,'error'])
        with self.shopping:
            
            self.shopping.userdata.PT_LIST = {}
            self.shopping.userdata.mission = {}
            self.shopping.userdata.task = list()
            self.shopping.userdata.rec = 5.0

            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'TRACE','error':'error'},
                                remapping={'rec':'rec'})

            StateMachine.add("TRACE" , self.trace , transitions={'remeber':'succeeded' ,
                                                                'stop':'succeeded',
                                                                'aborted':'aborted'})

            
        intro_server = IntrospectionServer('shopping',self.shopping, 'SM_ROOT')
        intro_server.start()
        out = self.shopping.execute()
        intro_server.stop()

    def trace_child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('get the stop signal, stop tracing ........')
            pid = get_pid("people_tracking")
            subprocess.call('kill '+str(pid[0]),shell=True)
            return True
        elif outcome_map['STOP'] == 'remeber':
            rospy.logwarn('ready to remeber the position')
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
            print(outcome_map)
            return False    


if __name__ == "__main__":

    Follow()
