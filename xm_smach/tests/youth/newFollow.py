#!/usr/bin/env python
# encoding:utf8

import rospy
from smach import *
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.target_gpsr import *
from geometry_msgs.msg import *
from xm_smach.shopping_lib import *
import math
import subprocess
from xm_msgs.msg import *
from xm_msgs.srv import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from xm_smach.help_me_carry_lib import *
from new_lib.special import LegTracker

class Follow():
    def __init__(self):
        rospy.init_node('Follow')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Let\'s go')
        
        self.trace = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'},
                                              'aborted':{'FOLLOW':'preempted'}},
                                 child_termination_cb = self.child_cb,
                                 input_keys = ['people_id'])
        with self.trace:
            self.meta_follow = StateMachine(['succeeded','aborted','preempted'],input_keys = ['people_id'])
            with self.meta_follow:
                self.meta_follow.userdata.pos_xm = Pose()
                self.meta_follow.userdata.rec = 0.2
                StateMachine.add('FIND',
                                    LegTracker().tracker,
                                    transitions = {'invalid':'WAIT','valid':'FIND','preempted':'preempted'},
                            
                                    remapping={'pos_xm':'pos_xm' , 'people_id':'people_id'})
                StateMachine.add('WAIT' , 
                                    Wait(),
                                    transitions={'succeeded':'META_NAV' , 'error':'META_NAV'})
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
            Concurrence.add('STOP',CheckStop())

        self.sm_GPSR = StateMachine(outcomes = ['succeeded' , 'aborted' , 'error'])

        with self.sm_GPSR:
            self.sm_GPSR.userdata.target = list()
            self.sm_GPSR.userdata.action = list()
            self.sm_GPSR.userdata.task_num =0
            self.sm_GPSR.userdata.current_task =-1
            self.sm_GPSR.userdata.people_id =-1
            # StateMachine.add('RUNNODE',RunNode(),
            #                 transitions={'succeeded':'XM','aborted':'aborted'})
            StateMachine.add('XM' , self.trace,
                             transitions = {'succeeded': 'succeeded',
                                            'aborted' : 'aborted',
                                            #'preempted':'error'
                                            })


        intro_server = IntrospectionServer('sm_gpsr' , self.sm_GPSR , '/SM_ROOT')
        intro_server.start()
        out = self.sm_GPSR.execute()
        print out
        intro_server.stop()
        self.smach_bool =True

            
            
    def child_cb(self , outcome_map):

        if outcome_map['STOP'] == 'stop' :
            rospy.logwarn('get the goal , stop tracing.......')

            pid = get_pid('people_tracking')
            subprocess.call('kill ' + str(pid[0]) , shell = True)
            return True
        elif outcome_map['STOP'] == 'aborted':
            rospy.logerr('the stop state meet error !!!!')
            return True


        elif outcome_map['FOllOW']:
            rospy.logerr('FUCK , error state!!!!')
            return True
            
        return False

        

    # def nav_child_cb(self , outcome_map):

    #     if outcome_map['WAIT'] == 'succeeded':
    #         return True
    #     elif outcome_map['NAV'] == 'succeeded':
    #         return True
    #     elif outcome_map['NAV'] ==  'aborted':
    #         return True

    #     else:
    #         return False
	
    def shutdown(self):
        
        rospy.loginfo('smach succeeded')
        
	# def trace_child_cb(self,outcome_map):
    #     if outcome_map['STOP'] == 'stop':
    #         rospy.logwarn('get the stop signal, stop tracing ........')
    #         pid = get_pid("people_tracking")
    #         subprocess.call('kill '+str(pid[0]),shell=True)
    #         # subprocess.call('xterm -e touch /home/ye/Recognition/kinect2/close_image_test &', shell = True)
    #         return True
    #     elif outcome_map['STOP'] == 'remeber':
    #         rospy.logwarn('ready to remeber the position')
    #         return True
    #     elif outcome_map['STOP'] == 'aborted':
    #         rospy.logwarn('check stop error')
    #         return True   
    #     if outcome_map['FOLLOW']:
    #         rospy.logerr('the follow state meet error!')
    #         return True
    #     return False 
    def nav_child_cb(self,outcome_map):
        if outcome_map['WAIT'] == 'succeeded':
            rospy.logwarn('get the pos again')
            return True
        elif outcome_map['NAV'] == 'succeeded':
            return True
        elif outcome_map['NAV'] == 'aborted':
            return True
        elif outcome_map['NAV'] == None:
            return True
        else:
            print outcome_map
            return False



if __name__ == '__main__':
    try:
	    Follow()
    except Exception , e:
        rospy.logerr(e)
