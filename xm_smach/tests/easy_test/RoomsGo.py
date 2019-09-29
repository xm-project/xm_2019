#!/usr/bin/env python
# encoding:utf8

import rospy
import smach
import smach_ros
from smach import Iterator,State,StateMachine
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import *
from xm_smach.target_gpsr import gpsr_target

class IndexTrans(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded'],
                       input_keys=['index' , 'poses'],
                       output_keys=['index' , 'current_pos'])
    def execute(self,userdata):
        userdata.current_pos = userdata.poses[userdata.index]
        rospy.logwarn(userdata.index)
        return 'succeeded'
                       


class RoomsGo():
    def __init__(self):
        rospy.init_node('RoomsGo')
        self.Rooms = ['speaker','kitchen' ,'speaker']
        self.RoomsPoses =[]

        for Room in self.Rooms:
            self.RoomsPoses.append(gpsr_target[Room]['pos'])

    def execute(self):
        
        sm_top = smach.StateMachine(outcomes = ['succeeded','preemted','aborted','continue'])
         
        with sm_top:
            self.rooms_go = Iterator(outcomes = ['succeeded','preemted','aborted'],
                                     input_keys=[],
                                     output_keys = ['index'],
                                     it = lambda :range(0,3),
                                     it_label = 'index',
                                     exhausted_outcome = 'succeeded')
            with self.rooms_go:
                self.rooms_go_proton = StateMachine(outcomes=['succeeded','aborted','continue'],
                                                    input_keys=['index'])
                with self.rooms_go_proton:
                    #self.current_pos = self.RoomsPoses[index]
                    
                    self.rooms_go_proton.userdata.current_poses = self.RoomsPoses
                    self.rooms_go_proton.userdata.current_pos2 = 0
                    self.rooms_go_proton.add('INDEXTRANS',
                                    IndexTrans(),
                                    transitions={'succeeded':'GO'},
                                    remapping={'poses':'current_poses',
                                                'current_pos':'current_pos2',
                                                'index':'index'})
                    
                    self.rooms_go_proton.add('GO',
                                    NavStack(),
                                    transitions={'succeeded':'continue',
                                                'aborted':'GO',
                                                'error':'continue'},
                                    remapping={'pos_xm' :'current_pos2'})

                Iterator.set_contained_state('CONTAINER_STATE',self.rooms_go_proton,loop_outcomes=['continue'])
            

            sm_top.add('ROOMS_GO',self.rooms_go,transitions={'succeeded':'succeeded',
                                                               'aborted':'aborted'},
                                                remapping={'index':'index'})
        out = sm_top.execute()
        rospy.logerr(out)

def main():
    RobotGo = RoomsGo()
    RobotGo.execute()


if __name__ =='__main__':
    main()

####################################################################################################################
# import roslib; roslib.load_manifest('smach')
# roslib.load_manifest('smach_ros')
# import rospy

# import smach
# from smach import Iterator, StateMachine, CBState
# from smach_ros import ConditionState, IntrospectionServer

# def construct_sm():
#     sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
#     sm.userdata.nums = range(25, 88, 3)
#     sm.userdata.even_nums = []
#     sm.userdata.odd_nums = []
#     with sm:
#         tutorial_it = Iterator(outcomes = ['succeeded','preempted','aborted'],
#                                input_keys = ['nums', 'even_nums', 'odd_nums'],
#                                it = lambda: range(0, len(sm.userdata.nums)),
#                                output_keys = ['even_nums', 'odd_nums'],
#                                it_label = 'index',
#                                exhausted_outcome = 'succeeded')
#         with tutorial_it:
#             container_sm = StateMachine(outcomes = ['succeeded','preempted','aborted','continue'],
#                                         input_keys = ['nums', 'index', 'even_nums', 'odd_nums'],
#                                         output_keys = ['even_nums', 'odd_nums'])
#             with container_sm:
#                 #test wether even or odd
#                 StateMachine.add('EVEN_OR_ODD',
#                                  ConditionState(cond_cb = lambda ud:ud.nums[ud.index]%2, 
#                                                 input_keys=['nums', 'index']),
#                                  {'true':'ODD',
#                                   'false':'EVEN' })
#                 #add even state
#                 # @smach.cb_interface(input_keys=['nums', 'index', 'even_nums'],
#                 #                     output_keys=['odd_nums'], 
#                 #                     outcomes=['succeeded'])
#                 def even_cb(ud):
#                     ud.even_nums.append(ud.nums[ud.index])
#                     return 'succeeded'
#                 StateMachine.add('EVEN', CBState(even_cb), 
#                                  {'succeeded':'continue'})
#                 #add odd state
#                 # @smach.cb_interface(input_keys=['nums', 'index', 'odd_nums'], 
#                 #                     output_keys=['odd_nums'], 
#                 #                     outcomes=['succeeded'])
#                 def odd_cb(ud):
#                     ud.odd_nums.append(ud.nums[ud.index])
#                     return 'succeeded'
#                 StateMachine.add('ODD', CBState(odd_cb), 
#                                  {'succeeded':'continue'})
#             #close container_sm
#             Iterator.set_contained_state('CONTAINER_STATE', 
#                                          container_sm, 
#                                          loop_outcomes=['continue'])
#         #close the tutorial_it
#         StateMachine.add('TUTORIAL_IT',tutorial_it,
#                      {'succeeded':'succeeded',
#                       'aborted':'aborted'})
#     return sm

# def main():
#     rospy.init_node("iterator_tutorial")
#     sm_iterator = construct_sm()                               

        
        
    
        
        
