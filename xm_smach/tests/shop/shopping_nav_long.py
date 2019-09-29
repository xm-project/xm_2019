#!/usr/bin/env python
# encoding:utf-8
#请求跟随->跟随的同时语音识别->识别到后记录位置和物品->继续跟随->语音识别到结束后停止->语音识别物品名->
import rospy
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.shopping_lib import * 
from xm_smach.target_gpsr import *
from smach import State, StateMachine, Concurrence
from math import pi
from geometry_msgs.msg import *
import subprocess

class Shopping():
    def __init__(self):
        rospy.init_node('Shopping')
        self.smach_bool =False
        rospy.on_shutdown(self.shutdown)
        self.get_command = StateMachine(outcomes =['succeeded','aborted'])
        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            # rospy.logerr('sm_EnterRoom add State')
            # arm go to the nav_pose in the srdf
            # self.sm_EnterRoom.userdata.arm_mode =0
            # self.sm_EnterRoom.userdata.arm_ps_1 = PointStamped()
            # StateMachine.add('NAV_POSE',
            #                     ArmCmd(),
            #                     transitions={'succeeded':'DOOR_DETECT','aborted':'NAV_POSE','error':'error'},
            #                     remapping ={'mode':'arm_mode','arm_ps':'arm_ps_1'})
            # wait the door to open
            # StateMachine.add('DOOR_DETECT',
            #                     DoorDetect().door_detect_,
            #                     transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'error'})
            # simple delay 5s
            self.sm_EnterRoom.userdata.rec = 5.0
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
                                remapping ={'rec':'rec'})

            self.sm_EnterRoom.userdata.point = Point(1.5,0.0,0.0)
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove_move(),
                                transitions={'succeeded':'NAV_1','aborted':'NAV_1','error':'error'},
                                remapping={'point':'point'})
            
            # navstack to room of crowds
            # waypoints are the list of Pose fit the data type need
            self.sm_EnterRoom.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            # self.sm_EnterRoom.userdata.nav_mode =0
            StateMachine.add('NAV_1',
                                NavStack(),
                                transitions={'succeeded':'succeeded','aborted':'NAV_1','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
            
            # self-introduce
            # self.sm_EnterRoom.userdata.sentences_2 = 'I am robot xiaomeng, I come from the nwpu'
            # StateMachine.add('SELF_INTRO',
            #                     Speak(),
            #                     remapping ={'sentences':'sentences_2'},
            #                     transitions ={'succeeded':'succeeded','aborted':'SELF_INTRO','error':'error'})
        with self.get_command:
            StateMachine.add('GET_FOLLOW',
                                GetSignal(),
                                transitions={'succeeded':'succeeded','aborted':'aborted'})
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

        # self.get_mission = StateMachine(outcomes=['succeeded','aborted'],
        #                                 output_keys=['things_list'])
        # with self.get_mission:
        #     self.get_mission.userdata.things_list = list()
        #     StateMachine.add('LISTEN',
        #                         GetMission(),
        #                         remapping = ['things_list'],
        #                         transitions={'succeeded':'succeeded','aborted':'aborted'})

        self.nav_mission = StateMachine(outcomes=['succeeded','aborted','error'],
                                        input_keys=['mission'])
        with self.nav_mission:
            self.nav_mission.userdata.pos_xm = Pose()
            StateMachine.add('GET_TARGET',
                                GetTarget(),
                                remapping = {'pos_xm':'pos_xm','mission':'mission'},
                                transitions = {'succeeded':'NAV','aborted':'aborted','finish':'NAV_FINAL'})
            StateMachine.add('NAV',
                                NavStack(),
                                remapping = {'pos_xm':'pos_xm'},
                                transitions = {'succeeded':'GET_TARGET','aborted':'aborted','error':'error'})  
            StateMachine.add('NAV_FINAL',
                                NavStack(),
                                remapping = {'pos_xm':'pos_xm'},
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})           


        self.shopping = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.shopping:
            self.shopping.userdata.PT_LIST = {}
            self.shopping.userdata.mission = {}
            self.shopping.userdata.rec = 5.0
            # StateMachine.add('ENTERROOM',
            #                     self.sm_EnterRoom,
            #                     transitions={'succeeded':'START','aborted':'aborted'})
            StateMachine.add('START',
                                self.get_command,
                                transitions={'succeeded':'RUNNODE','aborted':'aborted'})
            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'TRACE','error':'error'},
                                remapping={'rec':'rec'})
            StateMachine.add('TRACE',
                                self.trace,
                                transitions={'remeber':'TRACE','stop':'NAV_MISSION','aborted':'aborted'},
                                remapping={'PT_LIST':'PT_LIST','mission':'mission'})
            StateMachine.add('NAV_MISSION',
                                self.nav_mission,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping={'mission':'mission'})
        intro_server = IntrospectionServer('shoping',self.shopping, 'SM_ROOT')
        intro_server.start()
        self.shopping.execute()
        intro_server.stop()
        self.smach_bool =True

    def shutdown(self):
        if self.smach_bool ==False:
            rospy.logwarn('smach execute failed')
        else:
            rospy.logwarn('smach execute successfully')

    def trace_child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('get the stop signal, stop tracing ........')
            subprocess.call('xterm -e touch /home/ye/Recognition/kinect2/close_image_test &', shell = True)
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
            print outcome_map
            return False

if __name__ =="__main__":
    try:
        Shopping()
    except Exception,e:
        rospy.logerr(e)
