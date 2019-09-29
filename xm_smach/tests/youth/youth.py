#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
import new_lib.basic_move as new_move
from xm_smach.gpsr_lib import * 
from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
from xm_smach.pick_turn import *
from xm_smach.checkturn import *
import math
import subprocess  
from tf.transformations import euler_from_quaternion,quaternion_from_euler
class simple_turn(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'aborted', 'error'])
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.twist_xm = Twist()

    def execute(self , userdata):
        if self.preempt_requested():
            return 'error'

        self.twist_xm.angular.z = 1.0
        self.twist_xm.linear.x = 2.0



        try:
            self.cmd_vel.publish(self.twist_xm)
            rospy.logwarn(self.twist_xm)
            return 'succeeded'
        except Exception,e:
            rospy.logerr(e)
            rospy.logerr("Simple_Turn Error")
            return 'aborted'


class Youth():
    def __init__(self):
        rospy.init_node('Youth')
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('------------start test----------------')
        self.test_bool = False

        self.run = StateMachine(outcomes=['succeeded','aborted','error'])
        self.run.userdata.point1 = Pose(Point( 5.295, -1.189, 0.000),Quaternion(0.000, 0.000,0.038, 0.999))
        #self.run.userdata.point2 = gpsr_target['point_2']['pos']
        #self.run.userdata.point3 = gpsr_target['speaker']['pos']
        #self.run.userdata.point4 = gpsr_target['bedroom_table_2']['pos']
        self.run.userdata.point0 = Pose(Point(0,0,0),Quaternion(0,0,0,1))
        self.run.userdata.pos_xm = Pose()

        self.CheckTurn = Concurrence(outcomes=['turn', 'aborted', 'error'],
                                     default_outcome='aborted',
                                     input_keys=['nav_pos'],
                                     outcome_map={'turn': {'IsTurn': 'yes'},
                                                  'aborted': {'IsTurn': 'no'},
                                                  'error': {'IsTurn': 'error'}},
                                     child_termination_cb=self.Child_Turn_cb)

        with self.CheckTurn:
            #self.CheckTurn.userdata.nav_pos = gpsr_target[self.CheckTurn.userdata.target_name]['pos']
            Concurrence.add('IsTurn', IsTurn())
            Concurrence.add('TestNav', NavStack(),
                            remapping={'pos_xm': 'nav_pos'})

        self.TurnNav = Concurrence(outcomes = ['succeeded','aborted','error','turn'],
                                    default_outcome = 'succeeded',
                                    input_keys=['nav_pos'],
                                    outcome_cb=self.ocb,
                                    child_termination_cb=self.child_cb)
        
        with self.TurnNav:
            # self.turn = StateMachine(outcomes = ['succeeded','aborted'])
            # with self.turn:
            #     self.turn.userdata.rec = 1.0
            #     StateMachine.add('TURNOR',check_turn(),
            #                     transitions={'yes':'GOTURN',
            #                                 'no':'WAIT',
            #                                 'stop':'aborted'})
            #     StateMachine.add('WAIT',Wait(),
            #                     transitions={'succeeded':'TURNOR',
            #                                 'error':'TURNOR'},
            #                     remapping={'rec':'rec'})
            #     #StateMachine.add('GOTURN',go_turn(),transitions={'succeeded':'succeeded',
            #     #                                               'aborted':'TURNOR'})
            
            Concurrence.add('Turn',check_turn())
            Concurrence.add('Nav',
                            NavStack(),
                            remapping={'pos_xm':'nav_pos'})
        
        self.run.userdata.rec = 1.0
        with self.run:
            self.run.userdata.turn_pose = Pose()
            self.run.userdata.nav_pos = Pose()
            StateMachine.add('TURNNAV0',new_move.NavStack(),
                            transitions={'succeeded':'ADJ',
                                        'aborted':'TURNNAV0',
                                        'error':'error',
                                        'preempted':'error'},
                            remapping={'pos_xm':'point1'}) 
            
            StateMachine.add("ADJ" , CBState(self.cbCall , outcomes=['succeeded' ,'error'],input_keys =['pos_xm'] ,output_keys=['nav_pos']),
                                    transitions={'succeeded':'NAV2' , 'error':'error' },
                                    remapping={'pos_xm':'point1','nav_pos':'nav_pos'})

            StateMachine.add('NAV2' , NavStack() , transitions={'succeeded':'succeeded','aborted':'NAV2','error':'error'},
                               remapping={'pos_xm':'nav_pos'} )
            # StateMachine.add('TURNNAV1',self.CheckTurn,
            #                 transitions={'error':'TURNNAV2',
            #                             'turn':'GOTURN',
            #                             'aborted':'TURNNAV2',
            #                             },
            #                 remapping={'nav_pos':'point3'})
            # StateMachine.add('GOTURN',PickTurn(),
            #                 transitions={'succeeded':'TURNNAV3',
            #                             'aborted':'TURNNAV2'},
            #                 remapping={'turn_pose':'turn_pose'})

            # StateMachine.add('TURNNAV3',NavStack(),
            #                 transitions={'succeeded':'TURNNAV2',
            #                             'aborted':'TURNNAV2',
            #                             'error':'error'},
            #                 remapping={'pos_xm':'turn_pos'}) 

            # StateMachine.add('TURNNAV2',NavStack(),
            #                 transitions={'succeeded':'succeeded',
            #                             'aborted':'TURNNAV2',
            #                             'error':'error'},
            #                 remapping={'pos_xm':'point3'}) 
            #self.run.userdata.point5 = gpsr_target['diningroom']['pos']
            # StateMachine.add('CHECKTURN',check_turn(),
            #                 transitions={'yes':'NAV1' , 'no':'NAV1'},
            #                 remapping={'nav_pos':'point1'})
            # StateMachine.add('NAV1',
            #                     NavStack(),
            #                     transitions={'succeeded':'NAV5','aborted':'NAV1','error':'error'},
            #                     remapping = {'pos_xm':'point1'})
            # StateMachine.add('WAIT',

            #                 Wait(),
            #                 transitions = {'succeeded':'TURN','error':'error'},
            #                 remapping ={'rec':'rec'})
		

            # StateMachine.add('NAV2',
            #                     NavStack(),
            #                     transitions={'succeeded':'NAV3','aborted':'NAV2','error':'error'},
            #                     remapping={'pos_xm':'point2'})
            # StateMachine.add('NAV3',
            #                     NavStack(),
            #                     transitions={'succeeded':'NAV4','aborted':'NAV3','error':'error'},
            #                     remapping={'pos_xm':'point3'})
            # StateMachine.add('NAV4',
            #                     NavStack(),
            #                     transitions={'succeeded':'NAV5','aborted':'NAV4','error':'error'},
            #                     remapping={'pos_xm':'point4'})
            # StateMachine.add('TURN',PickTurn(),
            #                 transitions={'succeeded':'NAV5','aborted':'TURN'},
            #                 remapping={'xm_pos':'point1','turn_pose':'pos_xm'})
            # StateMachine.add('CHECKTURN2',self.CheckTurn,
            #                 transitions={'turn':'NAV5','aborted':'NAV5','error':'error'},
            #                 remapping={'nav_pos':'point3'})
            # StateMachine.add('NAV5',
            #                 NavStack(),
            #                 transitions={'succeeded':'succeeded','aborted':'NAV5','error':'error'},
            #                 remapping={'pos_xm':'point3'})
        self.youth = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.youth:
            self.youth.userdata.rec = 10.0
            self.youth.userdata.go_point=Point(1.5,0.0,0.0)
            self.youth.userdata.arrive_sentences = 'fight for the bright tomorrow'
            StateMachine.add('RUN',
                                self.run,
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})
            # StateMachine.add('DOORDETECT',
            #                     DoorDetect().door_detect_,
            #                     transitions={'invalid':'WAIT','valid':'DOORDETECT','preempted':'aborted'})
            # StateMachine.add('WAIT',
            #                     Wait(),
            #                     transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
            #                     remapping ={'rec':'rec'})
            # StateMachine.add('SIMPLE_MOVE',
            #                     SimpleMove_move(),
            #                     remapping ={'point':'go_point'},
            #                     transitions={'succeeded':'SPEAK_ARRIVE','aborted':'SPEAK_ARRIVE','error':'error'})
            # StateMachine.add('SPEAK_ARRIVE',
            #                     Speak(),
            #                     remapping = {'sentences':'arrive_sentences'},
            #                     transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
  
        out = self.youth.execute()
        print out
        if out =='succeeded':
            self.test_bool = True
    def shutdown(self):
        if self.test_bool == True:
            rospy.loginfo('test succeeded')
        else:
            rospy.logerr('test failed')
    
    def Child_Turn_cb(self , outcome_map):
        if outcome_map['IsTurn']:
            return True
        else:
            return False

    def child_cb(self , outcome_map):
        if outcome_map['Turn']:
            return True
        elif outcome_map['Nav']:
            return True
        else:
            return False

    def ocb(self,outcome_map):
        if(outcome_map['Turn']=='yes'):
            return 'turn'
        elif outcome_map['Nav']=='succeeded':
            return 'succeeded'
        elif outcome_map['Nav']=='aborted':
            return 'aborted'
        else:
            return 'error'
    def cbCall(self, userdata):
        try:
            rospy.logerr(userdata.pos_xm)

            qu = userdata.pos_xm.orientation
            
            eu = euler_from_quaternion([qu.x,qu.y, qu.z , qu.w])
            eu2= eu[2]+pi/2
            rospy.logerr(qu)
            qu = quaternion_from_euler(eu[0],eu[1],eu2)
            
            nav_pos = Pose(userdata.pos_xm.position , Quaternion(qu[0] ,qu[1],qu[2],qu[3]))
            rospy.logerr(nav_pos)
            userdata.nav_pos = nav_pos
            return 'succeeded'
        except Exception,e:
            rospy.logerr(e)
            return 'error'



if __name__ == '__main__':
    try:
        Youth()
    except Exception,e:
        rospy.logerr(e)
