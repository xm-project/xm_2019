#!/usr/bin/env python
# encoding:utf8

from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import *
from geometry_msgs.msg import Pose
import tf
import rospy
from math import pi
import subprocess





class Pick_Clothes():
    def __init__(self):
        rospy.init_node('PickClothes')
        rospy.on_shutdown(self.shutdown)

        self.Find_And_Pick = StateMachine(outcomes =['succeeded','error','aborted'],
                                input_keys = ['clothes_name'])
        with self.Find_And_Pick:
            self.Find_And_Pick.userdata.pose = Pose()
            StateMachine.add('FIND_1',FindClothes(),
                            transitions = {'succeeded':'POSJUST',
                                            'aborted':'aborted',
                                            'error':'error'},
                            remapping = {'clothes_name':'clothes_name',
                                        'clothes_pos':'clothes_pos'})
            StateMachine.add('POSJUST',PosJustfy(),
                            transitions = {'succeeded':'FIND_2',
                                            'aborted':'NAV_CLO',
                                            'error':'error'},
                            remapping = {'object_pos':'clothes_pos',
                                        'pose':'pose'})
            StateMachine.add('NAV_CLO',NavStack(),
                            transitions = {'succeeded':'PICK',
                                            'aborted':'NAV_CLO',
                                            'error':'error'},
                            remapping = {'pos_xm':'pose'})
            StateMachine.add('FIND_2', FindClothes(),
                            transitions = {'succeeded':'PICK',
                                            'aborted':'NAV_CLO',
                                            'error':'error'},
                            remapping = {'clothes_pos':'clothes_pos'})
            self.Find_And_Pick.userdata.objmode = 1
            self.Find_And_Pick.userdata.mode = 1
            StateMachine.add('PICK',ArmCmd(),
                            transitions = {'succeeded':'succeeded',
                                            'aborted':'succeeded',
                                            'error':'error'},
                            remapping = {'arm_ps':'clothes_pos'})
        self.sm_Person = StateMachine(outcomes = ['succeeded','aborted','error'])

        with self.sm_Person:
            self.sm_Person.userdata.rec = 2.0
            # run the people_tracking node
            StateMachine.add('RUNNODE',
                             RunNode(),
                             transitions={'succeeded': 'WAIT', 'aborted': 'RUNNODE'})  # On simulation , We need to skip RunNode ;So aborted -> Wait Or aborted->RunNode
            StateMachine.add('WAIT',
                             Wait(),
                             transitions={
                                 'succeeded': 'GET_PEOPLE_POS', 'error': 'error'},
                             remapping={'rec': 'rec'})
            #the data should be PointStamped()
            # 这里必须先运行节点，也就是用RunNode()状态
            self.sm_Person.userdata.pos_xm = Pose()
            StateMachine.add('GET_PEOPLE_POS',
                             FindPeople().find_people_,
                             transitions={
                                 'invalid': 'NAV_PEOPLE', 'valid': 'GET_PEOPLE_POS', 'preempted': 'aborted'},
                             remapping={'pos_xm': 'pos_xm'}
                             )
            # this state will use the userdata remapping in the last state

            StateMachine.add('NAV_PEOPLE',
                             NavStack(),
                             transitions={
                                 'succeeded': 'succeeded', 'aborted': 'NAV_PEOPLE', 'error': 'error'},
                             remapping={'pos_xm': 'pos_xm'})


        self.PickClothes = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.PickClothes:
            self.PickClothes.userdata.consolation_sentence = 'It is 25 centigrade degree today,I think you need a T-shirt'
            self.PickClothes.userdata.clothes_pos = Pose()
            self.PickClothes.userdata.degree = pi
            self.PickClothes.userdata.clothes_name = 'T-shirt'

            StateMachine.add('SPEAK1' , Speak(),
                            transitions = {'succeeded':'TURN',
                                            'aborted':'TURN',
                                            'error' :'error'},
                            remapping = {'sentences':'consolation_sentence'})
            
            StateMachine.add('TURN' , TurnDegree(),
                            transitions = {'succeeded':'FIND',
                                            'aborted':'FIND',
                                            'error':'error'},
                            remapping = {'degree':'degree'})


            StateMachine.add('FIND' ,self.Find_And_Pick,
                            transitions = {'succeeded':'TURN2',
                                            'aborted':'aborted',
                                            'error':'error'},
                            remapping = {'clothes_name':'clothes_name'})
            
            StateMachine.add('TURN2' , TurnDegree(),
                            transitions = {'succeeded':'NAV_PEO',
                                            'aborted':'NAV_PEO',
                                            'error':'error'},
                            remapping = {'degree':'degree'})
            StateMachine.add('NAV_PEO' , self.sm_Person , 
                            transitions = {'succeeded':'PLACE',
                                            'aborted':'aborted',
                                            'error':'error'})

            StateMachine.add('PLACE' , PlaceCloth() , 
                            transitions = {'succeeded':'succeeded',
                                            'aborted':'aborted',
                                            })
            


        
        intro = IntrospectionServer('PickClothes',self.PickClothes,'/SM_ROOT')
        intro.start()
        self.PickClothes.execute()
        intro.stop()
    def shutdown():
        rospy.logerr('DONE')
        
if __name__ == "__main__":
    try:
        Pick_Clothes()
    
    except Exception,e:
        rospy.logerr(e)

