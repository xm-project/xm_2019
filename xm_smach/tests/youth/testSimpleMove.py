# /usr/bin/env python

import rospy
from smach import StateMachine
from geometry_msgs.msg import *
from new_lib.basic_move import  LinearDisplacement,TurnDegree

class test():
    def __init__(self):

        rospy.init_node('test')
        rospy.on_shutdown(self.shutdown)

        self.top = StateMachine( outcomes = ['succeeded' , 'aborted' , 'error'])

        with self.top:
            self.top.userdata.go_point = Point(1,0,0)
            self.top.userdata.degree = 3.14
            StateMachine.add('sm1' , LinearDisplacement() ,
                            transitions = {'succeeded' :'sm2' ,
                                            'preempted':'sm2',
                                            'error':'error'},
                            remapping = {'displacementVec' : 'go_point'})
            
            StateMachine.add('sm2' , TurnDegree() , 
                            transitions = {'succeeded':'succeeded',
                                            'aborted':'aborted'} ,
                            remapping = {'degree':'degree'})

        self.top.execute()

    
    def shutdown(self):
        rospy.logerr('done')


if __name__ == '__main__':
    try :
        test()
    
    except Exception,e:
        rospy.logerr(e)