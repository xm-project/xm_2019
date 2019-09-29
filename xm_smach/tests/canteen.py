#!/usr/bin/env python
# encoding:utf8

from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from xm_smach.common_lib import *
from geometry_msgs.msg import Pose
import tf
import rospy
from math import pi
import subprocess

class Find_People(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       output_keys=['people_pos'])
        self.find_client = rospy.ServiceProxy('get_position', xm_ObjectDetect)
        self.listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            subprocess.Popen(
                'xterm -e rosrun xm_vision dining_people  &', shell=True)

            for i in range(5):
                self.find_client.wait_for_service(timeout=30.0)
                find_request = xm_ObjectDetectRequest()
                find_request.object_name = 'people'
                rospy.logerr('call----')
                res = self.find_client.call(find_request)
                rospy.logerr(res)
                if len(res.object) != 0:
                    break

            if i == 2:
                rospy.logerr('camera goes bad')

            rospy.logwarn(res.object[0])
            if res.object[0].pos.point.x == -10.000 and res.object[0].pos.point.y == -10.000 and res.object[0].pos.point.z == -10.000:
                rospy.logerr('find nothing')
                return 'aborted'

            object_pos = PointStamped()
            object_pos.point.x = res.object[0].pos.point.z - 0.11
            object_pos.point.y = res.object[0].pos.point.x - 0.125
            object_pos.point.z = 0.917-res.object[0].pos.point.y
            object_pos.header.frame_id = 'base_link'

            # 将物品的位置信息传输到userdata
            userdata.people_pos = object_pos

            # userdata.object_pos = res.object[0].pos
            # output_keys cannot be read
            # print userdata.object_pos
            return 'succeeded'

        except Exception, e:
            rospy.logerr(e)
            return 'error'


class Canteen():
    def __init__(self):
        rospy.init_node('canteen')
        rospy.on_shutdown(self.shutdown)
        self.Canteen = StateMachine(outcomes = ['succeeded','error','aborted'])

        with self.Canteen:
            self.Canteen.userdata.people_pos = PointStamped()
            self.Canteen.userdata.pos_xm = Pose()
            self.Canteen.userdata.rec = 3.0
            self.Canteen.userdata.sentence = 'What would you like , my sweetie?'
            StateMachine.add('FIND_PEO',Find_People(),
                            transitions = {'succeeded':'POS_JUS',
                                            'aborted':'WAIT',
                                            'error':'error'},
                            remapping = {'people_pos':'people_pos'})
            StateMachine.add('WAIT' , Wait() , 
                                transitions = {'error':'error',
                                                'succeeded':'FIND_PEO'},
                                remapping = {'rec':'rec'})

            StateMachine.add('POS_JUS' , PosJustfy(), 
                            transitions = {'succeeded':'NAV_PEO',
                                            'aborted':'aborted',
                                            'error':'error'},
                            remapping = {'object_pos':'people_pos',
                                        'pose':'pos_xm'})

            StateMachine.add('NAV_PEO' , NavStack(),
                            transitions = {'succeeded':'Speak',
                                            'aborted':'NAV_PEO',
                                            'error':'error'},
                            remapping = {'pos_xm':'pos_xm'})


            StateMachine.add('Speak' , Speak(),
                            transitions = {'succeeded':'succeeded',
                                            'aborted':'aborted',
                                            'error':'error'},
                            remapping = {'sentences':'sentence'})
            
        self.Canteen.execute()

    def shutdown(self):
        rospy.logwarn('done')    


if __name__ == '__main__':
    try:
        Canteen()
    
    except Exception ,e:
        rospy.logerr(e)

