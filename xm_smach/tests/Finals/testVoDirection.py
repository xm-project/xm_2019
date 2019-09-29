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




def callback(data):
    #rospy.logwarn(data.HarkSourceVal.length())
    rospy.logerr(data)


def test():
        rospy.init_node('good')
        rospy.Subscriber("/hark_source" , HarkSource , callback)
        rospy.spin()

if __name__ == '__main__':
    test()


    
