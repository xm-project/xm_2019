#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_msgs.msg import String
from xm_speech.msg import *

#------------------------------------------------------------------------------

def xmlocation():
    rospy.init_node('xmlocation')
    rospy.ServiceProxy('hark_source',xm_Speech_tts)

    #rospy.spin()

#------------------------------------------------------------------------------

if __name__ == "__main__":
    xmlocation()    

