#!/usr/bin/env python

import sys
import rospy
from xm_msgs.srv import *
from xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level
from xm_arm_moveit_level import xm_arm_moveit_level, xm_arm_moveit_name


def laiya():
        rospy.wait_for_service('arm_stack')
        grasp = rospy.ServiceProxy('arm_stack', xm_PickOrPlace)
        req = xm_PickOrPlaceRequest()
        req.action = 1
        req.objmode = 0
        req.goal_position.header.frame_id = 'base_link'
        req.goal_position.point.x = 0.8
        req.goal_position.point.y = 0.0
        req.goal_position.point.z = 0.7
        res1 = grasp(req)
        xm_arm_moveit_name('after_grasp_things')
        gripper_service_level(2)
        req.action = 3
        req.goal_position.point.y += 0.2
        res2 = grasp(req)
        xm_arm_moveit_name('after_grasp_things')
        gripper_service_level(2)
        req.goal_position.point.y += 0.2
        res3 = grasp(req)
    

if __name__ == "__main__":
    laiya()