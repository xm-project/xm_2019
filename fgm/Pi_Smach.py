#!/usr/bin/env python
# encoding: utf8


import rospy,sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist,PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from visualization_msgs.msg import Marker
from math import  pi
from rbx2_msgs.srv import *
from collections import OrderedDict

#创建初始环境
def setup_environment(self):
    self.low_battery_threshold =rospy.get_param('~low_battery_threshold',50)
    self.move_base_timeout = rospy.get_param("~move_base_timeout", 60)
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    
    self.move_base.wait_for_server(rospy.Duration(60))
    
    rospy.loginfo("Connected to move_base action server")

    quaternions=list()

    euler_angles=(0,pi)

    for angle in euler_angles:
        q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
        q = Quaternion(*q_angle)
        quaternions.append(q)
    
    self.waypoints = list()
    
    self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[0]))
    self.waypoints.append(Pose(Point(0.0,1.0, 0.0), quaternions[1]))
    self.waypoints.append(Pose(Point(0.0,0.0, 0.0), quaternions[0]))
    self.docking_station_pose = (Pose(Point(0.5, 0.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))

class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

#输入机器人移动位置坐标的状态
class InputXY(State):
    def __init__(self):
      State.__init__(self,outcomes=['succeeded'],output_keys=['X_output','Y_output'])
    
    def execute(self,userdata):
       X_output=input("where u wanna go? X=")
       Y_output=input("Y=")
       userdata.X_output=X_output
       userdata.Y_output=Y_output
       return 'succeeded'

#移向指定目标
class ChoseWay(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','preempted'],
                        input_keys=['X','Y'])

        self.move_base=actionlib.SimpleActionClient("move_base",MoveBaseAction)

        self.move_base.wait_for_server(rospy.Duration(60))

        self.goal=MoveBaseGoal()
        self.goal.target_pose.header.frame_id='map'

    def execute(self,userdata):
        X=userdata.X
        Y=userdata.Y
        q_angle = quaternion_from_euler(0, 0, pi, axes='sxyz')
        q = Quaternion(*q_angle)
        self.goal.target_pose.pose=(Pose(Point(X, Y, 0.0), q))
        self.move_base.send_goal(self.goal)                                
        
        if self.preempt_requested():
           self.sevice_preempt()
           return 'preempted'

        finished_within_time=self.move_base.wait_for_result(rospy.Duration(60))

        if not finished_within_time:
            self.move_base.cancel_goal()
            return 'aborted'

        else :
            state=self.move_base.get_state()
            return 'succeeded' 

#移动机械臂到指定坐标

class  Move_Arm(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        moveit_commander.roscpp_initialize(sys.argv)
        self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
       
        self.right_arm.set_end_effector_link('right_gripper_link') 
        self.end_effector_link =self.right_arm.get_end_effector_link()
                        
        self.reference_frame = 'base_footprint'
        
        self.right_arm.set_pose_reference_frame(self.reference_frame)
                
        self.right_arm.allow_replanning(True)
        
    
        self.right_arm.set_goal_position_tolerance(0.01)
        self.right_arm.set_goal_orientation_tolerance(0.1)

       
    
    def execute(self,userdata):
        self.right_arm.set_named_target('resting')
        self.right_arm.go()
        rospy.sleep(2)
               
    
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x =input("where you wanna the arm go? x=")
        target_pose.pose.position.y = input("y=")
        target_pose.pose.position.z =input("z=")
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0
        
        self.right_arm.set_start_state_to_current_state()
        
        self. right_arm.set_pose_target(target_pose,self.end_effector_link)
        
        traj = self.right_arm.plan()
        
        self.right_arm.execute(traj)
        rospy.sleep(1)
         
        self.right_arm.set_named_target('resting')
        self.right_arm.go()
        moveit_commander.roscpp_shutdown()
        return 'succeeded'

    
    




class Patrol():
    def __init__(self):
        rospy.init_node('Pi_Smach',anonymous=False)

        rospy.on_shutdown(self.shutdown)
        
    

        setup_environment(self)

        self.last_nav_state = None

        self.recharging = False
        nav_states = list()

        for waypoint in self.waypoints:           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = waypoint
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, 
                                                 exec_timeout=rospy.Duration(60.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            nav_states.append(move_base_state)  

        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose = self.docking_station_pose
        nav_docking_station = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, 
                                             exec_timeout=rospy.Duration(60.0),
                                             server_wait_timeout=rospy.Duration(10.0)) 
        
        self.sm_nav = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        self.sm_nav.userdata.X_input=0
        self.sm_nav.userdata.Y_input=0
        with self.sm_nav:
            StateMachine.add('NAV_STATE_0', nav_states[0], transitions={'succeeded':'NAV_STATE_1','aborted':'NAV_STATE_1'})
            StateMachine.add('NAV_STATE_1', nav_states[1], transitions={'succeeded':'INPUTXY','aborted':'INPUTXY'})
            StateMachine.add('INPUTXY',InputXY(),transitions={'succeeded':'GOCHOSEWAY'},
                                                remapping={'X_output':'X_input',
                                                           'Y_output':'Y_input'})

            StateMachine.add('GOCHOSEWAY',ChoseWay(),transitions={'succeeded':'MOVE_ARM','aborted':'MOVE_ARM'},
                                                    remapping={'X':'X_input',
                                                              'Y':'Y_input'})

            StateMachine.add('MOVE_ARM',Move_Arm(),transitions={'succeeded':'NAV_STATE_2'})

            StateMachine.add('NAV_STATE_2',nav_states[2],transitions={'succeeded':'','aborted':''})                                              
                                                    


        

        self.sm_nav.register_transition_cb(self.nav_transition_cb, cb_args=[])

        self.sm_recharge = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        with self.sm_recharge:
            StateMachine.add('NAV_DOCKING_STATION', nav_docking_station, transitions={'succeeded':'RECHARGE_BATTERY'})
            StateMachine.add('RECHARGE_BATTERY', ServiceState('battery_simulator/set_battery_level', SetBatteryLevel, 100, response_cb=self.recharge_cb), 
                             transitions={'succeeded':''})        

        
        self.nav_patrol = Concurrence(outcomes=['succeeded', 'recharge', 'stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)

        with self.nav_patrol:
           Concurrence.add('SM_NAV', self.sm_nav)
           Concurrence.add('MONITOR_BATTERY', MonitorState("battery_level", Float32, self.battery_cb))


        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])   

        with self.sm_top:
            StateMachine.add('PATROL', self.nav_patrol, transitions={'succeeded':'PATROL', 'recharge':'RECHARGE', 'stop':'STOP'}) 
            StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''})

        
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()

        sm_outcome = self.sm_top.execute()

        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))

        intro_server.stop()
       

    def nav_transition_cb(self, userdata, active_states, *cb_args):
        self.last_nav_state = active_states    

    def concurrence_child_termination_cb(self, outcome_map):
        if outcome_map['SM_NAV'] == 'succeeded':
            return True
        
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            rospy.loginfo("LOW BATTERY! NEED TO RECHARGE...")
            if self.last_nav_state is not None:
                self.sm_nav.set_initial_state(self.last_nav_state, UserData())
            return True
        else:
            return False  


    def concurrence_outcome_cb(self, outcome_map):
       
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            return 'recharge'
       
        elif outcome_map['SM_NAV'] == 'succeeded':
            self.sm_nav.set_initial_state(['NAV_STATE_2'], UserData())
            return 'stop'
       
        else:
            return 'recharge'
        
    def battery_cb(self, userdata, msg):
        if msg.data < self.low_battery_threshold:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True
        
    def recharge_cb(self, userdata, response):
        return 'succeeded'
        
   
   

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_nav.request_preempt()
        
        #self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")   