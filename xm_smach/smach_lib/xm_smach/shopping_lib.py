#!/usr/bin/env python
# encoding:utf8
# this module only contants the simple states usually used, the smach_ros we will directly use in the scripts
# userdata of smach-container includes the whole userdata interface ,you can achieve the different interface by try-except function
# no param specified error should be raise by state itself so it is easy for us to debug
import rospy
from smach import State,UserData,StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
#import xm_speech.srv
from xm_msgs.msg import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool,Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os

class CheckStop(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['remeber','stop','aborted'],
                        input_keys = ['PT_LIST','mission'],
                        output_keys= ['PT_LIST','mission'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            while not rospy.is_shutdown():
                self.target_client.wait_for_service(timeout = 10)
                self.response = self.target_client.call(command = 6)
                self.action = self.response.action
                self.object = self.response.object
                self.target = self.response.target
                rospy.logwarn(self.action)
                if len(self.object) == 1:
                    # subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)

                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')

                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
   
                    
                    userdata.PT_LIST[str(self.object[0])] = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    return 'remeber'
                elif len(self.action)>0 and self.action[0] == 'stop' and len(userdata.PT_LIST) >= 3:
                    rospy.logwarn('I will stop!!')
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    rospy.logerr(userdata.mission)
                    rospy.logerr(userdata.PT_LIST)
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    return 'stop'
                elif len(self.object) == 3:
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    rospy.logerr(userdata.mission)
                    rospy.logerr(userdata.PT_LIST)
                    userdata.mission['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    return 'stop'
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'

class CheckStop2(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['remeber','stop','aborted'],
                        input_keys = ['PT_LIST','mission'],
                        output_keys= ['PT_LIST','mission'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            while not rospy.is_shutdown():
                self.target_client.wait_for_service(timeout = 10)
                self.response = self.target_client.call(command = 6)
                self.action = self.response.action
                self.object = self.response.object
                self.target = self.response.target
                rospy.logwarn(self.action)
                rospy.logerr(self.object)
                rospy.logwarn("len"+str(len(userdata.PT_LIST)))
                if len(self.object) == 1:
                    # subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)

                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')

                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))

                    if len(self.target)!= 0:
                        rospy.logerr("target")
                        rospy.logerr(self.target)
                        eulerVec = euler_from_quaternion(rot)
                        eulerVec2 = eulerVec[2]
                        if self.target[0] == 'left':
                            
                            eulerVec2 = eulerVec[2]+pi/2
                            
                        elif self.target[0] == 'right':
                            eulerVec2 = eulerVec[2]-pi/2

                        rot = quaternion_from_euler(eulerVec[0] , eulerVec[1] , eulerVec2)
                            

                    
                    userdata.PT_LIST[str(self.object[0])] = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    return 'remeber'
                elif len(self.action)>0 and self.action[0] == 'stop' and len(userdata.PT_LIST)>=3:
                    rospy.logwarn('I will stop!!')
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    rospy.logerr(userdata.PT_LIST)
                    return 'stop'
                elif len(self.object) == 3:
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    rospy.logerr(userdata.PT_LIST)
                    return 'stop'
                # else:
                #     rospy.logwarn("FUCK :")
                #     return 'remeber'
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'
        
# 运行跟随人的节点
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.Popen('xterm -e rosrun xm_vision people_tracking &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'

class GetMission(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],output_keys = ['things_list'])
        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10)
            self.response = self.client.call(command=6)
            self.object = self.response.object
            userdata.things_list = self.object
            return 'succeeded'
        except Exception,e:
            rospy.logwarn(e)
            return 'aborted'

class GetTarget(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','finish'],
                        output_keys = ['pos_xm','mission_name'],
                        input_keys = ['mission'])
    def execute(self,userdata):
        try:
            self.mission = userdata.mission
            rospy.logwarn(self.mission)
        except Exception,e:
            rospy.logerr('no  specific value')
            rospy.logerr(e)
        try: 
            if len(self.mission) >1:
                key_list = self.mission.keys()
                for i in key_list:
                    if i != 'cashier':
                        thing = self.mission.pop(i)
                        userdata.mission_name = i
                        userdata.pos_xm = thing
                        rospy.logwarn('finish one mission')
                        return 'succeeded'
            elif len(self.mission) == 1:
                thing = self.mission['cashier']
                userdata.pos_xm = thing
                rospy.logwarn('complete the mission')
                return 'finish'
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'

class GetSignal(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','aborted'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)

    def execute(self,userdata):
        try:
            self.target_client.wait_for_service(timeout = 10)
            self.response = self.target_client.call(command = 6)
            rospy.logwarn(self.response.action)
            if self.response.action[0] == 'follow':
                return 'succeeded'
            else:
                return 'aborted'
        except Exception,e:
            rospy.logerr('xm meet wrong when get follow signal')
            rospy.logerr(e)
            return 'aborted'

class FindPeople():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])
        self.tf_listener = tf.TransformListener()

# 如果相机找到人,这个状态将会返回False
# 相反,如果在五个循环里相机都没找到人,将会返回True
# msg传入主题的数据
    def people_cb(self,userdata,msg):
        print msg
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)
                rospy.logwarn('finding people........')
                if self.tmp_pos.point.x==10 and tmp_pos.point.y == 10 and tmp_pos.point.z == 10:
                    rospy.logwarn('no people')
                    pass
            #如果得到人的坐标信息返回移动的位置
                elif self.get_distance(self.tmp_pos)>0.5 and self.get_distance(self.tmp_pos)<=4.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                else:
                    rospy.logerr('the person is out of the range')
                    return True
            except:
                rospy.logerr(e)
                return True
            
        else:
            raise Exception('MsgNotFind')
    def get_distance(self,pos_xm):
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        person_x = person_x - 1.0*cos(angle)
        person_y = person_y - 1.0*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
#返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
#        person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
#        person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        person_x = person_x - 0.8*cos(angle)
        person_y = person_y - 0.8*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0

        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    

        #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

# 延续(苟)一段时间的简单状态,rec为延续(苟)的时间
class Wait(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error'],
        input_keys=['rec'])

    def execute(self, userdata):
        try:
            self.rec = userdata.rec
        except:
            rospy.logerr('no param specified')
            return 'error'
        else:  
            rospy.sleep(userdata.rec)
            return "succeeded"

class Wait_trace(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error','preempted'])

    def execute(self, userdata):
        for i in range(0,20):
            if self.preempt_requested():
                return 'preempted'
            else:
                rospy.sleep(0.1)
        return "succeeded"

class RunNode_img(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen('xterm -e rosrun xm_vision image_test &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'
