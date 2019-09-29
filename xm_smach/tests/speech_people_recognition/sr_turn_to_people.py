#!/usr/bin/env python
# encoding:utf8

# 语音与人脸识别项目测试程序最终版
# 将原本前后5个问题的分布改成了一直循环
# 没有加入麦克风阵列，等待改进
# 状态机顺序：开场白->等待测试人员排成一排->转身->人脸识别->汇报识别情况->请求提问->回答问题(10个)
import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator 
from smach_ros import IntrospectionServer,MonitorState
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.speech_reco_lib import *
from subprocess import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math
import os


class Run_Sound_Node(State):
    def __init__(self):
        State.__init__(self , outcomes = ['succeeded' , 'aborted'])
    
    def execute(self , userdata):
        try:
            subprocess.Popen('xterm -e roslaunch hark_sound_source_localization pr2_kinect.launch &' , shell = True)
            return 'succeeded'
        
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'

class people_pos():
    def __init__(self):
        
        rospy.sleep(5.0)
        self.people_direct  = MonitorState('/hark_source' , HarkSource ,  self.hark_cb , 
                                            max_checks = 30 , 
                                            output_keys = ['turn_degree'] )
    
    def hark_cb(self , userdata , msg):
        try:
            rospy.sleep(3.0)
            userdata.turn_degree = 0
            rospy.logerr(userdata.turn_degree)
            return False
            if msg is not None:
                rospy.logwarn(msg)
                if msg.src is not None:
                    if msg.src[0].azimuth > 7.0 and msg.src[0].azimuth < 16.0:
                        userdata.turn_degree = -72/180*math.pi
                        return False
                    elif msg.src[0].azimuth < -7.0 and msg.src[0].azimuth > -15.0:
                        userdata.turn_degree = 72/180*math.pi
                        return False
                    elif msg.src[0].azimuth >=16  or msg.src[0].azimuth<= -15.0:
                        userdata.turn_degree = -144/180*math.pi
                        return False
                    else:
                        userdata.turn_degree = 0
                        return False
            else:
                return True
        except Exception,e:
            rospy.logerr(e)
            return False
 
class SpeechRecognition():
    def __init__(self):
        rospy.init_node('speech_recognition')
        rospy.on_shutdown(self.shutdown)

        self.find_people = StateMachine(outcomes = ['succeeded','aborted','error'],input_keys = ['people_condition'],output_keys = ['people_condition'])
        self.test_bool = False

        with self.find_people:
            #这里用Pose类型代替传输三个数据
            #第一个是总人数
            #第二个是站着的人数
            #第三个是坐着的人数
            
            self.find_people.userdata.sentences = ''
            self.find_people.userdata.rec = 13.0
            # StateMachine.add('RUNNODE',
            #                     RunNode_Num(),
            #                     transitions={'succeeded':'succeeded','aborted':'aborted'})
            # StateMachine.add('RUNNODE',
            #                     RunNode_Num(),
            #                     transitions={'succeeded':'WAIT','aborted':'aborted'})
            # StateMachine.add('WAIT',
            #                     Wait(),
            #                     transitions={'succeeded':'GET_PEOPLE_NUM','error':'error'})
            StateMachine.add('GET_PEOPLE_NUM',
                                CountPeople(),
                                transitions={'succeeded':'succeeded','aborted':'aborted'})
            StateMachine.add('GET_SENTENCE',
                                GetSentences(),
                                transitions={'succeeded':'SPEAK_SIZE','error':'error'},
                                remapping = {'sentences':'sentences'})
            StateMachine.add('SPEAK_SIZE',
                                Speak(),
                                transitions={'succeeded':'succeeded','error':'error'},
                                remapping={'sentences':'sentences'})
            
            
            # StateMachine.add('CLOSE_KINECT',
            #                     CloseKinect(),
            #                     transitions ={'succeeded':'succeeded','aborted':'aborted'})
        
        self.TurnToPeo = StateMachine(outcomes = ['succeeded', 'aborted' ,'error'])

        with self.TurnToPeo:
            self.TurnToPeo.userdata.turn_degree = 0
            StateMachine.add('PEOPLE_POS' , people_pos().people_direct, 
                            transitions = {'invalid':'TURN' , 'valid' : 'PEOPLE_POS' , 'preempted':'TURN'},
                            remapping = {'turn_degree' : 'turn_degree'})
            StateMachine.add('TURN' , TurnDegree(),
                            transitions = {'succeeded':'succeeded' , 'aborted':'succeeded' , 'error':'error'} , 
                            remapping = {'degree' : 'turn_degree'})

        self.speech_rec = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.speech_rec:
            self.speech_rec.userdata.sentences_star = 'i want to play a riddle game'
            self.speech_rec.userdata.wait = 17.0
            self.speech_rec.userdata.degree = math.pi
            self.speech_rec.userdata.people_condition = {}
            self.speech_rec.userdata.sentences = ''
            # StateMachine.add('SPEAK_1',
            #                     Speak(),
            #                     transitions={'succeeded':'WAIT_1', 'aborted':'SPEAK_1', 'error':'error'},
            #                     remapping={'sentences':'sentences_star'})
            # StateMachine.add('WAIT_1',
            #                     Wait(),
            #                     transitions={'succeeded':'SPEAK_2','error':'error'},
            #                     remapping={'rec':'wait'})
            StateMachine.add('RECO',
                                self.find_people,
                                transitions={'succeeded':'ANS1','aborted':'aborted','error':'error'})
            StateMachine.add('ANS1',
                                Answer3() , 
                                transitions = {'succeeded':'SPEAK_2' , 'aborted':'aborted'})
                        
            self.speech_rec.userdata.sentences1 = 'who want to play a riddle game with me'
            StateMachine.add('SPEAK_2',
                                Speak(),
                                transitions={'succeeded':'TURN_AND_ANS', 'aborted':'SPEAK_2', 'error':'error'},
                                remapping={'sentences':'sentences1'})
            self.speech_rec.rec =5.0
            StateMachine.add('WAIT' , 
                                Wait() , 
                                transitions = {'succeeded':'TURN_AND_ANS'  , 'error':'TURN_AND_ANS'},
                                remapping = {'rec':'rec'})

             

            # StateMachine.add('RUN_SN' , 
            #                     Run_Sound_Node() , 
            #                     transitions = {'succeeded':'TURN_AND_ANS' , 'aborted':'aborted'})
            
            
            self.TurnAndAnswer = Concurrence(outcomes = ['succeeded' , 'aborted' , 'error'],
                                                default_outcome = 'aborted',
											    input_keys = ['sentences','people_condition'],
                                                output_keys = ['sentences'],
                                                outcome_map = {'succeeded':{'TURN_TO_PEO':'succeeded',
																		    'ANSWER':'succeeded'},},
											    child_termination_cb = self.child_cb)
            
            with self.TurnAndAnswer:
                Concurrence.add('TURN_TO_PEO' , 
                                    self.TurnToPeo)

                Concurrence.add('ANSWER',
                                    Answer2() , remapping = {'sentences':'sentences', 
                                                                'people_condition':'people_condition'})
                Concurrence.add('RUN_NS' , Run_Sound_Node())

            StateMachine.add('TURN_AND_ANS' , self.TurnAndAnswer , 
							transitions = {'succeeded':'WAIT_2' , 'aborted':'WAIT_2' , 'error':'error'},
                            remapping = {'sentences':'sentences'})
							
            
            self.speech_rec.userdata.wait = 3.0
            StateMachine.add('WAIT_2',
                                Wait(),
                                transitions={'succeeded':'SPEAK_REC','error':'error'},
                                remapping={'rec':'wait'})

            StateMachine.add('SPEAK_REC' ,
                                Speak() , 
                                transitions = {'succeeded':'SPEAK_2' , 'aborted':'SPEAK_2' , 'error':'error'} ,
                                remapping = {'sentences':'sentences'})

            
			
        
        out = self.speech_rec.execute()
        if out  == 'succeeded':
            self.test_bool = True
            rospy.logerr('test finished, all thing is done well!')

    def shutdown(self):
        if self.test_bool == False:
            
            rospy.logerr('test failed, there is no chicken dinner!')

    def child_cb(self , outcome_map):
        if outcome_map['TURN_TO_PEO'] and outcome_map['ANSWER']:
            return True
        else :
            return False

if __name__ == "__main__":
    try:
        SpeechRecognition()
    except:
        rospy.logerr('test wrong!')
        subprocess.call("rosnode kill -a")
