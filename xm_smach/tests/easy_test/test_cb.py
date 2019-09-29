#!/usr/bin/env python
#encoding:utf-8
# import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if userdata.foo_counter_in < 3:
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
        return 'outcome1'
        



class main():
    def __init__(self):
        rospy.init_node('smach_example_state_machine')

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['outcome4'])
        self.sm.userdata.sm_counter = 0

        # Open the container
        with self.sm:
            # Add states to the container
            self.sm.add('FOO', Foo(), 
                                transitions={'outcome1':'BAR', 
                                                'outcome2':'outcome4'},
                                remapping={'foo_counter_in':'sm_counter', 
                                            'foo_counter_out':'sm_counter'})
            self.sm.add('BAR', Bar(), 
                                transitions={'outcome1':'FOO'},
                                remapping={'bar_counter_in':'sm_counter'})
        self.sm.register_termination_cb(self.test_cb)


        # Execute SMACH plan
        outcome = self.sm.execute()
        print outcome
    def test_cb(self, userdata ,terminal_states,container_outcome):
        if terminal_states[0] == 'FOO':
            print userdata.sm_counter
            userdata.sm_counter = 0
            container_outcome = 'fuck'


if __name__ == '__main__':
    main()
