#!/usr/bin/env python

""" random_patrol_smach.py - Version 1.0 2013-04-12

    Control a robot to patrol four waypoints chosen at random

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
import smach
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from actionlib import GoalStatus
from std_msgs.msg import UInt16, Int32, Empty
from random import randrange
from tf.transformations import quaternion_from_euler
from math import  pi
from collections import OrderedDict

class Pause(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass

    def execute(self, userdata):
        rospy.loginfo("Thinking...")
        rospy.sleep(0.5)   
        return 'succeeded'




class SMACHAI():
    def __init__(self):
        rospy.init_node('smach_home_status', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        


        self.jo_status = 0 # 0:awake 1:sleep 2:out 
        self.carole_status = 0 # 0:awake 1:sleep 2:out




	# State machine for Jo-awake-go-sleep
        self.sm_jo_awake_sleep = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_jo_awake_sleep.userdata.test = 0.1

        with self.sm_jo_awake_sleep:
	    StateMachine.add('LOOK_WAKE', MonitorState("/HOME/wake_up", Empty, self.empty_cb), transitions={'valid':'PAUSE1', 'preempted':'preempted', 'invalid':'PAUSE1'})
            StateMachine.add('PAUSE1', Pause(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})

        # State machine for Jo-awake-go-out
        self.sm_jo_awake_out = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_jo_awake_out.userdata.test = 0.1

        with self.sm_jo_awake_out:
            StateMachine.add('LOOK_OUT', MonitorState("/HOME/jo_go_out", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})


        # State machine for Jo-awake
        self.sm_jo_awake = Concurrence(outcomes=['succeeded', 'stop', 'go_sleep', 'go_out'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.jo_awake_child_termination_cb,
                                        outcome_cb=self.jo_awake_outcome_cb)

        with self.sm_jo_awake:
            Concurrence.add('SM_GO_TO_SLEEP', self.sm_jo_awake_sleep)
            Concurrence.add('SM_GO_OUT', self.sm_jo_awake_out)

	# State machine for Jo-sleep
        self.sm_jo_sleep = StateMachine(outcomes=['succeeded','aborted','preempted', 'wake_up'])
        self.sm_jo_sleep.userdata.test = 0.1

        with self.sm_jo_sleep:
            StateMachine.add('WAIT_WAKE_UP', MonitorState("/HOME/jo_wake_up", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'wake_up',
                                          'aborted':'aborted'})

	# State machine for Jo-out
        self.sm_jo_out = StateMachine(outcomes=['succeeded','aborted','preempted', 'back_home'])
        self.sm_jo_out.userdata.test = 0.1

        with self.sm_jo_out:
            StateMachine.add('WAIT_BACK_HOME', MonitorState("/HOME/jo_back_home", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'back_home',
                                          'aborted':'aborted'})



	# State machine for JO
        self.sm_jo = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_jo.userdata.test = 0.1

        with self.sm_jo:
	    StateMachine.add('AWAKE', self.sm_jo_awake, transitions={'succeeded':'PAUSE', 'stop':'aborted', 'go_sleep':'SLEEP', 'go_out':'OUT'})
	    StateMachine.add('SLEEP', self.sm_jo_sleep, transitions={'succeeded':'PAUSE', 'wake_up':'AWAKE'})
	    StateMachine.add('OUT', self.sm_jo_out, transitions={'succeeded':'PAUSE', 'back_home':'AWAKE'})
	    StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})


        # State machine for CAROLE
        self.sm_carole = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_carole.userdata.test = 0.1

        with self.sm_carole:
	    StateMachine.add('WAIT3', MonitorState("/TEST/wait3", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'WAIT3',
                                          'aborted':'aborted'})



        # State machine for EAT
        self.sm_eat = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_eat.userdata.test = 0.1

        with self.sm_eat:
	    StateMachine.add('WAIT2', MonitorState("/TEST/wait2", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'WAIT2',
                                          'aborted':'aborted'})



        # State machine for SHOWER
        self.sm_shower = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_shower.userdata.test = 0.1

        with self.sm_shower:
	    StateMachine.add('WAIT1', MonitorState("/TEST/wait1", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'WAIT1',
                                          'aborted':'aborted'})





        # Create the top level state machine
        self.sm_top = Concurrence(outcomes=['succeeded', 'stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)

        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
	    Concurrence.add('SM_JO', self.sm_jo)
	    Concurrence.add('SM_CAROLE', self.sm_carole)
	    Concurrence.add('SM_EAT', self.sm_eat)
	    Concurrence.add('SM_SHOWER', self.sm_shower)








        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()


    def empty_cb(self, userdata, msg):
        return False

    def time_cb(self, userdata, msg):
        if msg.data < 2:
            self.stopping = True
            return False
        else:
            self.stopping = False
            return True

    def start_cb(self, userdata, msg):
	rospy.loginfo("Start !")
        return False

    def color_cb(self, userdata, msg):
        rospy.loginfo("Color " + str(msg.data))
	self.robot_side = msg.data

	self.sm_action1.userdata.robot_side = self.robot_side
	self.sm_action2.userdata.robot_side = self.robot_side
	self.sm_action3.userdata.robot_side = self.robot_side
	self.sm_action4.userdata.robot_side = self.robot_side
	self.sm_action5.userdata.robot_side = self.robot_side
	self.sm_action6.userdata.robot_side = self.robot_side
	self.sm_action7.userdata.robot_side = self.robot_side
	
	self.sm_top.userdata.robot_side = self.robot_side # TODO REMOVE

        return False

    def battery_cb(self, userdata, msg):
        if msg.data < 320:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True


    # Gets called when ANY child state terminates
    def concurrence_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_JO'] == 'succeeded':
            return True
        if outcome_map['SM_CAROLE'] == 'succeeded':
            return True
        if outcome_map['SM_EAT'] == 'succeeded':
            return True
        if outcome_map['SM_SHOWER'] == 'succeeded':
            return True
        else:
            return False


    # Gets called when ALL child states are terminated
    def concurrence_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['SM_JO'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['SM_CAROLE'] == 'succeeded':
            return 'succeeded'
        # Otherwise, if the last nav goal succeeded, return 'succeeded' or 'stop'
        elif outcome_map['SM_EAT'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['SM_SHOWER'] == 'succeeded':
            return 'succeeded'
        else:
            return 'stop'


    # Gets called when ANY child state terminates
    def jo_awake_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_GO_TO_SLEEP'] == 'succeeded':
            return True
        if outcome_map['SM_GO_OUT'] == 'succeeded':
            return True
        else:
            return False


    # Gets called when ALL child states are terminated
    def jo_awake_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['SM_GO_TO_SLEEP'] == 'succeeded':
            return 'go_sleep'
        elif outcome_map['SM_GO_OUT'] == 'succeeded':
            return 'go_out'
        else:
            return 'stop'







    def shutdown(self):
        rospy.loginfo("Stopping the home automation...")
        
        self.sm_top.request_preempt()
        
        
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        SMACHAI()
    except rospy.ROSInterruptException:
        rospy.loginfo("Home automation finished.")
