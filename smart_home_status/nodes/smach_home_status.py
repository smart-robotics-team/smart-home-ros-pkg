#!/usr/bin/env python

""" .py - Version 1.0 2016-02-20

    Control a robot to patrol four waypoints chosen at random

    Created for 
    Copyright (c) 2016 Joffrey Kriegel.  All rights reserved.

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
from std_msgs.msg import UInt16, Int32, Empty, String
from random import randrange
from tf.transformations import quaternion_from_euler
from math import  pi
from collections import OrderedDict

# GLOBAL VARIABLE ! VERY BAD !
jo_status = 0 # 0:awake 1:sleep 2:out 
carole_status = 0 # 0:awake 1:sleep 2:out

class Pause(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass

    def execute(self, userdata):
        rospy.loginfo("Thinking...")
        rospy.sleep(0.5)   
        return 'succeeded'


class PreparingShower(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.heat_pub = rospy.Publisher('/HOME/showerHeatON', Empty)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
        pass

    def execute(self, userdata):
	global jo_status
	global carole_status
        if (jo_status != 0) and (carole_status != 0): # No one is available for a shower
		rospy.loginfo("No shower possible")
		return 'aborted'
        elif ((jo_status == 0) and (carole_status != 0)) or ((jo_status != 0) and (carole_status == 0)): # No sound
		rospy.loginfo("Heating...")
        	self.heat_pub.publish(Empty())
        	rospy.sleep(300)
        	return 'succeeded'
	else :
		rospy.loginfo("Heating...")
                my_string = String()
                my_string.data = "La salle de bain est en train de chauffer. Vous pourrez l'utiliser dans cinq minutes."
                self.french_pub.publish(my_string)
                self.heat_pub.publish(Empty())
                rospy.sleep(300)
		return 'succeeded'

class GoShower(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
        pass

    def execute(self, userdata):
	global jo_status
	global carole_status
        rospy.loginfo("Going into shower...")
	if (jo_status != 1) and (carole_status != 1):
        	my_string = String()
        	my_string.data = "La salle de bain est chaude. Vous pouvez aller vous doucher."
        	self.french_pub.publish(my_string)
        rospy.sleep(600)
        return 'succeeded'

class StopShower(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.heat_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
        pass

    def execute(self, userdata):
	global jo_status
	global carole_status
        rospy.loginfo("Stop shower...")
        if (jo_status != 1) and (carole_status != 1):
		my_string = String()
        	my_string.data = "Il est temps de sortir de la douche."
        	self.french_pub.publish(my_string)
        self.heat_pub.publish(Empty())
        rospy.sleep(1)
        return 'succeeded'

class JoGoingSleep(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON1_pub = rospy.Publisher('/MILIGHT/light1ON', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.ON3_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):
	global jo_status
	global carole_status

	jo_status = 1

	if carole_status == 1: # Carole is already sleeping -> Dont say anything and dont light up restroom
                myint = Int32() 
                rospy.sleep(0.01)
                self.ON3_pub.publish(Empty())
                rospy.sleep(0.01)
                self.ON2_pub.publish(Empty())
                rospy.sleep(0.01)
                myint.data = 175
                self.color3_pub.publish(myint)
                rospy.sleep(0.01)
                myint.data = 10
                self.brightness2_pub.publish(myint)
                rospy.sleep(0.01)
                myint.data = 9
                self.brightness3_pub.publish(myint)
                rospy.sleep(0.01)


	else:
        	tosay = String()
        	myint = Int32()
        	tosay.data = "Il est l'heure d'aller dormir."
        	self.french_pub.publish(tosay)
		rospy.sleep(0.01)
		self.ON3_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.ON2_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.ON1_pub.publish(Empty())
        	rospy.sleep(0.01)
        	myint.data = 175
        	self.color1_pub.publish(myint)
        	rospy.sleep(0.01)
        	self.color3_pub.publish(myint)
        	rospy.sleep(0.01)
        	myint.data = 2
        	self.brightness1_pub.publish(myint)
        	rospy.sleep(0.01)
        	myint.data = 10
        	self.brightness2_pub.publish(myint)
        	rospy.sleep(0.01)
        	myint.data = 9
        	self.brightness3_pub.publish(myint)
        	rospy.sleep(0.01)

        return 'succeeded'

class JoInBed(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.OFF1_pub = rospy.Publisher('/MILIGHT/light1OFF', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.OFF2_pub = rospy.Publisher('/MILIGHT/light2OFF', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.OFF3_pub = rospy.Publisher('/MILIGHT/light3OFF', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
	self.heatOFF_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)

    def execute(self, userdata):

	global jo_status
	global carole_status

        if carole_status == 1: # Carole is already sleeping -> Dont say anything and dont light up restroom
 
                myint = Int32()
                myint.data = 6
                self.brightness2_pub.publish(myint)
                rospy.sleep(0.01)
                myint.data = 6
                self.brightness3_pub.publish(myint)
                rospy.sleep(0.3)

		# Shutdown everything
        	self.OFF3_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.OFF2_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.heatOFF_pub.publish(Empty())
        	rospy.sleep(0.01)

        else:

		tosay = String()
		myint = Int32()
        	tosay.data = "Bonne nuit."
        	self.french_pub.publish(tosay)
        	rospy.sleep(0.01)

        	myint.data = 1
        	self.brightness1_pub.publish(myint)
        	rospy.sleep(0.01)
        	myint.data = 6
        	self.brightness2_pub.publish(myint)
        	rospy.sleep(0.01)
        	myint.data = 6
        	self.brightness3_pub.publish(myint)
        	rospy.sleep(0.3)

		# Shutdown everything
        	self.OFF3_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.OFF2_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.OFF1_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.heatOFF_pub.publish(Empty())
        	rospy.sleep(0.01)

        return 'succeeded'



class BothGoingSleep(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON1_pub = rospy.Publisher('/MILIGHT/light1ON', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.ON3_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):
	global jo_status
	global carole_status

        jo_status = 1
        carole_status = 1

	tosay = String()
	myint = Int32()
        tosay.data = "Il est l'heure d'aller dormir."
        self.french_pub.publish(tosay)
        rospy.sleep(0.01)
        self.ON3_pub.publish(Empty())
        rospy.sleep(0.01)
        self.ON2_pub.publish(Empty())
        rospy.sleep(0.01)
        self.ON1_pub.publish(Empty())
        rospy.sleep(0.01)
        myint.data = 175
        self.color1_pub.publish(myint)
        rospy.sleep(0.01)
        self.color3_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 2
        self.brightness1_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 10
        self.brightness2_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 9
        self.brightness3_pub.publish(myint)
        rospy.sleep(0.01)

        return 'succeeded'


class BothInBed(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.OFF1_pub = rospy.Publisher('/MILIGHT/light1OFF', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.OFF2_pub = rospy.Publisher('/MILIGHT/light2OFF', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.OFF3_pub = rospy.Publisher('/MILIGHT/light3OFF', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
	self.heatOFF_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)

    def execute(self, userdata):


        tosay = String()
        myint = Int32()
        tosay.data = "Bonne nuit."
        self.french_pub.publish(tosay)
        rospy.sleep(0.01)

        myint.data = 1
        self.brightness1_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 6
        self.brightness2_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 6
        self.brightness3_pub.publish(myint)
        rospy.sleep(0.3)

        # Shutdown everything
        self.OFF3_pub.publish(Empty())
        rospy.sleep(0.01)
        self.OFF2_pub.publish(Empty())
        rospy.sleep(0.01)
        self.OFF1_pub.publish(Empty())
        rospy.sleep(0.01)
        self.heatOFF_pub.publish(Empty())
        rospy.sleep(0.01)

        return 'succeeded'

class JoWakingUp(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
	self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
	self.OFF1_pub = rospy.Publisher('/MILIGHT/light1OFF', Empty)
	self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.white3_pub = rospy.Publisher('/MILIGHT/light3White', Empty)
	self.ON_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.heatON_pub = rospy.Publisher('/HOME/showerHeatON', Empty)
        self.heatOFF_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)
        self.weather_pub = rospy.Publisher('/NESTOR/weather', Empty)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
        self.music_pub = rospy.Publisher('/NESTOR/radio_jap', Empty)

    def execute(self, userdata):

	global jo_status
	global carole_status

	jo_status = 0

        if carole_status == 1: # Carole is already sleeping -> Dont say anything and dont light up restroom

                #self.ON_pub.publish(Empty())
                myint = Int32()

                rospy.sleep(120)

                self.ON2_pub.publish(Empty())
                rospy.sleep(0.01)
                self.ON_pub.publish(Empty())
                rospy.sleep(0.01)

                self.white3_pub.publish(Empty())
                rospy.sleep(0.03)
                myint.data = 14
                self.brightness3_pub.publish(myint)

                rospy.sleep(120)
                rospy.sleep(0.01)
                self.heatON_pub.publish(Empty())
                rospy.sleep(0.01)
                #self.music_pub.publish(Empty())

                rospy.sleep(60)
                #self.weather_pub.publish(Empty())

                rospy.sleep(240)
                self.heatOFF_pub.publish(Empty())


	else:

		#self.ON_pub.publish(Empty())
        	myint = Int32()

		myint.data = 175
		self.color1_pub.publish(myint)
		rospy.sleep(0.01)
		myint.data = 1
                self.brightness1_pub.publish(myint)

        	rospy.sleep(120)

        	my_string=String()
        	my_string.data="Il est l'heure de se lever. Debout les feignants."
        	self.french_pub.publish(my_string)

        	self.ON2_pub.publish(Empty())
		rospy.sleep(0.01)
        	self.ON_pub.publish(Empty())
		rospy.sleep(0.01)

        	self.white3_pub.publish(Empty())
                rospy.sleep(0.03)
                myint.data = 14
                self.brightness3_pub.publish(myint)

		for brightness in range(2,9):
			color=175-2*brightness
			myint.data = color
			self.color1_pub.publish(myint)			
        		rospy.sleep(0.01)
			myint.data = brightness
			self.brightness1_pub.publish(myint)
                	rospy.sleep(30) 

		for brightness in range(10,16):
                        myint.data = brightness
                        self.brightness1_pub.publish(myint)
                        rospy.sleep(40)

        	rospy.sleep(120)
        	my_string.data="Chauffage de la salle de bain en cours."
        	self.french_pub.publish(my_string)
        	rospy.sleep(0.01)
        	self.heatON_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.music_pub.publish(Empty())
        	rospy.sleep(0.01)
        	self.OFF1_pub.publish(Empty())

        	rospy.sleep(60)
        	self.weather_pub.publish(Empty())

        	rospy.sleep(240)
        	self.heatOFF_pub.publish(Empty())

        return 'succeeded'

class BothWakingUp(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.OFF1_pub = rospy.Publisher('/MILIGHT/light1OFF', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.white3_pub = rospy.Publisher('/MILIGHT/light3White', Empty)
        self.ON_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.heatON_pub = rospy.Publisher('/HOME/showerHeatON', Empty)
        self.heatOFF_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)
        self.weather_pub = rospy.Publisher('/NESTOR/weather', Empty)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
        self.music_pub = rospy.Publisher('/NESTOR/radio_jap', Empty)

    def execute(self, userdata):

	global jo_status
	global carole_status

        jo_status = 0
        carole_status = 0

        #self.ON_pub.publish(Empty())
        myint = Int32()

        myint.data = 175
        self.color1_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 1
        self.brightness1_pub.publish(myint)

        rospy.sleep(120)

        my_string=String()
        my_string.data="Il est l'heure de se lever. Debout les feignants."
        self.french_pub.publish(my_string)

        self.ON2_pub.publish(Empty())
        rospy.sleep(0.01)
        self.ON_pub.publish(Empty())
        rospy.sleep(0.01)

        self.white3_pub.publish(Empty())
        rospy.sleep(0.03)
        myint.data = 14
        self.brightness3_pub.publish(myint)

        for brightness in range(2,9):
        	color=175-2*brightness
                myint.data = color
                self.color1_pub.publish(myint)
                rospy.sleep(0.01)
                myint.data = brightness
                self.brightness1_pub.publish(myint)
                rospy.sleep(30)

        for brightness in range(10,16):
                myint.data = brightness
                self.brightness1_pub.publish(myint)
                rospy.sleep(40)

        rospy.sleep(120)
        my_string.data="Chauffage de la salle de bain en cours."
        self.french_pub.publish(my_string)
        rospy.sleep(0.01)
        self.heatON_pub.publish(Empty())
        rospy.sleep(0.01)
        self.music_pub.publish(Empty())
        rospy.sleep(0.01)
        self.OFF1_pub.publish(Empty())

        rospy.sleep(60)
        self.weather_pub.publish(Empty())

        rospy.sleep(240)
        self.heatOFF_pub.publish(Empty())

        return 'succeeded'

class JoGoingOut(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON1_pub = rospy.Publisher('/MILIGHT/light1ON', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.ON3_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):

	global jo_status
	global carole_status

        jo_status = 2

        if carole_status == 1: # Carole is already sleeping ->
		return 'succeeded'
	else:
		return 'succeeded'

class JoIsOut(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON1_pub = rospy.Publisher('/MILIGHT/light1ON', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.ON3_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):

	global jo_status
	global carole_status

        if carole_status == 1: # Carole is already sleeping ->
                return 'succeeded'
        else:
                return 'succeeded'


class JoBackHome(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON1_pub = rospy.Publisher('/MILIGHT/light1ON', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.ON3_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):

	global jo_status
	global carole_status

        jo_status = 0

        if carole_status == 1: # Carole is already sleeping ->
                return 'succeeded'
        else:
                return 'succeeded'



class BothBackHome(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON1_pub = rospy.Publisher('/MILIGHT/light1ON', Empty)
        self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
        self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.ON3_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):

	global jo_status
	global carole_status

        jo_status = 0
        carole_status = 0

        return 'succeeded'
























class SMACHAI():
    def __init__(self):
        rospy.init_node('smach_home_status', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        






#####################################
# JO IS AWAKE
#####################################

	# State machine for Jo-awake-go-sleep
        self.sm_jo_awake_sleep = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo_awake_sleep:
	    StateMachine.add('LOOK_WAKE', MonitorState("/JO/sleep", Empty, self.empty_cb), transitions={'valid':'GOING_SLEEP', 'preempted':'preempted', 'invalid':'GOING_SLEEP'})
            StateMachine.add('GOING_SLEEP', JoGoingSleep(),
                             transitions={'succeeded':'LOOK_IN_BED'})
	    StateMachine.add('LOOK_IN_BED', MonitorState("/myo_disconnected", Empty, self.empty_cb), transitions={'valid':'IN_BED', 'preempted':'preempted', 'invalid':'IN_BED'})
	    StateMachine.add('IN_BED', JoInBed(),
                             transitions={'succeeded':'succeeded'})

        # State machine for Jo-awake-bothgo-sleep
        self.sm_jo_awake_bothsleep = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo_awake_bothsleep:
            StateMachine.add('LOOK_WAKE', MonitorState("/BOTH/sleep", Empty, self.empty_cb), transitions={'valid':'GOING_SLEEP', 'preempted':'preempted', 'invalid':'GOING_SLEEP'})
            StateMachine.add('GOING_SLEEP', BothGoingSleep(),
                             transitions={'succeeded':'LOOK_IN_BED'})
            StateMachine.add('LOOK_IN_BED', MonitorState("/myo_disconnected", Empty, self.empty_cb), transitions={'valid':'IN_BED', 'preempted':'preempted', 'invalid':'IN_BED'})
            StateMachine.add('IN_BED', BothInBed(),
                             transitions={'succeeded':'succeeded'})


        # State machine for Jo-awake-go-out
        self.sm_jo_awake_out = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo_awake_out:
            StateMachine.add('LOOK_OUT', MonitorState("/JO/go_out", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'succeeded'})


        # State machine for Jo-awake
        self.sm_jo_awake = Concurrence(outcomes=['succeeded', 'stop', 'go_sleep', 'go_out'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.jo_awake_child_termination_cb,
                                        outcome_cb=self.jo_awake_outcome_cb)

        with self.sm_jo_awake:
            Concurrence.add('SM_GO_TO_SLEEP', self.sm_jo_awake_sleep)
            Concurrence.add('SM_BOTH_GO_TO_SLEEP', self.sm_jo_awake_bothsleep)
            Concurrence.add('SM_GO_OUT', self.sm_jo_awake_out)


#####################################
# JO IS SLEEPING
#####################################

        # State machine for Jo-sleep-waking
        self.sm_jo_sleep_waking = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo_sleep_waking:
	    StateMachine.add('WAIT_WAKE_UP', MonitorState("/JO/wake_up", Empty, self.empty_cb), transitions={'valid':'WAKING_UP', 'preempted':'preempted', 'invalid':'WAKING_UP'})
            StateMachine.add('WAKING_UP', JoWakingUp(),
                             transitions={'succeeded':'succeeded'})

        # State machine for Jo-sleep-bothwaking
        self.sm_jo_sleep_bothwaking = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo_sleep_bothwaking:
            StateMachine.add('WAIT_WAKE_UP', MonitorState("/BOTH/wake_up", Empty, self.empty_cb), transitions={'valid':'WAKING_UP', 'preempted':'preempted', 'invalid':'WAKING_UP'})
            StateMachine.add('WAKING_UP', BothWakingUp(),
                             transitions={'succeeded':'succeeded'})



	# State machine for Jo-awake
        self.sm_jo_sleep = Concurrence(outcomes=['succeeded','aborted','preempted', 'wake_up'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.jo_sleep_child_termination_cb,
                                        outcome_cb=self.jo_sleep_outcome_cb)

        with self.sm_jo_sleep:
	    Concurrence.add('SM_WAKE_UP', self.sm_jo_sleep_waking)
	    Concurrence.add('SM_BOTH_WAKE_UP', self.sm_jo_sleep_bothwaking)


#####################################
# JO IS OUT         TODO
#####################################

        # State machine for Jo-out-back
        self.sm_jo_out_back = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo_out_back:
            StateMachine.add('WAIT_BACK_HOME', MonitorState("/JO/back_home", Empty, self.empty_cb), transitions={'valid':'WAIT_MYO', 'preempted':'preempted', 'invalid':'WAIT_MYO'})
            StateMachine.add('WAIT_MYO', MonitorState("/myo_connected", Empty, self.empty_cb), transitions={'valid':'COMING_BACK', 'preempted':'preempted', 'invalid':'COMING_BACK'})
            StateMachine.add('COMING_BACK', JoBackHome(),
                             transitions={'succeeded':'succeeded'})

        # State machine for Jo-out-bothback
        self.sm_jo_out_bothback = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo_out_bothback:
            StateMachine.add('WAIT_BACK_HOME', MonitorState("/BOTH/back_home", Empty, self.empty_cb), transitions={'valid':'WAIT_MYO', 'preempted':'preempted', 'invalid':'WAIT_MYO'})
            StateMachine.add('WAIT_MYO', MonitorState("/myo_connected", Empty, self.empty_cb), transitions={'valid':'COMING_BACK', 'preempted':'preempted', 'invalid':'COMING_BACK'})
            StateMachine.add('COMING_BACK', BothBackHome(), 
                             transitions={'succeeded':'succeeded'})



	# State machine for Jo-out
	self.sm_jo_out = Concurrence(outcomes=['succeeded','aborted','preempted', 'back_home'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.jo_out_child_termination_cb,
                                        outcome_cb=self.jo_out_outcome_cb)

        with self.sm_jo_out:
	    Concurrence.add('SM_BACK_HOME', self.sm_jo_out_back)
            Concurrence.add('SM_BOTH_BACK_HOME', self.sm_jo_out_bothback)


#####################################
# TOP LVL JO SM
#####################################

	# State machine for JO
        self.sm_jo = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_jo:
	    StateMachine.add('AWAKE', self.sm_jo_awake, transitions={'succeeded':'succeeded', 'stop':'aborted', 'go_sleep':'SLEEP', 'go_out':'OUT'})
	    StateMachine.add('SLEEP', self.sm_jo_sleep, transitions={'succeeded':'succeeded', 'wake_up':'AWAKE'})
	    StateMachine.add('OUT', self.sm_jo_out, transitions={'succeeded':'succeeded', 'back_home':'AWAKE'})


#####################################
# TOP LVL CAROLE SM        TODO
#####################################


        # State machine for CAROLE
        self.sm_carole = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_carole:
	    StateMachine.add('WAIT3', MonitorState("/TEST/wait3", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'WAIT3',
                                          'aborted':'aborted'})



#####################################
# TOP LVL EAT SM        TODO
#####################################


        # State machine for EAT
        self.sm_eat = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_eat:
	    StateMachine.add('WAIT2', MonitorState("/TEST/wait2", Empty, self.empty_cb), transitions={'valid':'PAUSE', 'preempted':'preempted', 'invalid':'PAUSE'})
            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded':'WAIT2',
                                          'aborted':'aborted'})


#####################################
# TOP LVL SHOWER SM 
#####################################



        # State machine for SHOWER
        self.sm_shower = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_shower:
	    StateMachine.add('WAIT_SHOWER', MonitorState("/HOME/go_shower", Empty, self.empty_cb), transitions={'valid':'PREPARING_SHOWER', 'preempted':'preempted', 'invalid':'PREPARING_SHOWER'})
            StateMachine.add('PREPARING_SHOWER', PreparingShower(),
                             transitions={'succeeded':'GO_SHOWER',
                                          'aborted':'WAIT1'})
            StateMachine.add('GO_SHOWER', GoShower(),
                             transitions={'succeeded':'STOP_SHOWER',
                                          'aborted':'aborted'})
            StateMachine.add('STOP_SHOWER', StopShower(),
                             transitions={'succeeded':'WAIT1',
                                          'aborted':'aborted'})



#####################################
# TOP LVL SM 
#####################################


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
        if outcome_map['SM_BOTH_GO_TO_SLEEP'] == 'succeeded':
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
        elif outcome_map['SM_BOTH_GO_TO_SLEEP'] == 'succeeded':
            return 'go_sleep'
        elif outcome_map['SM_GO_OUT'] == 'succeeded':
            return 'go_out'
        else:
            return 'stop'


    # Gets called when ANY child state terminates
    def jo_sleep_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_WAKE_UP'] == 'succeeded':
            return True
        if outcome_map['SM_BOTH_WAKE_UP'] == 'succeeded':
            return True
        else:
            return False


    # Gets called when ALL child states are terminated
    def jo_sleep_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['SM_WAKE_UP'] == 'succeeded':
            return 'wake_up'
        elif outcome_map['SM_BOTH_WAKE_UP'] == 'succeeded':
            return 'wake_up'
        else:
            return 'stop'

    # Gets called when ANY child state terminates
    def jo_out_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_BACK_HOME'] == 'succeeded':
            return True
        if outcome_map['SM_BOTH_BACK_HOME'] == 'succeeded':
            return True
        else:
            return False


    # Gets called when ALL child states are terminated
    def jo_out_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['SM_BACK_HOME'] == 'succeeded':
            return 'back_home'
        elif outcome_map['SM_BOTH_BACK_HOME'] == 'succeeded':
            return 'back_home'
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
