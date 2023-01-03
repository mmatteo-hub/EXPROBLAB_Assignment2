#!/usr/bin/env python

"""
.. module:: recahrge
	:platform: ROS
	:synopsis: Class for Recharge state of the finite state machine

.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class represent the recharging state of the finite state machine. This state is executed every time the battery of the robot is low and the robot is located in the recharging room.
In particular, if the robot is in a location and it can reach the recharging room then the FSM enters in this state, brings the robot to the recharging room and then recharges it.
If the robot is in a location and it cannot reach the recharging room this state is not executed because the robot could not physically enter in the rechargin location.
At the end if the robot is already in the recharging location then the recharging state just recharges the robot and returns an execute action that allows the program know the robot can start again moving randonmly.
The recharging method takes more or less 5 seconds to recharge the robot completely to 100% before let it go away again.
The movement or the robot is performed thanks to the :mod:`planner` and the :mod:`controller` through a request to the servers by the clients used.
The location of the robot is checked by a query by the aRMOR client and its respective server.
The clients are taken from the helper object initialized in the state.

Clients:
	:attr:`client`: aRMOR client used to update the timestamp of the robot and of the location the robot has just visited
	
	:attr:`planner_client`: client from which it is taken the goal, which means the path just computed to be passed to the controller.
	
	:attr:`controller_client`: client that makes the request to the respective server to move the robot through the path from the starting to the end position.
"""

import sys
import os
import roslib
import rospy
import actionlib
import smach
import smach_ros
import time
from EXPROBLAB_Assignment2 import name_mapper as nm

class Recharge(smach.State):
	def __init__(self, helper):
		"""
		This function is used to initialize the Recharge state for the finite state machine.
		It is used also the helper object imported from the Helper class to use the shared variables and the mutex.
		
		Args:
			helper(Helper): object that helps the sharing of information relating the shared variables and the mutex among the states of the machine
		
		Returns:
			none
		"""
		self._helper = helper
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED, nm.MOVE_RANDOM],
								input_keys=['recharge_counter_in'],
								output_keys=['recharge_counter_out'])
		
	def execute(self, userdata):
		"""
		Function that is executed every time the machine enters the state.
		It is responsible of returning a state transitioning to change the state.
		It uses the mutex instantiated in the helper to manage the variable access.
		
		Args:
			userdata: pointer to pass the data among the states of a State Machine
			
		Returns:
			transition(String): string containing the label of the action performed and used to change state in the machine.
		"""
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.RECHARGE + ' (users = %f)'%userdata.recharge_counter_in)
		userdata.recharge_counter_out = userdata.recharge_counter_in + 1
				
		# send the plan goal and control goal to the respective action server to move the robot to the recharging location
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			self._pos = self._check_rob_pos()
			try:
				if self._helper.action_for_change == nm.BATTERY_OK:
					return nm.BATTERY_OK
				if self._pos == nm.RECHARGING_ROOM:
					self._recharging_method()
					return nm.BATTERY_OK
				if self._pos != nm.RECHARGING_ROOM:
					self._helper.planner_client.cancel_goal()
					self._plan_and_go_to_recharge()
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
			
	def _check_rob_pos(self):
		"""
		Private function that checks the robot position with a query by the aRMOR client to the respective server.
		
		Args:
			none
			
		Returns:
			_location(String): actual location of the robot
		"""
		return self._helper.format(self._helper.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0]

	def _plan_and_go_to_recharge(self):
		"""
		Private function that is used to plan the path to the recharging location and move the robot to that position before recharging it.
		
		Args:
			none
			
		Returns:
			none
		"""
		_goal = self._helper.plan_location(nm.RECHARGING_ROOM)
		self._helper.planner_client.send_goal(_goal)
		log_msg = f'Planning the path to the recharging room \033[0;36;49m' + str(nm.RECHARGING_ROOM) + '\033[0m'
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))
		
		self._helper.planner_client.wait_for_result()
		self._helper.controller_client.send_goal(self._helper.planner_client.get_result())
		
		log_msg = f'Moving the robot to the recharging room \033[0;36;49m' + str(nm.RECHARGING_ROOM) + '\033[0m'
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))
		
		self._helper.controller_client.wait_for_result()
		
		self._helper.old_loc = self._pos
		
		self._helper.client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', str(self._helper.choice), str(self._helper.old_loc))
		
		self._helper.update_timestamp()
		self._helper.reason_changes()
		
		log_msg = f'Robot in recharging room \033[0;36;49m' + str(nm.RECHARGING_ROOM) + '\033[0m'
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))

	def _recharging_method(self):
		"""
		Private function that performs the recharging on the robot if and only if it is located in the recharging room.
		
		Args:
			none
			
		Returns:
			none
		"""
		log_msg = f'Robot in \033[0;36;49m' + str(nm.RECHARGING_ROOM) + '\033[0m is recharging'
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))
		
		_battery = []
		for i in range(100):
			if i % 5 == 0:
				_battery.append('#')
			# print red battery
			if i <= 20:
				print("[\033[1;31;49m" + ''.join(_battery) + "\033[0m] | Battery: \033[1;31;49m" + str(i) + "\033[0m %", end="\r")
			# print yellow battery
			elif 20 < i <= 70:
				print("[\033[1;33;49m" + ''.join(_battery) + "\033[0m] | Battery: \033[1;33;49m" + str(i) + "\033[0m %", end="\r")
			# print green battery
			elif i > 70:
				print("[\033[1;32;49m" + ''.join(_battery) + "\033[0m] | Battery: \033[1;32;49m" + str(i) + "\033[0m %", end="\r")
			rospy.sleep(0.05)
		
		self._helper.action_for_change = nm.BATTERY_OK
		self._helper.battery_timestamp = int(time.time())
		
