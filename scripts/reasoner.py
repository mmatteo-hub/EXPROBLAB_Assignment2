#!/usr/bin/env python

"""
.. module:: reasoner
	:platform: ROS
	:synopsis: Class for Reason state of the finite state machine
	
.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class is used to reason the changes in the program execution.
The changes included are:

* actual robot position in the environment built;
* the choice of the next reachable location the robot will have to reach.

All these are computed thanks to the use of the Helper object and few private functions.
The important thing the reasoner has to take into account is the hierarchy among the locations that it has to choose among, in particular:

* the robot should stay mainly in the corridors;
* as soon as a room becomes urgent the robot has to visit it, if it is reachable by the actual robot position;
* the robot has to go to recharge itself in the proper recharging room as soon as the battery is low and the robot can reach that specific location.

For this purpose the recharging room is set as the most urgent among the other as soon as the battery is low so that it has the priority for the robot as soon as it is reachable by it.
The client is taken from the helper object.

Clients:
	:attr:`client`: aRMOR client to make the query request to the respective server to take the infos from the actual situation of the robot and the location in the environment
"""

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import random
from EXPROBLAB_Assignment2 import name_mapper as nm

class Reasoner(smach.State):
	def __init__(self, helper):
		"""
		This function is used to initialize the Reasoner state for the finite state machine.
		It is used also the helper object imported from the Helper class to use the shared variables and the mutex.
		
		Args:
			helper(Helper): object that helps the sharing of information relating the shared variables and the use of mutex among the states of the machine
		
		Returns:
			none
		"""
		self._helper = helper
				
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED, nm.MOVE_RANDOM],
								input_keys=['reasoner_counter_in'],
								output_keys=['reasoner_counter_out'])
		
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
		rospy.loginfo('Executing state ' + nm.REASONER + ' (users = %f)'%userdata.reasoner_counter_in)
		userdata.reasoner_counter_out = userdata.reasoner_counter_in + 1
		self._helper.reason_changes()
		self._helper.check_battery()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.action_for_change == nm.BATTERY_LOW:
					self._helper.choice = self._check_recharge_location_available()
					if self._helper.choice != [] or self._helper.format(self._helper.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0] == nm.RECHARGING_ROOM:
						return nm.BATTERY_LOW
					else: self._helper.action_for_change = nm.RECHARGING_CHECK
						
				if self._helper.action_for_change == nm.LOADED_ONTOLOGY or self._helper.action_for_change == nm.LOCATION_REACHED or self._helper.action_for_change == nm.BATTERY_OK or self._helper.action_for_change == nm.RECHARGING_CHECK:
					self._helper.choice = self._check_accessible_location()
					self._helper.old_loc = self._helper.format(self._helper.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0]
					return nm.REASONED
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)

	def _check_accessible_location(self):
		"""
		Private function that checks for the reachable locations of the robot when it is in a certain location.
		Its main functionality is to return the list of reachable location that are needed: in particular it has to follow the hierarchy of the program:
		
		* urgent location (if any);
		* corridors (if there are not any urgent and if there are any corridors reachable);
		* general location (if none of the previous returned something)
		
		The choice is computed randomly among the list that are found and returned.
		
		Args:
			none
		
		Returns:
			location(List): a list of string representing the amount of reachable location that can visit accordin to the hierarchy.
		"""
		# query to find the entire group of total locations reachable
		_reachable_locations = self._helper.format(self._helper.client.query.objectprop_b2_ind('canReach','Robot1'), '#', '>')
		# function to return only the corridors reachable
		_reachable_corridors = self._check_for_corridors(_reachable_locations)
		# function to return only the urgent locations reachable
		_reachable_urgent = self._check_for_urgent_locations(_reachable_locations)
		
		# return to switch among the different lists to return the most prioritized one
		if _reachable_urgent != []:
			return self._choose_destination(_reachable_urgent)
		elif _reachable_corridors != []:
			return self._choose_destination(_reachable_corridors)
		elif _reachable_locations != []:
			return self._choose_destination(_reachable_locations)
		else:
			log_msg = f'No locations reachable from {str(self._helper.choice)} '
			rospy.loginfo(nm.tag_log(log_msg, nm.REASONER))
			
	def _check_recharge_location_available(self):
		"""
		Private funtion to check if the recharging room is available from the actual robot position.
		This is a function used when the battery is low to see if the robot can directly move to the recharging room or has to move to another location before reaching it.
		If it can reach then it is returned the name of the location otherwise an empty list.
		
		Args:
			none
			
		Returns:
			name(String): name of the recharging room
		"""
		_reachable_E_room = self._check_for_recharging_room(self._helper.format(self._helper.client.query.objectprop_b2_ind('canReach','Robot1'), '#', '>'))
		if _reachable_E_room == nm.RECHARGING_ROOM:
			return nm.RECHARGING_ROOM
		else: return []
			
	def _check_for_corridors(self, _reachable_locations):
		"""
		Private function to check if there are any corridors that can be reached by the robot from the actual position.
		The group of all the corridors is taken by a query to the aRMOR server by the client.
		However this list includes all the corridors in the ontology so they have to be sorted according to the locations reachable by the robot, which is the parameter passed to the function.
		
		Args:
			_reachable_locations(List): list of all the reachable locations by the robot from its actual position
			
		Returns:
			_reachable_corridors(List): list of all the reachable corridors by the robot from its actual position
		"""
		_corridors = self._helper.format(self._helper.client.query.ind_b2_class('CORRIDOR'), '#', '>')
		_reachable_corridors = []
		for i in range(len(_corridors)):
			if(_corridors[i] in _reachable_locations):
				_reachable_corridors.append(_corridors[i])
				
		return _reachable_corridors
		
	def _check_for_urgent_locations(self, _reachable_locations):
		"""
		Private function to check if there are any urgent locations that can be reached by the robot from the actual position.
		The group of all the urgent locations is taken by a query to the aRMOR server by the client.
		However this list includes all the locations that are currently urgent in the ontology so they have to be sorted according to the locations reachable by the robot, which is the parameter passed to the function.
		
		Args:
			_reachable_locations(List): list of all the reachable locations by the robot from its actual position
			
		Returns:
			_reachable_urgent(List): list of all the urgent locations reachable by the robot from its actual position
		"""
		_urgent_locations = self._helper.format(self._helper.client.query.ind_b2_class('URGENT'), '#', '>')
		_reachable_urgent = []
		for i in range(len(_urgent_locations)):
			if(_urgent_locations[i] in _reachable_locations):
				_reachable_urgent.append(_urgent_locations[i])
				
		return _reachable_urgent
		
	def _check_for_recharging_room(self, _reachable_locations):
		"""
		Private function to check the rechability of the recharging room from the actual position of the robot.
		The list of all the reachable locations is passed as parameter and the function checks if the recharging room is present or not in the list.
		If yes it returns the name of this location.
		
		Args:
			_reachable_locations(List): list of all the reachable locations by the robot from its actual position
			
		Returns:
			recharging_room(String): name of the recharging room
		"""
		for i in range(len(_reachable_locations)):
			if _reachable_locations[i] == nm.RECHARGING_ROOM:
				return nm.RECHARGING_ROOM
			
	def _choose_destination(self, _locations):
		"""
		Private function to choose the actual location the robot has to reach.
		The location is chosen randomly from a list of rechable locations passed to the function.
		
		Args:
			_locations(List): list of all the reachable locations by the robot from its actual position
			
		Returns:
			_choice(String): name of the location the robot has to reach
		"""
		return random.choice(_locations)
