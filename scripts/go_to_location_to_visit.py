#!/usr/bin/env python

"""
.. module:: go_to_location_to_visit
	:platform: ROS
	:synopsis: Class for the GoToLocationToVisit state of the sub-finite state machine
	
.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class is a state of the sub-state machine *move randomly*. This class has the aim to move the robot through the path computed in the previous state.
The path is taken from the :mod:`planner` client by taking its result provided by the respsctive server and then it is passed to the :mod:`controller` ones. This uses the :mod:`controller` to make the robot move through the points of the path.
This state is a little trivial since it is used both to move the robot from a location to another both to drive it to the recharging room when necessary and when possible.
Due to this as soon as the robot has a location that has to reach, this location is reached and then, depending of the variable shared by the Helper, it can be retrieved if the move was just a random movement or it was needed to reach a location from which check again the possibility to reach the recharging room.
The clients are taken from the helper object.

Clients:
	:attr:`client`: aRMOR client used to update the timestamp of the robot and of the location the robot has just visited
	
	:attr:`planner_client`: client from which it is taken the goal, which means the path just computed to be passed to the controller.
	
	:attr:`controller_client`: client that makes the request to the respective server to move the robot through the path from the starting to the end position.
"""

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import time
from actionlib_msgs.msg import GoalStatus
from EXPROBLAB_Assignment2 import name_mapper as nm

class GoToLocationToVisit(smach.State):
	def __init__(self, helper):
		"""
		Function used to initialize the state of the sub-machine.
		In this step there is also the declaration of all the outcomes that the state can have.
		
		Args:
			helper(Helper): helper object that allows the user to use shared elements among the scripts.
			
		Returns:
			none
		"""
		self._helper = helper
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED, nm.MOVE_RANDOM],
								input_keys=['go_to_location_to_visit_counter_in'],
								output_keys=['go_to_location_to_visit_counter_out'])
		
	def execute(self, userdata):
		"""
		Function that is executed every time the machine enters the state.
		It is responsible of returning a state transitioning to change the state.
		It uses the mutex instantiated in the helper to manage the variable access.
		
		Args:
			userdata: pointer to pass the data among the states of a State Machine
			
		Returns:
			transition(String): string containing the label of the action performed and used to change state in the sub machine and returning to the upper one.
		"""
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.GO_TO_LOCATION_TO_VISIT + ' (users = %f)'%userdata.go_to_location_to_visit_counter_in)
		userdata.go_to_location_to_visit_counter_out = userdata.go_to_location_to_visit_counter_in + 1
		
		# send the goal to the controller client
		self._helper.controller_client.send_goal(self._helper.planner_client.get_result())
		# waiting the client to finish the computation
		self._helper.controller_client.wait_for_result()
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.action_for_change == nm.BATTERY_LOW:
					self._helper.controller_client.cancel_goal()
					return nm.BATTERY_LOW
				if self._helper.planner_client.get_state() == GoalStatus.SUCCEEDED:
					self._helper.client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', str(self._helper.choice), str(self._helper.old_loc))
					
					log_msg = f'Moved the robot from \033[0;35;49m' + str(self._helper.old_loc) + '\033[0m to \033[0;36;49m' + str(self._helper.choice) + '\033[0m'
					rospy.loginfo(nm.tag_log(log_msg, nm.GO_TO_LOCATION_TO_VISIT))
					
					self._helper.update_timestamp()
					
					if(self._helper.controller_client.get_state() == GoalStatus.SUCCEEDED):
						log_msg = f'Doing something in the location \033[0;36;49m' + str(self._helper.choice) + '\033[0m'
						rospy.loginfo(nm.tag_log(log_msg, nm.GO_TO_LOCATION_TO_VISIT))
						rospy.sleep(nm.BUSY_PARAMETER)
					
					# if the reasoner is checking for the recharging room but it cannot reach it directly because in a location no directly connected to the recharging one,
					# then it move the robot into another location and try to reach the recharging room from this new one
					if self._helper.action_for_change == nm.RECHARGING_CHECK:
						self._helper.action_for_change = nm.BATTERY_LOW
					else:
						self._helper.action_for_change = nm.LOCATION_REACHED
						return nm.LOCATION_REACHED
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
