#!/usr/bin/env python

"""
.. module:: plan_path_to_location
	:platform: ROS
	:synopsis: Class for the PlanPathToLocation state of the sub-finite state machine

.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class is a state of the sub-state machine *move random*. This class aim to compute a plan of *n* points from the actual position of the robot to the target location given by the reasoner.
The number of points is a parameter specified in the :mod:`name_mapper` file.
The class calls the :func:`plan_location` function from the helper object, previously initialized, which uses the :mod:`planner` server to compute the path.
As soon as the server gives the client the response with the result of the path the sub-machine passes to the next state.
The client is taken from the helper object.

Clients:
	:attr:`planner_client`: client that makes the request to the respective server to take the path from the starting to the end position for the robot
"""

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import random
from EXPROBLAB_Assignment2.msg import Point, PlanAction, PlanGoal, ControlAction
from actionlib_msgs.msg import GoalStatus
from EXPROBLAB_Assignment2 import name_mapper as nm

class PlanPathToLocation(smach.State):
	def __init__(self, helper):
		"""
		The function is used to initialize the state of the sub-machine.
		In this step, there is also the declaration of all the outcomes that the state can have.
		
		Args:
			helper(Helper): helper object that allows the user to use shared elements among the scripts.
			
		Returns:
			none
		"""
		self._helper = helper
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED, nm.MOVE_RANDOM],
								input_keys=['plan_path_to_location_counter_in'],
								output_keys=['plan_path_to_location_counter_out'])
		
	def execute(self, userdata):
		"""
		The function is executed every time the machine enters the state.
		It is responsible for returning a state and transitioning to change the state.
		It uses the mutex instantiated in the helper to manage the variable access.
		
		Args:
			userdata: pointer to pass the data among the states of a State Machine
			
		Returns:
			transition(String): string containing the label of the action performed and used to change state in the sub-machine.
		"""
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.PLAN_PATH_TO_LOCATION + ' (users = %f)'%userdata.plan_path_to_location_counter_in)
		userdata.plan_path_to_location_counter_out = userdata.plan_path_to_location_counter_in + 1
		# take the _goal position the robot has to reach
		_goal = self._helper.plan_location(self._helper.choice)
		# gives the client the goal to allow the server computing the path
		self._helper.planner_client.send_goal(_goal)
		# waiting the client to finish the computation
		self._helper.planner_client.wait_for_result()
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.action_for_change == nm.BATTERY_LOW:
					self._helper.planner_client.cancel_goal()
					return nm.BATTERY_LOW
				if self._helper.planner_client.get_state() == GoalStatus.SUCCEEDED:
					self._helper.action_for_change = nm.PLANNED_PATH  
					return nm.PLANNED_PATH
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
