#!/usr/bin/env python

"""
.. module:: init_state
	:platform: ROS
	:synopsis: Class for the InitState state of the finite state machine

.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class represents the initial state of the state machine. It is the first state the program is into and it is not executed any longer during its entire execution.
It is responsible of instantiating a type helper to use the useful functions there provided. Then it use a private function to start modifying the ontology given as reference.
It adds all the parameters to the locations, doors, so that the reasoner will be able to know which one communicates with; it later sets that all the elements are different so that there cannot be ambiguities.
At the end of this process it is also retrieve the actual time in the execution and it is add among the properties of the entities so that it can be modified later when necessary.
The execution ends with a return that allows the main program to pass to the next state of the finite state machine.
The steps are computed thanks to the use of the aRMOR client that provides query to the respective server to modify and use the parameters.
The client is taken from the helper object.

Clients:
	:attr:`client`: aRMOR client used from the helper entity to send request to the respective server

"""

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import time
import numpy
from os.path import dirname, realpath
from EXPROBLAB_Assignment2 import name_mapper as nm

from armor_api.armor_client import ArmorClient

class InitState(smach.State):
	def __init__(self, helper):
		"""
		Function used to initialize the state of the machine.
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
								input_keys=['init_state_counter_in'],
								output_keys=['init_state_counter_out'])
								
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
		rospy.loginfo('Executing state ' + nm.INIT_STATE + ' (users = %f)'%userdata.init_state_counter_in)
		userdata.init_state_counter_out = userdata.init_state_counter_in + 1
		# initialize the battery timestamp with the current time value
		self._helper.battery_timestamp = int(time.time())
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				self._ontology_initialization()
				#self._helper.action_for_change = nm.LOADED_ONTOLOGY
				#return nm.LOADED_ONTOLOGY
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
			
	def _ontology_initialization(self):
		"""
		Function used to store all the request to the aRMOR server, through the client, to modifiy the onotlogy.
		In particular it uses a pre-built ontology that is stored in the project folder and it modifies it by adding entities and properties.
		It adds entities, it adds them properties, doors and it adds the timestamp.
		When it ends it returns to the execute function and it changes state.
		
		(There is also the possibility to save the ontology in another .owl file, and this can be done by un-commenting the last line of code of this script)
		
		Args:
			none
			
		Returns:
			none
		"""
		if self._helper.markerArr:
			self._server_request(self._helper.markerArr)

	def _server_request(self, lst):
		for el in lst:
			res = self._helper.marker_client(el)
			print(res)
