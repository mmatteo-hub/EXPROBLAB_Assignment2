#!/usr/bin/env python

"""
.. module:: helper
	:platform: ROS
	:synopsis: Class for help functions

.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class implements an helper member that can be used in the program it is included into to simplify the code.
In particular this helper provides all the action clients used and needed to control the robot plus other functions used to retrieve information from the data and queries acquired.
It is a way to avoid the use of many global variables that could lead to some problems in the code and it also allows an easier re-use of the code.

Clients:
	:attr:`armor_client`: client to communicate with the aRMOR server. It provides the possibility to create, load, modify and save an ontology. Moreover, it is used to retrieve information about the robot position and its possibility to reach a particular location.
	
	:attr:`motion_planner`: client to communicate with the planner server. This client is used to give the sever a start an a target position and the server provides a plan of points that the robot will have to follow.
	
	:attr:`motion_controller`: client to communicate with the controller server. The client is responsible of moving the robot, so it has to give the robot the planner client points and makes it follow them to reach the target position.
"""

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import random
import time
import re
import numpy
from std_msgs.msg import Int32,Int32MultiArray
from EXPROBLAB_Assignment2 import name_mapper as nm
from EXPROBLAB_Assignment2.msg import Point, PlanAction, PlanGoal, ControlAction
from EXPROBLAB_Assignment2.srv import RoomInformation
from armor_api.armor_client import ArmorClient

from threading import Thread
from threading import Lock

class Helper:
	def __init__(self, done_callback = None, feedback_callback = None, mutex = None):
		"""
		Function to inizialize the helper.
		It also initializes the mutex that will be used to share the common resources in order to avoid multiple accesses and having errors while reading/writing values.
		
		The initalization includes of course also the clients used in the program.
		
		Args:
			none
		
		Returns:
			none
		"""
		if mutex is None:
			self.mutex = Lock()
		else:
			self.mutex = mutex
		self.action_for_change = ''
		self.robot_timestamp_value = -1

		# marker array
		self.markerArr = []
		
		# client for the arMOR server
		self.client = ArmorClient('armor_client', "reference")
			
		# client for the planner server
		self.planner_client = actionlib.SimpleActionClient('motion/planner', PlanAction)
		
		# client for the controller server
		self.controller_client = actionlib.SimpleActionClient('motion/controller', ControlAction)

		# subscriber for taking the list
		rospy.Subscriber("/list_topic", Int32MultiArray, self.listCallback)

		# client for the marker server
		self.marker_client = rospy.ServiceProxy('/room_info', RoomInformation)

	def listCallback(self, msg):
		self.markerArr = msg.data
		
	def reason_changes(self):
		"""
		Function to apply the modifications to the ontology.
		It uses the aRMOR client to do this and it is called every time the program changes something related to the ontology entities (timestamps, robot position etc ...)
		
		Args:
			none
		
		Returns:
			none
		"""
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		
	def check_battery(self):
		"""
		Function to check the threshold to rise the battery low process.
		This function checks the difference between the actual time and the last time the battery has been recharged and if it has passed too much time it changes the value of the shared variable.
		This changes is read by the states of the finite state machine and so the SMACH machine can adapt its behaviour to deal with this event.
		
		Args:
			none
		
		Returns:
			none
		"""
		if (int(time.time()) - self.battery_timestamp) > nm.BATTERY_THRESHOLD:
			self.mutex.acquire()
			self.action_for_change = nm.BATTERY_LOW
			self.mutex.release()
		
	def format(self, oldlist, start, end):
		"""
		Function to format a list of strings.
		For all the elements in the list, it takes two character, a start and a finish one. In each element i-th it is looked for these two characters and it is taken just the portion of the string between them.
		The return is a new list with the elements "cleaned" by not useful characters.
		
		Args:
			oldlist(List): the list that has to be re-written in a better way
			start(Char): the start character for the list member cut
			end(Char): the end characted for the list memeber cut
		
		Returns:
			newList(List): the new list with all the elements written in a proper way
		"""
		newlist = []
		for string in oldlist:
			newlist.append(re.search(start + '(.+?)' + end, string).group(1))
		return newlist

	def plan_location(self, location):
		"""
		It returns the client goal formatted as the start and target position.
		It uses some private methods to compute the actual robot position and the target goal for the robot.
		
		Args:
			location(String): location to which the robot has to arrive
		
		Returns:
			goal(PlanGoal): the goal composed as two Points (start and target both with x and y coordinates)
		"""
		goal = PlanGoal()
		goal.start = self._robot_pos()
		goal.target = self._target_coordiantes(location)
		return goal

	def _robot_pos(self):
		"""
		Private function that checks and returns the actual robot position thanks to a query to the aRMOR server.
		The function returns a type Point with its x and y parameters.
		
		Args:
			none
		
		Returns:
			position(Point): actual position of the robot casted as a Point with x and y coordinates
		"""
		_pos = self.format(self.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0]
		for i in range(len(nm.ROOMS)):
			if _pos == nm.ROOMS[i]:
				return Point(x = nm.COORDINATES[i][0], y = nm.COORDINATES[i][1])
				
	def _target_coordiantes(self, _location):
		"""
		Function to retrieve the room coordinates (x and y) from its name.
		The function works with the help of the :mod:`name_mapper` file that stores both the name and the corresponding coordinates for each location in the ontology.
		In this file there are two list with a one to one correspondence between the name and the coordinates location.
		
		Args:
			_location(String): string containing the name of the location
		
		Returns:
			coordinates(Point): actual coordiantes of the location casted as a Point (with x and y)
		"""
		for i in range(len(nm.ROOMS)):
			if _location == nm.ROOMS[i]:
				return Point(x = nm.COORDINATES[i][0], y = nm.COORDINATES[i][1])

	def _robot_timestamp_value(self):
		"""
		Function to retrieve the timestamp of the robot.
		It is used the aRMOR server with a query and the timestamp is returned as output.
		
		Args:
			none
			
		Returns:
			_timestamp(String): timestamp of the robot casted as a string
		"""
		_timestamp = self.client.query.dataprop_b2_ind('now', 'Robot1')
		return str(self.format(_timestamp, '"', '"')[0])
				
	def _location_old_timestamp(self, _location):
		"""
		Function to retrieve the timestamp of the lcoation.
		It is used the aRMOR server with a query and the timestamp is returned as output.
		The output represents the last time the location was visited by the robot.
		
		Args:
			_location(String): name of the location the timestamp is required for
			
		Returns:
			_timestamp(String): timestamp of the location casted as a string
		"""
		_timestamp = self.client.query.dataprop_b2_ind('visitedAt', _location)
		return str(self.format(_timestamp, '"', '"')[0])

	def update_timestamp(self):
		"""
		Function to update the timestamp of an entity in the ontology.
		It is replaced the timestamp of the robot with the acutal time.
		This timestamp is then used to replace the location timestamp to set where the robot is at the actual time (the actual robot timestamp is now taken from the private method since it has just been updated)
		
		Args:
			none
			
		Returns:
			none
		"""
		self.client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), self._robot_timestamp_value())
		self.client.manipulation.replace_dataprop_b2_ind('visitedAt', str(self.choice), 'Long', self._robot_timestamp_value(), self._location_old_timestamp(str(self.choice)))
					
