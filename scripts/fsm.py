#!/usr/bin/env python

"""
.. module:: fsm
	:platform: ROS
	:synopsis: Script for the initialization of the Finite State Machine.

.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This script is used to define the structure of the finite state machine. Here it is initialized with all its states and their respective transitions.
For each state it is defined how to behave for each transition received so that the machine cannot be stuck or have errors in the changes.
At the beginning of the execution it is also instantiated a helper entity that is passed to each state as parameter to make it easier in some cases to use functions and shared variables. This entity is an attribute of the respective Class :mod:`helper`.
However, the main role of the helper is the sharing of the mutex that is used to access the shared variables without having troubles doing it. The mutex used is of course just one to try to have a perfect syncronization among the state and the reading/writing processes.
The finite state machine that has been created is composed of four main states: :mod:`ìnit_state`, :mod:`reasoner`, *move randomly*, which is in fact a sub machine, and :mod:`recharge`.
Of course these states include many different tasks so the decision of using a sub-machine allows us to have a more modular code and a more reactive program since the execution cores of the states are not so long.
The machine is divided as follows:

* initialization phase: responsible of initialising the ontology of the environment;
* reason state: responsible of reasoning all the changes happened and computing the new location the robot has to visit (according to some statements);
* move random state: divided into :mod:`plan_path_to_location` and :mod:`go_to_location_to_visit`:
	* The first is responsible of computing a path from the actual robot position to the target position computed by the reasoner;
	* the second is responsible of moving the robot through the points of the path just computed.
* recharge state: it is responsible of recharging the robot battery. The robot can be recharged if and only if it is in the correct recharging room so there are some steps to make the robot arrive there before being recharged.

Servers:
	:attr:`sis`: this is the name that the variable of the ROS server has in the program. It is necessary for the smach ROS state machine and it is responsible of the execution of each state and their transitions.
"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
from EXPROBLAB_Assignment2 import name_mapper as nm
from helper import Helper
from init_state import InitState
from reasoner import Reasoner
from recharge import Recharge
from plan_path_to_location import PlanPathToLocation
from go_to_location_to_visit import GoToLocationToVisit

def main():
	"""
	This function initializes the finite state machine node, called _smach_finite_state_machine_.
	It also creates the helper object to make it easier to access shared variables and use some functions.
	It is important to note that the SMACH machine needs the definition of all the outcomes needed to pass from one state to another: in this way if the execute of one state returned a transition that is not included, the machine does not return an error or does not remain stuck because not knowing what to do.
	The machine includes also one sub-machine, since we have four main states and one of them can be split into tow others to have a more modular architecture.
	"""
	rospy.init_node('smach_finite_state_machine')
	
	# Define an helper
	_helper = Helper()
	
	# Create a top level SMACH state machine
	sm = smach.StateMachine(outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED, nm.MOVE_RANDOM])
	# Create a lower level SMACH state machine
	sm_sub = smach.StateMachine(outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED, nm.MOVE_RANDOM])
	
	sm.userdata.sm_counter = 0
	sm_sub.userdata.sm_counter = 0

	# Open the container
	with sm:
		# Add states to the container
		# Initialization State
		smach.StateMachine.add(nm.INIT_STATE, InitState(_helper),
								transitions={nm.BATTERY_LOW: nm.INIT_STATE,
											 nm.BATTERY_OK: nm.INIT_STATE,
											 nm.LOADED_ONTOLOGY: nm.REASONER,
											 nm.REASONED: nm.INIT_STATE,
											 nm.PLANNED_PATH: nm.INIT_STATE,
											 nm.LOCATION_REACHED: nm.INIT_STATE,
											 nm.MOVE_RANDOM: nm.INIT_STATE},
								remapping={'init_state_counter_in':'sm_counter',
										   'init_state_counter_out':'sm_counter'})
		
		# Recharge State
		smach.StateMachine.add(nm.RECHARGE, Recharge(_helper),
								transitions={nm.BATTERY_LOW: nm.RECHARGE,
											 nm.BATTERY_OK: nm.REASONER,
											 nm.LOADED_ONTOLOGY: nm.RECHARGE,
											 nm.REASONED: nm.RECHARGE,
											 nm.PLANNED_PATH: nm.RECHARGE,
											 nm.LOCATION_REACHED: nm.RECHARGE,
											 nm.MOVE_RANDOM: nm.RECHARGE},
								remapping={'recharge_counter_in':'sm_counter',
										   'recharge_counter_out':'sm_counter'})
		
		# Reasoner State	   
		smach.StateMachine.add(nm.REASONER, Reasoner(_helper),
								transitions={nm.BATTERY_LOW: nm.RECHARGE,
											 nm.BATTERY_OK: nm.REASONER,
											 nm.LOADED_ONTOLOGY: nm.REASONER,
											 nm.REASONED: nm.MOVE_RANDOM,
											 nm.PLANNED_PATH: nm.REASONER,
											 nm.LOCATION_REACHED: nm.REASONER,
											 nm.MOVE_RANDOM: nm.REASONER},
								remapping={'reasoner_counter_in':'sm_counter',
										   'reasoner_counter_out':'sm_counter'})
		
		with sm_sub:
			# Add states to the container
			# Plan path to the location Stae
			smach.StateMachine.add(nm.PLAN_PATH_TO_LOCATION, PlanPathToLocation(_helper),
									transitions={nm.BATTERY_LOW: nm.BATTERY_LOW,
												 nm.BATTERY_OK: nm.PLAN_PATH_TO_LOCATION,
												 nm.LOADED_ONTOLOGY: nm.PLAN_PATH_TO_LOCATION,
												 nm.REASONED: nm.PLAN_PATH_TO_LOCATION,
												 nm.PLANNED_PATH: nm.GO_TO_LOCATION_TO_VISIT,
												 nm.LOCATION_REACHED: nm.PLAN_PATH_TO_LOCATION,
												 nm.MOVE_RANDOM: nm.PLAN_PATH_TO_LOCATION},
									remapping={'plan_path_to_location_counter_in':'sm_counter',
											   'plan_path_to_location_counter_out':'sm_counter'})
			
			# Go to the location State		
			smach.StateMachine.add(nm.GO_TO_LOCATION_TO_VISIT, GoToLocationToVisit(_helper),
									transitions={nm.BATTERY_LOW: nm.BATTERY_LOW,
												 nm.BATTERY_OK: nm.GO_TO_LOCATION_TO_VISIT,
												 nm.LOADED_ONTOLOGY: nm.GO_TO_LOCATION_TO_VISIT,
												 nm.REASONED: nm.GO_TO_LOCATION_TO_VISIT,
												 nm.PLANNED_PATH: nm.GO_TO_LOCATION_TO_VISIT,
												 nm.LOCATION_REACHED: nm.LOCATION_REACHED,
												 nm.MOVE_RANDOM: nm.LOCATION_REACHED},
									remapping={'go_to_location_to_visit_counter_in':'sm_counter',
											   'go_to_location_to_visit_counter_out':'sm_counter'})	
		
		# Move Random State including the plan to location and the go to location
		smach.StateMachine.add(nm.MOVE_RANDOM, sm_sub,
									transitions={nm.BATTERY_LOW: nm.RECHARGE,
												 nm.BATTERY_OK: nm.MOVE_RANDOM,
												 nm.LOADED_ONTOLOGY: nm.MOVE_RANDOM,
												 nm.REASONED: nm.MOVE_RANDOM,
												 nm.PLANNED_PATH: nm.MOVE_RANDOM,
												 nm.LOCATION_REACHED: nm.REASONER,
												 nm.MOVE_RANDOM: nm.MOVE_RANDOM})

	# Create and start the introspection server for visualization
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()

##########

if __name__ == '__main__':
	main()


