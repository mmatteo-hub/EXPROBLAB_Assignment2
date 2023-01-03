#!/usr/bin/env python

"""
.. module:: name_mapper
    :platform: ROS
    :synopsis: Class for the Name Mapper module
    
.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This scripts is used in the package to define all the names of the variables used. It is a way to use a variable instead of the total name thus avoiding spelling problems while writing it.
It is divided as follows:

* name of the state if the Finite State Machine and of the Sub Finite State Machine;
* name of all the transition used: there are also some parameters that are not properly transition but they are used to modify the value of the shared variable of the helper to guarantee the correct flow in the program;
* name of the action, planner and controller, used in the program;
* parameters for the robot behaviour:
    * battery threshold: value after which the robot has to be recharged;
    * busy paramter: value to define the busy waiting that the robot has to perform before starting the plan to another location;
    * number of points: value that define the number of points the planner module has to compute for each path required;
    * recharging room: string name of the recharging room. It is needed in particular when the robot has to check the availability of this particualr location when it has to go to the recharge state.
* rooms: name of all locations in the ontology;
* coordinates: x and y coordinate of each location. Here it is needed a  match one to one with the names above.

"""

import rospy
# File needed to define some variables that are used multiple times in the program.
# In this way thei can be accessed as a field of this file, thus avoiding error in strings writing.

##########
# Definition of the states name of the Finite State Machine and its sub-machine

INIT_STATE = 'INIT_STATE'
RECHARGE = 'RECHARGE'
REASONER = 'REASONER'
PLAN_PATH_TO_LOCATION = 'PLAN_PATH_TO_LOCATION'
GO_TO_LOCATION_TO_VISIT = 'GO_TO_LOCATION_TO_VISIT'
MOVE_RANDOM = 'MOVE_RANDOM'

##########
# Definition of the action needed to change state or to check the shared vriable for adopting the FSM behaviour

BATTERY_OK = 'BATTERY_OK'
BATTERY_LOW = 'BATTERY_LOW'
LOADED_ONTOLOGY = 'LOADED_ONTOLOGY'
REASONED = 'REASONED'
PLANNED_PATH = 'PLANNED_PATH'
LOCATION_REACHED = 'LOCATION_REACHED'
RECHARGING_CHECK = 'RECHARGING_CHECK'

##########
# Name of the Action Servers and Clients used in the program 

ACTION_PLANNER = 'motion/planner'
ACTION_CONTROLLER = 'motion/controller'
NODE_PLANNER = 'planner'
NODE_CONTROLLER = 'controller'
NODE_ROBOT_STATE = 'robot-state'

##########
# Parameter for the battery threshold (when it gets low) in seconds
BATTERY_THRESHOLD = 60 #[s]
# Parameter to simulate the busy waiting of the robot in a specific location (in seconds)
BUSY_PARAMETER = 5 #[s]
# Parameter to know the number of points needed in the path
NUMBER_OF_POINTS_PATH = 10
# Parameter to indicate the name of the recharging room
RECHARGING_ROOM = 'E'

##########
# List of all the locations in the onotlogy
ROOMS = ['E', 'C1', 'C2', 'R1', 'R2', 'R3', 'R4']
# List of all the coordinates of the locations in the ontology
# IMPORTANT: Follow a one to one relation between the name and the coordinates
COORDINATES = [[0.0,0.0], [2.0,0.0], [3.0,0.0], [1.0,0.0], [1.0,1.0], [4.0,0.0], [4.0,1.0]]

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    """
    This function is used to generate the log tag that is shown in the FSM while running.

        Args:
            msg(String): message to be visualized
            producer_tag(String): name of the state that has to send the log message

        Returns:
            log_msg(String): message for the log
    """
    return '@%s>> %s' % (producer_tag, msg)
