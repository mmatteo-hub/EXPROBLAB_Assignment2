#! /usr/bin/env python

"""
.. module:: planner
	:platform: ROS
	:synopsis: Class for the Planner server to compute the path
	
.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class is the server used by the FSM to compute the path for the robot from a starting position to a target one.	
Each position in the environment used is associated to a point coordinate [float x, float y] according to the list in the :mod:`name_mapper` file.
The plan is computed as a linear space on 'n' points between the two coordinates (the number of points is set in the same file as before).
The server computes the path and then publishes the result. In case the process is interrupted due to some signals (a battery low for example), then it returns nothing because of the preemption.

Servers:
	:attr:`motion/planner`: server used to plan the path between the two position passed as goal input to the server.

"""

import random
import rospy
import numpy as np
# Import constant name defined to structure the architecture.
from EXPROBLAB_Assignment2 import name_mapper as nm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from EXPROBLAB_Assignment2.msg import Point, PlanFeedback, PlanResult
import EXPROBLAB_Assignment2  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
from helper import Helper
from armor_api.armor_client import ArmorClient


# A tag for identifying logs producer.
LOG_TAG = nm.NODE_PLANNER


# An action server to simulate motion planning.
class PlaningAction(object):
    def __init__(self):
        """
        This function is used to initialize the Planner server

        Args:
            none

        Returns:
            none
        """
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(nm.ACTION_PLANNER, 
                                        EXPROBLAB_Assignment1.msg.PlanAction, 
                                        execute_cb=self.execute_callback, 
                                        auto_start=False)

        self._as.start()

        # Log information.
        log_msg = (f'`{nm.ACTION_PLANNER}` Action Server initialised. It will create random path with a number of point '
                   f'[{nm.NUMBER_OF_POINTS_PATH}). Each point will be generated with a fixed delay.')
      
        rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))
    def execute_callback(self, goal):
        """
        Function that is executed every time the machine needs to compute a plan from two locations.
        The callback invoked when a client set a goal to the :mod:`planner` server.
        This function will return a list of points (the plan) where the fist point is the current robot position (passed as goal.start parameter), while the last point is the *target* position (passed as :attr:`goal.target` parameter).

        Args:
            none
            
        Returns:
            none
        """
        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions from the client goal fields.
        start_point = goal.start
        target_point = goal.target

        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)

        # Log message to publish the number of points in the plan
        log_msg = f'Server is planning {nm.NUMBER_OF_POINTS_PATH} points...'
        rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))
        
        # computing the linspace for the x and y coordinates
        x = np.linspace(start_point.x, target_point.x, nm.NUMBER_OF_POINTS_PATH)
        y = np.linspace(start_point.y, target_point.y, nm.NUMBER_OF_POINTS_PATH)

        # Generate the points of the plan.
        for i in range(1, nm.NUMBER_OF_POINTS_PATH):
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(nm.tag_log('Server has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()  
                return
            # Generate a new random point of the plan.
            new_point = Point()
            new_point.x = x[i]
            new_point.y = y[i]
            feedback.via_points.append(new_point)

            # Wait to simulate computation.
            delay = 0.25
            rospy.sleep(delay)
            
            # Publish the new random point as feedback to the client.
            self._as.publish_feedback(feedback)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points
        log_msg = 'Motion plan succeeded with plan: '
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(nm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()
