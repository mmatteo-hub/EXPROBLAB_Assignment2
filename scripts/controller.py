#!/usr/bin/env python

"""
.. module:: controller
	:platform: ROS
	:synopsis: Class for the Controller server to compute the path

.. moduleauthor:: Matteo Maragliano 4636216@studenti.unitge.it

This class is the server used by the FSM to simulate the movement of the robot from a starting position to a target one.	
The path to follow is passed by the client. The controller starts as soon as the planner ends computing the path.
The server simulates the movement of the robot and then publishes the result. In case the process is interrupted due to some signals (a battery low for example), then it returns nothing because of the preemption.

Servers:
	:attr:`motion/controller`: server used to simulate the movement of the robot.

"""

import random
import rospy
# Import constant name defined to structure the architecture.
from EXPROBLAB_Assignment2 import name_mapper as nm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from EXPROBLAB_Assignment2.msg import ControlFeedback, ControlResult
from EXPROBLAB_Assignment2.srv import SetPose
import EXPROBLAB_Assignment2  # This is required to pass the ControlAction` type for instantiating the `SimpleActionServer`.
from helper import Helper
from armor_api.armor_client import ArmorClient

# A tag for identifying logs producer.
LOG_TAG = nm.NODE_CONTROLLER


# An action server to simulate motion controlling.
class ControllingAction(object):

    def __init__(self):
        """
        This function is used to initialize the Controller server

        Args:
            none

        Returns:
            none
        """
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(nm.ACTION_CONTROLLER,
                                        EXPROBLAB_Assignment2.msg.ControlAction,
                                        execute_cb=self.execute_callback,
                                        auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{nm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a fixed delay ' 
                   f'between each via point spanning in [{nm.NUMBER_OF_POINTS_PATH}).')
        rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))

    def execute_callback(self, goal):
        """
        Function that is executed every time the machine needs to simulate the movement of the robot between two locations.
        The callback is invoked when a client sets a goal to the :mod:`controller` server.
        This function requires a list of via points (the plan), and it simulates a movement through each point with a fixed delay.

        Args:
            none

        Returns:
            none
        """
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(nm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(nm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(nm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.

			# Wait to simulate movement.
            delay = 0.25
            rospy.sleep(delay)
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            
            # Log current robot position.
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(nm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(nm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
