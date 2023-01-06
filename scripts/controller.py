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
import actionlib
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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

        # controller for movig the robot
        self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        self.goal_counter = 0
        self.feedback_counter = 0
        self.is_active = False

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
            
        # Check that the client did not cancel this service.
        if self._as.is_preempt_requested():
            rospy.loginfo(nm.tag_log('Service has been cancelled by the client!', LOG_TAG))
            # Actually cancel this service.
            self._as.set_preempted()
            return

        # Wait before to reach the following via point. This is just for testing purposes.
        self.move_to_goal(goal.via_points[9].x, goal.via_points[9].y)

        # Log current robot position.
        log_msg = f'Reaching point ({goal.via_points[9].x}, {goal.via_points[9].y}).'
        rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))

        # Publish a feedback to the client to simulate that a via point has been reached. 
        feedback.reached_point = goal.via_points[9]
        self._as.publish_feedback(feedback)

        while self.is_active is True and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # set the client not active and cancel the goal reached
        self.is_active = False
        self.move_client.cancel_goal()
            
        # Log current robot position.
        log_msg = f'Reached point ({goal.via_points[9].x}, {goal.via_points[9].y}).'
        rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(nm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.

    def active_callback(self):
        """
        Function that is executed when the communication starts.
        It simply increments a goal counter

        Args:
            none

        Returns:
            none
        """
        self.goal_counter += 1
        rospy.loginfo("Goal " + str(self.goal_counter))

    def feedback_callback(self, feedback):
        """
        Function that is executed when a feedback is received.
        It simply increments a feedback counter

        Args:
            feedback: feedack of the action server

        Returns:
            none
        """
        self.feedback_counter += 1
        if self.feedback_counter % 100 == 0:
            rospy.loginfo("Feedback " + str(self.feedback_counter))

    def done_callback(self, status, result):
        """
        Function executed when the communication ends.
        It checks the status of the action client and it prints a message relating to the status.

        Args:
            status: actual status of the action server
            result: result to return

        Returns:
            none
        """
        self.is_active = False   # The action client communication is not active.

        # Prints on the info window the status returned by the action server communication.
        if status == 2 or status == 8:
            rospy.loginfo("Goal received a cancel request.")
            return

        if status == 3:
            rospy.loginfo("Goal reached.")
            return

        if status == 4:
            rospy.loginfo("Goal aborted.")
            return

        if status == 5:
            rospy.loginfo("Goal rejected.")
            return

    def move_to_goal(self, x, y):
        """
        Function to move the robot to a specific goal set by the planner.
        The function takes the x and y coordinates of the goal and publishes through an action client to the server the coordinates the robot has to reach

        Args:
            x: x coordinate of the goal
            y: y coordinate of the goal

        Returns:
            none
        """
        self.is_active = True

        self.move_client.wait_for_server()

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        self.move_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        rospy.loginfo("Goal Published")

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(nm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()