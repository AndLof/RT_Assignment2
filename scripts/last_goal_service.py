#! /usr/bin/env python

## @package assignment2_rt
# \file last_goal_service.py
# \brief Service node to return information about last goal
# \author Andrea Loforese
# \version 1.0
# \date 13/03/2025
#
# \details
#
# Subrscibes to: <BR>
# /reaching_goal/goal
#
# Service: <BR>
# last_goal
#
# Description: <BR>
# The node is executed with the launch file; it returns the coordinates of the last goal sent by the user whenever the command "rosservice call /last_goal" is executed in the terminal.

import rospy
import actionlib
from assignment_2_2024.msg import PlanningActionGoal #PlanningGoal, PlanningActionResult
from assignment2_rt.srv import Last_goal, Last_goalResponse
from geometry_msgs.msg import PoseStamped

## @brief Stores the last received goal position.
last_goal = None

## @brief Callback function for the /reaching_goal/goal topic.
#  @param msg Message containing the latest goal position.
def goal_callback(msg):
	global last_goal
	last_goal = msg.goal.target_pose
	#rospy.loginfo(f"Coordinates of last goal: {last_goal}")

## @brief Service callback function to return the last goal coordinates.
#  @param req Service request (unused in this case).
#  @return Last_goalResponse containing the last goal coordinates.
def service_function(req):
	global last_goal
	#rospy.loginfo("Coordinates of last goal:")
	return Last_goalResponse(last_goal)

## @brief Main function to initialize the service node and subscribers.
def main():
	rospy.init_node('last_goal_service')

	rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)

	service = rospy.Service('last_goal', Last_goal, service_function)

	rospy.spin()

## @brief Entry point of the script.
if __name__ == '__main__':
	main()

