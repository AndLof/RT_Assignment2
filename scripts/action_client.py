#! /usr/bin/env python

## @package assignment2_rt
# \file action_client.py
# \brief Node for Action Client: user interface
# \author Andrea Loforese
# \version 1.0
# \date 13/03/2025
#
# \details
#
# Subrscibes to: <BR>
# /odom <BR>
# /reaching_goal/result
#
# Publish to: <BR>
# /custom_info
#
# Description: <BR>
# This node consists to an action client that allows the user to control the robot into the simulation environment, providing a simple interface to set the desired goal position or to cancel it;
# besides, it publishes real time informations about robot's velocity and position. Whenever a goal position is reached or canceled, a message is displayed on the terminal.

import rospy
import actionlib
from actionlib import GoalStatus
from nav_msgs.msg import Odometry
from assignment2_rt.msg import Custom_info
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningActionResult


## 
# \brief Callback function to handle odometry messages and publish robot state.
# \param msg Odometry message containing the robot's current state.
def odom_callback(msg):
    global pub
    custom_msg = Custom_info()
    custom_msg.x = msg.pose.pose.position.x
    custom_msg.y = msg.pose.pose.position.y
    custom_msg.vel_x = msg.twist.twist.linear.x
    custom_msg.vel_z = msg.twist.twist.angular.z
    pub.publish(custom_msg)


##
# \brief Callback function to handle goal result messages and display the goal state.
# \param msg PlanningActionResult message containing the goal status.
def result_callback(msg):
	global current_result
	global converted_result

	current_result = msg.status.status
	
	if current_result == 0:
		converted_result = "PENDING"
	elif current_result == 1:
		converted_result = "ACTIVE"
	elif current_result == 2:
		converted_result = "PREEMPTED"
	elif current_result == 3:
		converted_result = "SUCCEEDED"
	elif current_result == 4:
		converted_result = "ABORTED"
	elif current_result == 5:
		converted_result = "REJECTED"
	elif current_result == 6:
		converted_result = "PREEMPTING"
	elif current_result == 7:
		converted_result = "RECALLING"
	elif current_result == 8:
		converted_result = "RECALLED"
	elif current_result == 9:
		converted_result = "LOST"
	
	rospy.loginfo(f"State of goal: {current_result}" + ":" + "%s", converted_result + ".Please continue writing g or c:")

##
# \brief Main function to initialize the node, handle user input, and send/cancel goals.
def main():
    global pub
    rospy.init_node('action_client')

    # Publish on topic custom_info
    pub = rospy.Publisher('/custom_info', Custom_info, queue_size=10)
    # Subscribe to topics /odom and /reaching_goal/result
    rospy.Subscriber('/odom', Odometry, odom_callback)  
    rospy.Subscriber('/reaching_goal/result', PlanningActionResult, result_callback)
    
    # Initialize action client
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    ## 
    # \brief Waits for the action server to start.
    # \param None
    client.wait_for_server()
    while not rospy.is_shutdown():
    # User input loop
    	while True:
    		user_input = input("Write g to set a new target or c to cancel the current one: ")
    		if user_input in ['g','c']:
    			break
    		else:
    			print("Wrong input. Please insert letter g or letter c")
    	if user_input == 'g':
    		goal = PlanningGoal()
    		while True:
    			try:
    				goal.target_pose.pose.position.x = float(input("Insert the x coordinate: "))
    				break
    			except ValueError:
    				print("Wrong input. Please insert a valid number.")
    		while True:
    			try:
    				goal.target_pose.pose.position.y = float(input("Insert the y coordinate: "))
    				break
    			except ValueError:
    				print("Wrong input. Please insert a valid number.")
    		##
    		# \brief Sends a goal to the action server:
    		# \param goal The target goal position (set by the user in the terminal).
    		client.send_goal(goal)
    	elif user_input == 'c':
    		##
    		# \brief Cancels the current goal being processed by the action server:
    		# \param None
    		client.cancel_goal()
    		
    		

##
# \brief Entry point of the script.   
if __name__ == '__main__':
    main()
 
