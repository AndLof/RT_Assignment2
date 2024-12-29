#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
from nav_msgs.msg import Odometry
from assignment2_rt.msg import Custom_info
from assignment_2_2024.msg import PlanningAction, PlanningGoal

def odom_callback(msg):
    global pub
    custom_msg = Custom_info()
    custom_msg.x = msg.pose.pose.position.x
    custom_msg.y = msg.pose.pose.position.y
    custom_msg.vel_x = msg.twist.twist.linear.x
    custom_msg.vel_z = msg.twist.twist.angular.z
    pub.publish(custom_msg)


def main():
    global pub
    rospy.init_node('action_client')

    # Publish on topic custom_info
    pub = rospy.Publisher('/custom_info', Custom_info, queue_size=10)
    #rospy.wait_for_message('/odom', Odometry)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    #rospy.spin()
    
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    while not rospy.is_shutdown():
    	user_input = input("Write g to set a new target or c to cancel the current one: ")
    	if user_input == 'g':
    		goal = PlanningGoal()
    		goal.target_pose.pose.position.x = float(input("Insert the x coordinate: "))
    		goal.target_pose.pose.position.y = float(input("Insert the y coordinate: "))
    		client.send_goal(goal)
    	elif user_input == 'c':
    		client.cancel_goal()
    		
    		

    
if __name__ == '__main__':
    main()
 
