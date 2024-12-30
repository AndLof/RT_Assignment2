#! /usr/bin/env python

import rospy
import actionlib
from assignment_2_2024.msg import PlanningActionGoal #PlanningGoal, PlanningActionResult
from assignment2_rt.srv import Last_goal, Last_goalResponse
from geometry_msgs.msg import PoseStamped

last_goal = None

def goal_callback(msg):
	global last_goal
	last_goal = msg.goal.target_pose
	#rospy.loginfo(f"Coordinates of last goal: {last_goal}")

def service_function(req):
	global last_goal
	#rospy.loginfo("Coordinates of last goal:")
	return Last_goalResponse(last_goal)


def main():
	rospy.init_node('last_goal_service')

	rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)

	service = rospy.Service('last_goal', Last_goal, service_function)

	rospy.spin()

if __name__ == '__main__':
	main()

