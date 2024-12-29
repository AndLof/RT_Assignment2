#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
from nav_msgs.msg import Odometry
from assignment2_rt.msg import Custom_info


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

    rospy.spin()

    
if __name__ == '__main__':
    main()
 
