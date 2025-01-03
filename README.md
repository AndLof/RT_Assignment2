# RT_Assignment2 (part 1)

# **ROS Package Documentation**

## **Description**

This ROS package consists of two nodes implemented in python. The purpose of the project is to manage and control a robot in a simulation environment, provided by the assignment. The first node consinsts in the action client and it allows the user to set a target or to cancel it; it uses the status of the action server to know when the target has been reached. Besides, the action client publishes the 
robot position and velocity as a custom message. The second node is a service node that, when called, returns the coordinates of the last target sent by the user. The package contains also a launch file that allows the user to execute all the required nodes to run the simulation.

---

## **Nodes Description**

### **First Node - `action_client`**

The action client allows the user to input the desired goal position.
The primary purpose of this node is to:
1. **input the desired goal position** 
   To set a new goal, the user simply has to press "g" and insert the coordinates of the desired goal position, follow the instruction displayed on the terminal.
   The robot will then start moving in order to reach the goal. While the robot is moving, the user can still input a new target position: in this case, the previous task will be preempted and the robot will adjust its movement to reach the new desired position. Notice that the action client provides also information about the running task: it may happen that a message is displayed on the terminal, with the information about the status of the task (e.g SUCCEEDED, PREEMPTED,...). However, the user can continue to input g to set a new goal or c to cancel it.

2. **cancel the goal**. 
   To cancel the goal, it is sufficient to input "c" instead of "g". Also in this case, it is shown the information about the status of the goal and the user will have the possibility to set a desired target position.

The action client publishes also the information about robot position and velocity as a custom message on the topic /custom_info
---

### **Second Node - `last_goal_service`**

This node is a service node that returns the coordinates of the last goal sent by the user. The node is executed with the launch file; since there is no other nodes that call the service, to test it is sufficient to open a new terminal and run $rosservice call /last_goal. A message (with the same structure of the goal message) will be displayed on the terminal.


---

## **How to Run the Package**

IMPORTANT: in order to be correctly executed, the implemented launch file requires the presence of the package "assignment_2_2024", provided by the assignment.

To run the simulation it is sufficient to run $roslaunch assignment2_rt assignment2_rt.launch

---




