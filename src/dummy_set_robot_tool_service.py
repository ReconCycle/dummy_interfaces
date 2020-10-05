#!/usr/bin/env python

import rospy
# import numpy as np
from robot_module_msgs.srv import SetRobotTool

def handle_set_robot_tool(req):

    r = rospy.Rate(125)
    r.sleep()

    # If only the tool weight is provided change only the tool weight
    if (all([len(req.toolPosition) == 0, len(req.toolRotation) == 0, len(req.toolName) == 0])):
        rospy.loginfo('Changing only the weight of the tool (Dummy service)')

    # If only the tool name is provided check if it exists in TF and change accoridngly
    elif (all([len(req.toolPosition) == 0, len(req.toolRotation) == 0, (req.weight == 0)])) and (
        len(req.toolName) != 0):
        rospy.loginfo('Setting tool to: ' + req.toolName + '(Dummy service)')


    elif len(req.toolPosition) != 3:
        rospy.logwarn('Wrong input position data!')
        return [False, "Wrong input position data!"]

    elif (all([len(req.toolPosition) != 0, len(req.toolRotation) != 0, len(req.toolName) == 0])):
        rospy.loginfo('Changing tool position and orientation to (Dummy service)')

    else:
        rospy.logwarn('Wrong input position data!')
        rospy.logwarn('You provided the tool name and tool transformation. Provide one OR the other!')
        return [False, "Wrong input data!"]


    return [True, "Success!"]


def set_robot_tool():
    rospy.init_node('set_robot_tool_service')
    # Initialize the 'tool_name' topic
    # initialize_topic()
    s = rospy.Service('set_robot_tool', SetRobotTool, handle_set_robot_tool)
    rospy.loginfo("Set new tool service running! (Dummy service)")

    rate = rospy.Rate(125)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    set_robot_tool()
