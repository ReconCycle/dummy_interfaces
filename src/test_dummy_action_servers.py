#! /usr/bin/env python

# ROS stuff
import rospy

# Brings in the SimpleActionClient
import actionlib

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# Action messages
import robot_module_msgs.msg

from actionlib_msgs.msg import GoalStatus

# Time
import time
import copy

PREEMPTED = GoalStatus.PREEMPTED
SUCCEEDED = GoalStatus.SUCCEEDED


def joint_trap_motion_test():
    # Creates the SimpleActionClient, passing the type of the action
    # (CartTrapVelAction) to the constructor.
    client = actionlib.SimpleActionClient('joint_trap_vel_action_server', robot_module_msgs.msg.JointTrapVelAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for {0} ...".format(client.action_client.ns))
    if not(client.wait_for_server(rospy.Duration(2))):
        raise Exception("Action server unreachable !!")


    # Creates a goal to send to the action server.
    goal_joints = JointState()
    goal = robot_module_msgs.msg.JointTrapVelGoal([goal_joints], 0.05, None)

    # Sends the goal to the action server and wait for it to finish
    rospy.loginfo("Normal call to action server ...")
    goal_state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(5))
    rospy.loginfo("Done !!")

    # Test preemtion
    # Sends the goal to the action server and wait for it to finish
    rospy.loginfo("Call to action server with sudden preemption ...")
    goal_state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(0.1), preempt_timeout=rospy.Duration(0.5))
    rospy.loginfo("Done !!")

    return client.get_result()


def cart_trap_motion_test():
    # Creates the SimpleActionClient, passing the type of the action
    # (CartTrapVelAction) to the constructor.
    client = actionlib.SimpleActionClient('cart_trap_vel_action_server', robot_module_msgs.msg.CartTrapVelAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for {0} ...".format(client.action_client.ns))
    if not(client.wait_for_server(rospy.Duration(2))):
        raise Exception("Action server unreachable !!")

    # Create a goal
    t_pose_target = Pose()
    goal = robot_module_msgs.msg.CartTrapVelGoal([t_pose_target], 0.05, None, None)
    # Sends the goal to the action server and wait for it to finish
    rospy.loginfo("Normal call to action server ...")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Done !!")
    # Test preemtion
    # Sends the goal to the action server and wait for it to finish
    rospy.loginfo("Call to action server with sudden preemption ...")
    client.send_goal(goal)
    rospy.sleep(0.5)
    client.cancel_all_goals()
    client.wait_for_result()
    rospy.loginfo("Done !!")

    return client.get_result()


def cart_lin_motion_test():
    # Creates the SimpleActionClient, passing the type of the action
    # (CartLinTaskAction) to the constructor.
    client = actionlib.SimpleActionClient('cart_lin_task_action_server', robot_module_msgs.msg.CartLinTaskAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for {0} ...".format(client.action_client.ns))
    client.wait_for_server()
    if not(client.wait_for_server(rospy.Duration(2))):
        raise Exception("Action server unreachable !!")

    # Create a goal
    t_pose_target = Pose()
    goal = robot_module_msgs.msg.CartLinTaskGoal([t_pose_target], 0.05, None)
    # Sends the goal to the action server and wait for it to finish
    rospy.loginfo("Normal call to action server ...")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Done !!")
    # Test preemtion
    # Sends the goal to the action server and wait for it to finish
    rospy.loginfo("Call to action server with sudden preemption ...")
    client.send_goal(goal)
    rospy.sleep(0.5)
    client.cancel_all_goals()
    client.wait_for_result()
    rospy.loginfo("Done !!")

    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('dummy_action_server_tester')
        joint_trap_motion_test()
        cart_trap_motion_test()
        cart_lin_motion_test()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion")
    except Exception as e:
        rospy.logerr("Exception during the test phase:\n{0}".format(e))
