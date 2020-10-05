#! /usr/bin/env python
PKG='dummy_interfaces'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

# Python unit tests
import unittest

# ROS stuff
import rospy

# Brings in the SimpleActionClient
import actionlib

# Neccesarry ROS messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

PREEMPTED = GoalStatus.PREEMPTED
SUCCEEDED = GoalStatus.SUCCEEDED

# Action messages
import robot_module_msgs.msg as amsgs

# Colour terminal print strings.
FAIL = '\033[91m'
ENDC = '\033[0m'

class TestJointTrap(unittest.TestCase):

    def runTest(self):
        # Test success variable
        test_success = []

        try:
            client = actionlib.SimpleActionClient('joint_trap_vel_action_server', amsgs.JointTrapVelAction)

            # Waits until the action server has started up and started
            # listening for goals.
            if not(client.wait_for_server(rospy.Duration(2))):
                test_success.append(False)
                raise Exception("Could not connect to action server")

            # Creates a goal to send to the action server.
            goal = amsgs.JointTrapVelGoal([JointState()], 0.05, None)

            # Test normal call to the action server
            test_success.append(
                client.send_goal_and_wait(
                    goal,
                    execute_timeout=rospy.Duration(5)) == SUCCEEDED)

            # Test preemtion
            test_success.append(
                client.send_goal_and_wait(
                    goal,
                    execute_timeout=rospy.Duration(0.1),
                    preempt_timeout=rospy.Duration(0.5)) == PREEMPTED)

            self.assertTrue(all(test_success))
        except Exception as e:
            self.assertTrue(all(test_success), "Test failed with exception: {0}".format(e))

class TestCartTrap(unittest.TestCase):

    def runTest(self):
        # Test success variable
        test_success = []

        try:
            client = actionlib.SimpleActionClient('cart_trap_vel_action_server', amsgs.CartTrapVelAction)

            # Waits until the action server has started up and started
            # listening for goals.
            if not(client.wait_for_server(rospy.Duration(2))):
                raise Exception("Could not connect to action server")

            # Creates a goal to send to the action server.
            goal = amsgs.CartTrapVelGoal([Pose()], 0.05, None, None)

            # Test normal call to the action server
            test_success.append(
                client.send_goal_and_wait(
                    goal,
                    execute_timeout=rospy.Duration(5)) == SUCCEEDED)

            # Test preemtion
            test_success.append(
                client.send_goal_and_wait(
                    goal,
                    execute_timeout=rospy.Duration(0.1),
                    preempt_timeout=rospy.Duration(0.5)) == PREEMPTED)

            self.assertTrue(all(test_success), "Success!")
        except Exception as e:
            self.assertTrue(False, "Test failed with exception: {0}".format(e))

class TestCartLin(unittest.TestCase):

    def runTest(self):
        # Test success variable
        test_success = []

        try:
            client = actionlib.SimpleActionClient('cart_lin_task_action_server', amsgs.CartLinTaskAction)

            # Waits until the action server has started up and started
            # listening for goals.
            if not(client.wait_for_server(rospy.Duration(2))):
                raise Exception("Could not connect to action server")

            # Creates a goal to send to the action server.
            goal = amsgs.CartLinTaskGoal([Pose()], 2.0, None)

            # Test normal call to the action server
            test_success.append(
                client.send_goal_and_wait(
                    goal,
                    execute_timeout=rospy.Duration(5)) == SUCCEEDED)

            # Test preemtion
            test_success.append(
                client.send_goal_and_wait(
                    goal,
                    execute_timeout=rospy.Duration(0.1),
                    preempt_timeout=rospy.Duration(0.5)) == PREEMPTED)

            self.assertTrue(all(test_success), "Success!")
        except Exception as e:
            self.assertTrue(False, "Test failed with exception: {0}".format(e))


if __name__ == '__main__':
    import rosunit
    rospy.init_node('dummy_action_server_tester')
    rosunit.unitrun(PKG, 'test_joint_trap', TestJointTrap)
    rosunit.unitrun(PKG, 'test_cart_trap', TestCartTrap)
    rosunit.unitrun(PKG, 'test_cart_lin', TestCartLin)