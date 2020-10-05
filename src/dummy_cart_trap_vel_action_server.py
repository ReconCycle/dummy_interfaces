#! /usr/bin/env python
#
# Action server for P2P motion of the robot to a desired pose with speed percentage.
#
# Authors: Barry Ridge, Timotej Gaspar
# E-mail: {barry.ridge, timotej.gaspar}@ijs.si
#


# ROS stuff
import rospy
import actionlib

# Action messages
import robot_module_msgs.msg

# ===================
# Action Server Class
# ===================
class CartTrapVelAction(object):

    # ROS rate variables
    _rate = None

    # Create messages that are used to publish feedback/result
    _feedback = robot_module_msgs.msg.CartTrapVelFeedback()
    _result = robot_module_msgs.msg.CartTrapVelResult()

    def __init__(self, name):

        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, robot_module_msgs.msg.CartTrapVelAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Create the ROS rate
        self._rate = rospy.Rate(125)

    # ------------------------------------------
    # Main action server execution callback
    # ------------------------------------------
    def execute_cb(self, goal):
        """Main action execution callback method."""

        rospy.loginfo(
            "[{0}]: Executing Cartesian Trapezoidal Velocity motion to pose:\n{1}"
            .format(self._action_name, goal.target_pose[0]))

        # Use the provided initial joints if they came, otherwise use the current joint configuration for calculating inverse kinematics
        if goal.init_joints == []:
            rospy.logwarn("{0}: No initial joint position provided. Using robot's current joint position as initial for inverse kinematics.".format(self._action_name))

        start_time = rospy.get_time()
        while ((rospy.get_time() - start_time) < 1):

            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.logwarn('[{0}]: Preempted'.format(self._action_name))
                self._result.fully_succeeded = False
                self._as.set_preempted(self._result)
                return

            # Publish the feedback
            self._as.publish_feedback(self._feedback)

            self._rate.sleep()

        rospy.loginfo('[{0}]: Fully succeeded'.format(self._action_name))
        self._result.fully_succeeded = True
        self._as.set_succeeded(self._result)
        return



if __name__ == '__main__':
    try:
        rospy.init_node('cart_trap_vel_action_server')
        CartTrapVelAction(rospy.get_name())

        rospy.loginfo("Dummy action server \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
