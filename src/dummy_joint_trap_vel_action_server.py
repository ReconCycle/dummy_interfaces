#! /usr/bin/env python
#
# Dummy action server to simulate the behaviour of the joint_trap_vel_action_server.
#
# Authors: Timotej Gaspar
# E-mail: timotej.gaspar AT ijs.si
#

# Numpy

# ROS stuff
import rospy
import actionlib

# Action messages
import robot_module_msgs.msg

# ===================
# Action Server Class
# ===================
class JointTrapVelAction(object):

    _feedback = robot_module_msgs.msg.JointTrapVelFeedback()
    _result = robot_module_msgs.msg.JointTrapVelResult()
    _action_name = ''
    # -----------
    # Constructor
    # -----------
    def __init__(self, name):
        """Constructor."""

        self._action_name = name
        # Start the action server
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                robot_module_msgs.msg.JointTrapVelAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)

        # Initialize the rate
        self._rate = rospy.Rate(125)

        # Start the action server
        self._as.start()

    # ------------------------------------------
    # Main action server execution callback
    # ------------------------------------------
    def execute_cb(self, goal):
        """Main action execution callback method."""

        # Publish info to the console for the user
        rospy.loginfo("[{0}]: Executing joint motion from current joints:\n{1} to joints: \n{2}"
                      " with {3} speed percentage.".format(self._action_name,
                                                           "[dummy action server, no current joints]",
                                                           goal.joints[0].position,
                                                           goal.speed_percent*100.0))


        start_time = rospy.get_time()
        # Start executing the action
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
        rospy.init_node('joint_trap_vel_action_server')
        JointTrapVelAction(rospy.get_name())

        rospy.loginfo("Dummy action server \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
