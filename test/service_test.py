#! /usr/bin/env python
PKG='dummy_interfaces'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

# Python unit tests
import unittest

# ROS stuff
import rospy

# Import service message 
from robot_module_msgs.srv import SetRobotTool

# Colour terminal print strings.
FAIL = '\033[91m'
ENDC = '\033[0m'

class TestSetRobotTool(unittest.TestCase):

    def runTest(self):
        # Test success variable
        test_success = []

        try:
            service = rospy.ServiceProxy('set_robot_tool',SetRobotTool)
            
            # Waits until the action server has started up and started
            # listening for goals.
            service.wait_for_service(timeout=2)

            # Test changing name of the tool
            test_success.append(
                service.call(toolName='toolChanger').success
            )

            # Test changing position and rotation of the tool
            test_success.append(
                service.call(toolPosition=[0,0,0], toolRotation=[1,0,0,0]).success
            )
            
            # Test changing weight of the tool
            test_success.append(
                service.call(weight=1).success
            )


            self.assertTrue(all(test_success))
        except Exception as e:
            self.assertTrue(False, "Test failed with exception: {0}".format(e))

if __name__ == '__main__':
    import rosunit
    rospy.init_node('dummy_service_tester')
    rosunit.unitrun(PKG, 'test_set_robot_tool', TestSetRobotTool)