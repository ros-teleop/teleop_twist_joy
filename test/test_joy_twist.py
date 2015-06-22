#!/usr/bin/env python
PKG = 'teleop_twist_joy'

import sys
import unittest
import time
import rospy
import geometry_msgs.msg
import sensor_msgs.msg

class TestJoyTwist(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_joy_twist_node', anonymous=True)
        self.pub = rospy.Publisher('joy', sensor_msgs.msg.Joy)
        rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, callback=self.callback)

        while (not rospy.has_param("~publish_joy")) and (not rospy.get_param("~expect_cmd_vel")):
            time.sleep(0.1)

        self.expect_cmd_vel = rospy.get_param("~expect_cmd_vel")
        self.joy_msg = rospy.get_param("~publish_joy")
        self.received_cmd_vel = None

    def test_expected(self):
        pub_joy = sensor_msgs.msg.Joy()
        pub_joy.axes.extend(self.joy_msg['axes'])
        pub_joy.buttons.extend(self.joy_msg['buttons'])
        while self.received_cmd_vel is None:
            self.pub.publish(pub_joy)
            time.sleep(0.1)

        self.assertAlmostEqual(self.expect_cmd_vel['linear'][0]['x'], self.received_cmd_vel.linear.x)
        self.assertAlmostEqual(self.expect_cmd_vel['linear'][1]['y'], self.received_cmd_vel.linear.y)
        self.assertAlmostEqual(self.expect_cmd_vel['angular'][2]['z'], self.received_cmd_vel.angular.z)

    def callback(self, msg):
        self.received_cmd_vel = geometry_msgs.msg.Twist()
        self.received_cmd_vel = msg



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_joy_twist', TestJoyTwist)
