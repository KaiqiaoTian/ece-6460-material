#! /usr/bin/env python
import unittest
import rospy
import time
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64


class Homework1TestCase(unittest.TestCase):
  def setUp(self):
    self.twist = TwistStamped()
    self.speed = 0
    self.received_twist = False
    self.received_speed = False
    self.start_time = time.time()
    
    rospy.Subscriber('vehicle/twist', TwistStamped, self.recv_twist)
    rospy.Subscriber('vehicle_speed', Float64, self.recv_speed)
    self.verification_errors = []
  
  def tearDown(self):
    rospy.loginfo('Test completed in %f seconds' % (time.time() - self.start_time))
    self.assertEqual([], self.verification_errors)
  
  def test_speed_republish(self):
    while not self.received_twist and not rospy.is_shutdown():
      rospy.sleep(0.01)
    
    rospy.sleep(0.1)
    self.assertTrue(self.received_speed, msg='Test script did not receive vehicle speed topic')

    self.run_check(10.0)

  def run_check(self, time_window):
    time_resolution = 0.01
    timeout_t = rospy.Time.now() + rospy.Duration(time_window)
    while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
      if self.received_speed and self.received_twist:
        self.received_speed = False
        self.received_twist = False
        try:
          self.assertAlmostEqual(
            first=self.twist.twist.linear.x,
            second=self.speed,
            msg='Values did not match (twist message speed = %f, speed topic value = %f)' % (self.twist.twist.linear.x, self.speed)
          )
        except AssertionError, e:
          self.verification_errors.append(str(e))

      rospy.sleep(time_resolution)

  def recv_speed(self, msg):
    self.speed = msg.data
    self.received_speed = True

  def recv_twist(self, msg):
    self.twist = msg
    self.received_twist = True

class Homework1Test(unittest.TestSuite):
  def __init__(self):
    rospy.init_node('homework1_tests')
    super(Homework1Test, self).__init__()
    self.addTest(Homework1TestCase('test_speed_republish'))
