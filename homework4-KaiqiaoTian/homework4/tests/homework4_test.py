#! /usr/bin/env python
import unittest
import rospy
import copy
import message_filters
import math
import time
from rosgraph_msgs.msg import Clock
from autoware_msgs.msg import DetectedObject, DetectedObjectArray


class Homework4TestCase(unittest.TestCase):
  def setUp(self):
    self.start_time = time.time()
    self.received_synced_topics = False

    self.sync_ref_objects = message_filters.Subscriber('/object_tracks', DetectedObjectArray)
    self.sync_homework_objects = message_filters.Subscriber('/homework4/object_tracks', DetectedObjectArray)
    self.time_syncer = message_filters.TimeSynchronizer([self.sync_ref_objects, self.sync_homework_objects], 100)
    self.time_syncer.registerCallback(self.recv_synced_topics)
    self.sub_clock = rospy.Subscriber('/clock', Clock, self.recv_clock)

  def tearDown(self):
    rospy.loginfo('Test completed in %f seconds' % (time.time() - self.start_time))
  
  def test_ekf_output(self):
    time_resolution = 0.01
    timeout_t = time.time() + 5.0
    while not rospy.is_shutdown() and (timeout_t - time.time()) > 0:
      time.sleep(0.01)
    self.assertTrue(self.received_synced_topics, msg="No messages received from '/homework4/detected_objects' topic")
    self.run_check(90.0)

  def run_check(self, time_window):
    time_resolution = 0.01
    timeout_t = time.time() + time_window
    while not rospy.is_shutdown() and (timeout_t - time.time()) > 0:
      if (time.time() - self.last_clock_msg) > 0.04:
        break

      if self.received_synced_topics:
        self.received_synced_topics = False

        if len(self.homework_objects.objects) == 0 and len(self.ref_objects.objects) > 0:
          self.fail(msg='No objects received')

        num_pos_failures = 0
        num_vel_failures = 0
        for homework_obj in self.homework_objects.objects:

          self.assertGreater(homework_obj.dimensions.x, 0, msg='Dimensions in output were 0')
          self.assertGreater(homework_obj.dimensions.y, 0, msg='Dimensions in output were 0')
          self.assertGreater(homework_obj.dimensions.z, 0, msg='Dimensions in output were 0')

          closest_obj = None
          min_dist2 = 9999
          for ref_obj in self.ref_objects.objects:
            dx = ref_obj.pose.position.x - homework_obj.pose.position.x
            dy = ref_obj.pose.position.y - homework_obj.pose.position.y
            d2 = dx * dx + dy * dy
            if d2 < min_dist2:
              min_dist2 = d2
              closest_obj = ref_obj

          dist_tol = 1
          if min_dist2 > (dist_tol * dist_tol):
            num_pos_failures += 1
          self.assertLess(num_pos_failures, 3, msg='Filtered position did not match reference (%f meters off)' % math.sqrt(min_dist2))

          homework_vel_magnitude = math.sqrt(homework_obj.velocity.linear.x * homework_obj.velocity.linear.x + homework_obj.velocity.linear.y * homework_obj.velocity.linear.y)
          self.assertGreater(homework_vel_magnitude, 0.0, msg='No velocity data in output')
          ref_vel_magnitude = math.sqrt(closest_obj.velocity.linear.x * closest_obj.velocity.linear.x + closest_obj.velocity.linear.y * closest_obj.velocity.linear.y)
          vel_tol = 1
          if abs(homework_vel_magnitude - ref_vel_magnitude) > vel_tol:
            num_vel_failures += 1
          self.assertLess(num_vel_failures, 5, msg='Filtered velocity did not match reference (%f m/s off)' % abs(homework_vel_magnitude - ref_vel_magnitude))

      time.sleep(time_resolution)

  def recv_synced_topics(self, ref_objects, homework_objects):
    self.received_synced_topics = True
    self.ref_objects = ref_objects
    self.homework_objects = homework_objects
  
  def recv_clock(self, msg):
    self.last_clock_msg = time.time()


class Homework4Test(unittest.TestSuite):
  def __init__(self):
    rospy.init_node('homework4_tests')
    super(Homework4Test, self).__init__()
    self.addTest(Homework4TestCase('test_ekf_output'))

