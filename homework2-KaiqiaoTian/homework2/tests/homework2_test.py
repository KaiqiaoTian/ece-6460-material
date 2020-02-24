#! /usr/bin/env python
import unittest
import rospy
import math
import geodesy.utm
import time
from autoware_msgs.msg import Lane, Waypoint
from gps_common.msg import GPSFix
from geometry_msgs.msg import Point


class Homework2TestCase(unittest.TestCase):
  def setUp(self):
    self.start_time = time.time()
    
    rospy.Subscriber('/final_waypoints', Lane, self.recv_lane)
    rospy.Subscriber('/vehicle/perfect_gps/enhanced_fix', GPSFix, self.recv_fix)

    self.waypoints = []
    self.fix = GPSFix()
    self.received_waypoints = False
    self.received_fix = False
    self.max_error = 0
    self.max_speed = 0
  
  def tearDown(self):
    rospy.loginfo('Test completed in %f seconds with max error of %f' % (time.time() - self.start_time, self.max_error))
  
  def test_tracking_accuracy(self):
    while not (self.received_waypoints and self.received_fix) and not rospy.is_shutdown():
      rospy.sleep(0.01)
    
    rospy.sleep(0.1)
    self.assertTrue(self.received_fix, msg='Test script did not receive GPS fix')
    self.assertTrue(self.received_waypoints, msg='Test script did not receive lane data')

    self.run_check(60.0)

    self.assertGreater(
      a=self.max_speed,
      b=2.27,
      msg='Vehicle never moved'
    )

  def run_check(self, time_window):
    time_resolution = 0.01
    num_tolerance_violations = 0
    timeout_t = rospy.Time.now() + rospy.Duration(time_window)
    while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
      if self.received_waypoints:
        self.received_waypoints = False
        lat_error_tol = 0.3
        lat_error = self.compute_lat_error()
        if lat_error > self.max_error:
          self.max_error = lat_error
        
        if self.fix.speed > self.max_speed:
          self.max_speed = self.fix.speed

        if lat_error > lat_error_tol:
          num_tolerance_violations += 1

        self.assertLessEqual(
          a=num_tolerance_violations,
          b=3,
          msg=('Lateral error of %f exceeded tolerance of %f' % (self.max_error, lat_error_tol))
        )

      rospy.sleep(time_resolution)

  def compute_lat_error(self):
    
    utm_position = geodesy.utm.fromLatLong(self.fix.latitude, self.fix.longitude)
    enu_heading = math.pi / 2 - self.fix.track
    central_meridian = 6 * (utm_position.zone - 1) - 177
    convergence_angle = math.atan(math.tan(self.fix.longitude + central_meridian) * math.sin(self.fix.latitude))
    heading = enu_heading + convergence_angle
    local_waypoints = []
    for wp in self.waypoints:
      dx = wp.pose.pose.position.x - utm_position.easting
      dy = wp.pose.pose.position.y - utm_position.northing
      local_wp = Waypoint()
      local_wp.pose.pose.position.x = math.cos(heading) * dx + math.sin(heading) * dy
      local_wp.pose.pose.position.y = -math.sin(heading) * dx + math.cos(heading) * dy
      local_wp.pose.pose.position.z = wp.pose.pose.position.z
      local_waypoints.append(local_wp)

    min_dist2 = float('inf')
    min_idx = -1
    for i in range(0, len(local_waypoints)):
      d2 = wp.pose.pose.position.x * wp.pose.pose.position.x + wp.pose.pose.position.y * wp.pose.pose.position.y
      if d2 < min_dist2:
        min_dist2 = d2
        min_idx = i

    return abs(local_waypoints[min_idx].pose.pose.position.y)

  def recv_lane(self, msg):
    self.waypoints = msg.waypoints
    self.received_waypoints = True

  def recv_fix(self, msg):
    self.fix = msg
    self.received_fix = True
  
class Homework2Test(unittest.TestSuite):
  def __init__(self):
    rospy.init_node('homework2_tests')
    super(Homework2Test, self).__init__()
    self.addTest(Homework2TestCase('test_tracking_accuracy'))
