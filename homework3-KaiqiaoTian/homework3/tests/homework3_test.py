#! /usr/bin/env python
import unittest
import rospy
import copy
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import PoseArray
from rosgraph_msgs.msg import Clock
import math
import time


class Homework3TestCase(unittest.TestCase):
  def setUp(self):
    self.start_time = time.time()
    self.received_cluster_cloud = False
    self.received_normals = False
    self.received_objects = False
    self.received_synced_topics = False

    self.sub_cloud = rospy.Subscriber('/merged_cluster_cloud', PointCloud2, self.recv_cluster_cloud)
    self.sub_normals = rospy.Subscriber('/normals', PoseArray, self.recv_normals)
    self.sub_objects = rospy.Subscriber('/objects', BoundingBoxArray, self.recv_objects)
    self.sub_clock = rospy.Subscriber('/clock', Clock, self.recv_clock)
    self.using_sim_time = rospy.get_param('/use_sim_time', default=False)

    self.sync_sub_cloud = message_filters.Subscriber('/merged_cluster_cloud', PointCloud2)
    self.sync_sub_normals = message_filters.Subscriber('/normals', PoseArray)
    self.sync_sub_objects = message_filters.Subscriber('/objects', BoundingBoxArray)
    self.time_syncer = message_filters.TimeSynchronizer([self.sync_sub_cloud, self.sync_sub_normals, self.sync_sub_objects], 10)
    self.time_syncer.registerCallback(self.recv_synced_topics)
    self.cluster_cloud = PointCloud2()
    self.normals = PoseArray()
    self.objects = BoundingBoxArray()
    
  def tearDown(self):
    rospy.loginfo('Test completed in %f seconds' % (time.time() - self.start_time))
  
  def test_pointcloud_processing(self):
    self.assertTrue(self.using_sim_time, msg='ROS not using clock from bag file')

    time_resolution = 0.01
    timeout_t = time.time() + 5.0
    found_individual_topics = False
    while not rospy.is_shutdown() and (timeout_t - time.time()) > 0:
      if self.received_cluster_cloud and self.received_normals and self.received_objects:
        found_individual_topics = True
        break
      time.sleep(0.01)

    self.assertTrue(self.received_cluster_cloud, msg='Merged cluster cloud topic not published')
    self.assertTrue(self.received_normals, msg='Normals topic not published')
    self.assertTrue(self.received_objects, msg='Objects topic not published')

    self.run_check(90.0)

  def run_check(self, time_window):
    time_resolution = 0.01
    timeout_t = time.time() + time_window
    while not rospy.is_shutdown() and (timeout_t - time.time()) > 0:
      if (time.time() - self.last_clock_msg) > 0.04:
        break

      if self.recv_synced_topics:
        self.received_synced_topics = False

        self.assertEqual(self.cluster_cloud.header.frame_id, 'base_footprint', msg='Cluster cloud frame ID not correct')
        self.assertEqual(self.normals.header.frame_id, 'base_footprint', msg='Normals frame ID not correct')
        self.assertEqual(self.objects.header.frame_id, 'base_footprint', msg='Objects frame ID not correct')

        self.assertGreater(len(self.objects.boxes), 3, msg='Not enough clusters detected')

        pointcloud_size_gen = point_cloud2.read_points(self.cluster_cloud)
        num_points = 0
        for p in pointcloud_size_gen:
          num_points += 1
        self.assertGreater(num_points, 200, msg='Not enough cluster points')
        self.assertGreater(len(self.normals.poses), 200, msg='Not enough normal poses')

        local_object_copy = copy.deepcopy(self.objects)
        for i in range(0, len(local_object_copy.boxes)):
          b = local_object_copy.boxes[i]
          self.assertTrue(b.dimensions.x > 0 and b.dimensions.y > 0 and b.dimensions.z > 0, msg='Detected object with zero size')
          self.assertTrue(b.dimensions.x < 6 or b.dimensions.y < 6, msg='Detected object has too much planar area (%f x %f)' % (b.dimensions.x, b.dimensions.y))
          self.assertEqual(b.label, i, msg='Incorrect object bounding box ID')
          self.assertTrue(b.pose.position.x != 0 or b.pose.position.y != 0, msg='Detected object with zero position')
          self.assertEqual(b.pose.orientation.w, 1, msg='Invalid orientation on bounding box (should be w = 1, x,y,z = 0)')

        for n in self.normals.poses:
          val = 2 * (n.orientation.x * n.orientation.z - n.orientation.w * n.orientation.y) # Z component of first basis vector
          ang = math.acos(val)
          self.assertTrue(ang > (math.pi / 6 - 0.02) and ang < (5 * math.pi / 6 + 0.02), msg='Normal at incorrect angle: %f degrees' % (180.0 / math.pi * ang))

      time.sleep(time_resolution)

  def recv_synced_topics(self, cluster_cloud, normals, objects):
    self.received_synced_topics = True
    self.cluster_cloud = cluster_cloud
    self.normals = normals
    self.objects = objects

  def recv_cluster_cloud(self, msg):
    self.received_cluster_cloud = True
    self.sub_cloud.unregister()

  def recv_normals(self, msg):
    self.received_normals = True
    self.sub_normals.unregister()

  def recv_objects(self, msg):
    self.received_objects = True
    self.sub_objects.unregister()
   
  def recv_clock(self, msg):
    self.last_clock_msg = time.time()


class Homework3Test(unittest.TestSuite):
  def __init__(self):
    rospy.init_node('homework3_tests')
    super(Homework3Test, self).__init__()
    self.addTest(Homework3TestCase('test_pointcloud_processing'))
