# ECE 6460 Homework #3

The purpose of this assignment is to provide a hands-on example of processing point cloud data from a 3D LIDAR in a typical public road setting. The goal is to detect 3D objects in the vehicle's surroundings.

## Due Date
This assignment is due on Saturday, October 26th at midnight.

## Provided Files
- Initial **homework3.launch** that must be modified to complete the assignment
- Initial **homework3_node.cpp**, **Homework3.cpp** and **Homework3.hpp** which must be modified to implement the required functionality of the **homework3** node.

## Requirements
Edit the template source files for the **homework3** node and the **homework3.launch** file to meet the following requirements.

#### 1. Set Up Static TF Transform
- Use the `static_transform_publisher` node to define a TF frame transform from `base_footprint` (parent frame) to `ouster` (child frame)
- The transformation between base_footprint and ouster is defined by:
	- Translation: `0.6` x, `0.0` y, `1.5` z
	- Orientation: `0.00` roll, `-0.04` pitch, `-0.02` yaw

#### 2. Process 3D LIDAR Points
- Subscribe to the raw point cloud topic `/ouster/points_raw` which is of type `sensor_msgs/PointCloud2`
- Transform incoming point cloud from the `ouster` frame to `base_footprint`
- Apply a passthrough filter to transformed cloud that only keeps points within the following region of interest (ROI):
	- `0.0 < x < 100.0`
	- `-10.0 < y < 10.0`
	- `-2.0 < z < 3.0`
- Apply a downsampling filter to the ROI cloud with a voxel size of 0.2 m.
- Remove all points from the downsampled cloud with normal vectors less than 30 degrees from vertical.
- Run the Euclidean clustering algorithm on the cloud. Use the following settings:
	- Cluster tolerance: `0.5`
	- Min cluster size: `5`
	- Max cluster size: `5000`
 
#### 3. Populate and Publish Outputs
- Merge each individual cluster cloud into a single point cloud, copy the result into a `sensor_msgs/PointCloud2` message and publish it on the `/merged_cluster_cloud` topic.
- Populate a `jsk_recognition_msgs/BoundingBox` for each cluster, add it to a `jsk_recognition_msgs/BoundingBoxArray` message, and publish it on the `/objects` topic.
- Publish a `geometry_msgs/PoseArray` message on the `/normals` topic that contains the normal vectors of the processed cloud.

## Test Procedure
The test script for Homework 3 verifies the following:

- Makes sure ROS is set to use clock from bag file
- Checks merged cluster cloud topic `/merged_cluster_cloud`
	- Messages are published
	- Frame ID is `base_footprint`
	- Appropriate number of points present in each message
- Checks filtered normals array topic `/normals`
	- Messages are published
	- Frame ID is `base_footprint`
	- Appropriate number of poses present in each message
	- None of the poses are less than 30 degrees from vertical (up or down)
- Checks the object bounding box array topic `/objects`
	- Messages are published
	- Frame ID is `base_footprint`
	- Appropriate number of poses present in each message
	- None of the bounding boxes exceed a size of 6 meters in both the x and y axes