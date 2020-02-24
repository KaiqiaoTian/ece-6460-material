# ECE 6460 Homework #5

The purpose of this assignment is to enhance the student's understanding of the subsystems covered in lecture and homework assignments 2, 3, and 4. Additionally, this assignment provides a hands-on example of configuring the runtime configuration of a full autonmous vehicle software system using ROS launch files.

## Due Date and Submission
This assignment is due on Monday, December 16th at 8:00am. The grade for this assignment will be zero if not received by then because that is when final grades for the course will start being calculated and submitted.

This homework assignment does not have an automated test to verify performance. Rather, the intention of this assignment is to be more of a take-home exam. While the submission process of opening a pull request from a solution branch remains the same, a student's submission will be locked in when the pull request is opened.

## Provided Files
- Initial **homework5.launch** that must be modified to complete the assignment
- Initial **homework5.rviz** that must be configured in Rviz to display the required visualization elements

## Requirements
### 1. Configure Runtime System [75 points / 100]
The initial state of **homework5.launch** brings up the familiar Gazebo simulation with the following:

- LIDAR sensor on the front bumper
- Monocular camera in the windshield
- GPS route that generates the target trajectory for the vehicle
- Adaptive cruise control program that sends the final speed and steering commands to the simulator.

Your task is to add the following nodes to the **homework5.launch** file:

- Your Homework 2 node to follow the target trajectory
- Your Homework 3 node to segment LIDAR data to detect the traffic vehicles
- Your Homework 4 node to filter the detected traffic vehicles and estimate their relative velocity
- The monocular vision example node to detect the lane markings and project them onto the ground plane

Once set up properly, launching **homework5.launch** should make the vehicle successfully accomplish the following:

- Follow the GPS route as it did in Homework 2
- Avoid crashing into the traffic vehicles by detecting them and adjusting speed accordingly
- Detect the lane markings on the road and project them onto the ground plane for visualization in Rviz

In addition to modifying **homework5.launch**, you can make changes to your homework nodes to fulfill the above objectives.

### 2. Set up Rviz Configuration [25 points / 100]
Launching **homework5.launch** opens Rviz and loads the Rviz configuration file **homework5.rviz**. Your task is to add the following visualization components to the display:

- TF frames
- Raw point cloud from the simulated LIDAR (`sensor_msgs/PointCloud2` message)
- Raw image from the simulated camera (`sensor_msgs/Image` message)
- Detected object bounding boxes from your Homework 3 node (`jsk_recognition_msgs/BoundingBoxArray` message)
- Kalman filter tracking markers from your Homework 4 node (`visualization_msgs/MarkerArray` message)
- Detected lane markers from the monocular vision example node (`visualization_msgs/MarkerArray` message)
- Also, configure Rviz to use the vehicle's footprint frame as its fixed reference frame.

Finally, save the Rviz configuration settings to update **homework5.rviz** so that it will use the latest settings on the next run.

### 3. Submitting the Assignment
Before opening the pull request to submit the homework, be sure to:

- Test the system thoroughly by launching **homework5.launch** and make sure it behaves properly according to the specifications in Section 1
- Double-check your additions to the Rviz display and make sure they appear correctly while the simulation is running
- If you make any changes to your homework software, make sure you commit them to the **solution** branch in the corresponding repository
- Commit your changes to both **homework5.launch** and **homework5.rviz** to the **solution** branch of your Homework 5 repository

## Hints

### Modify Homework 3
The simulated LIDAR point cloud is much less dense than the point cloud from the real sensor used in Homework 3. You can account for this by reducing the number of neigboring points to search during the normal vector computation procedure. Change the argument of `setKSearch` from 50 to 20.

### Use the Tools!
Make use of the tools built into ROS to help see what is happening in the system. For example:

- Use `rqt_graph` to see how nodes and topics interact with each other.
- Use the TF tree viewer `rosrun rqt_tf_tree rqt_tf_tree` to see the structure of the TF frames of the system.
- Take advantage of the `<remap>` capability in launch files to change the names of topics that nodes subscribe or publish to without having to change or compile any code.

### Slides
Slide 4 of the System Design Overview lecture slides might be a helpful reference.