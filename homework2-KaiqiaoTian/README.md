# ECE 6460 Homework #2
The goal of this assignment is to re-implement the trajectory following module from the reference design that was run for Homework #1.

The purpose of this assignment is to:

- Become familiar with how to send actuation commands to the simulation of the Dataspeed ADAS Kit CAN interface
- Practice converting latitude and longitude into Cartesian coordinates for navigation

## Due Date
This assignment is due on Saturday, October 5th at midnight.

## Provided Files
After accepting the assignment on GitHub Classroom, the student's assignment repository will be initialized with the following files:

- Starting point of  **package.xml** and **CMakeLists.txt** files for the **homework2** package.
- Template C++ files to be modified as part of the assignment:
	- src/homework2_node.cpp
	- src/Homework2.cpp
	- src/Homework2.hpp
- Dynamic parameter configuration file
	- cfg/Homework2.cfg
- Simulation sceneario config file
	- yaml/homework2_scenario.yaml
- Test configuration files in the **tests** folder of the **homework2** package

## Requirements
The **homework2** node must:

- Subscribe to the `/vehicle/perfect_gps/enhanced_fix` topic, which is of type `gps_common/GPSFix`
	- Messages on this topic contain the vehicle's position in latitude and longitude, as well as its heading in the North-East-Down reference frame.
	- Messages are published to this topic at a rate of 50 Hz
- Subscribe to the `/final_waypoints` topic, which is of type `autoware_msgs/Lane`
	- The `waypoints` field of messages on this topic contain an array of waypoints that represent the upcoming section of the desired route
	- Each waypoint is represented in raw UTM coordinates
	- Messages are published to this topic at a rate of 1 Hz
- Advertise the `/vehicle/ulc_cmd` topic with type `dataspeed_ulc_msgs/UlcCmd`
	- Messages must be published on this topic at a minimum rate of 50Hz
- Implement a control algorithm to follow the incoming waypoint trajectory with a maximum lateral offset of 30cm

## Hints

- A simple but effective way to steer along a path is to select one of the waypoints and input its coordinates into the control algorithm
- The selection of the target point has a large effect on the resulting path following performance
- It is highly recommended to compute the waypoint positions relative to the vehicle's reference frame whenever a new GPS fix message is received

## Curvature-Based Algorithm

![](./diagram.png) 

The turning radius to hit the target point can be computed in terms of the target point's coordinates in the vehicle's reference frame:

![](./radius_calculation.png) 

To avoid numerical problems for small values of y_t, and because the ULC steering control interface is curvature based, it is therefore better to compute curvature instead of turning radius:

![](./curvature_calculation.png) 