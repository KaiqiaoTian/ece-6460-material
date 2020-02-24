# ECE 6460 Homework #1
The goal of this assignment is to subscribe to the ground-truth velocity data coming from a Gazebo simulation of an autonmous vehicle, copy the forward speed data and re-publish it on a different ROS topic.

The purpose of this assignment is to:

- Ensure the student's PC is set up properly for the course
- Introduce writing simple ROS nodes
- Introduce the workflow procedure of submitting homework assignments through GitHub Classroom

## Provided Files
After accepting the assignment on GitHub Classroom, the student's assignment repository will be initialized with the following files:

- Pre-configured **package.xml** and **CMakeLists.txt** files for the **homework1** package
- Template C++ files to be modified as part of the assignment:
	- src/homework1_node.cpp
	- src/Homework1.cpp
	- src/Homework1.hpp
- Test configuration files in the **tests** folder of the **homework1** package

## Step 1: Configure Git Cola and Clone this Repository

#### 1.1 Clone the Repository
Open Git Cola and click **Clone** on the main screen. Copy the URL of this repository into the box and click **OK**. Open the **ros** folder in your home directory, then select the **src** folder to clone the repository here. After clicking **Open**, type in your GitHub credentials to clone the repository.

#### 1.2 Open the Repository
Click **Open** on the main Git Cola screen, go to the **src** folder, select the folder containing the cloned repository and click **Open**.

#### 1.3 Configure Git Commit Preferences
With an open repository, click **File --> Preferences**. In the **All Repositories** tab, type your name in the **User Name** box and the email tied to your GitHub account in the **Email Address** box. When you make commits to a repository, these settings will tie your commits to your GitHub account.

## Step 2: Modify Starter ROS Node Files
Modify **Homework1.cpp** and **Homework1.hpp** to implement the following requirements:

- Subscribe to the **/vehicle/twist** topic, which is of type **geometry_msgs/TwistStamped**
- Advertise the **/vehicle_speed** topic with type **std_msgs/Float64**
- Whenever a **geometry_msgs/TwistStamped** message is received on the **/vehicle/twist** topic, copy the vehicle's forward speed into a **std_msgs/Float64** message and publish it on the **/vehicle_speed** topic.

## Step 3: Compile and Test Manually
#### 3.1 Run catkin_make
After making the required changes to the starting ROS node source files, compile the ROS workspace using the `release.bash` script:

```
cd ~/ros
./release.bash
```
#### 3.2 Launch the System
Provided the compiling process finished successfully, proceed to manually test the modified ROS node by starting up the simulator and the **homework1** node. First, start a **roscore** in its own terminal window / tab:
```
roscore
```

In a new terminal window, launch the simulator and the **homework1** node at the same time:
```
roslaunch homework1 homework1.launch
```
#### 3.3 View ROS Topic Contents
Look at the `/vehicle/twist` topic by opening a new terminal window and using `rostopic echo`:

```
rostopic echo /vehicle/twist
```

Make sure the `linear.x` component of the twist message matches the `/vehicle_speed` topic by opening another terminal window and echoing it:

```
rostopic echo /vehicle_speed
```
After verifying that your node modifications are working as they should, stop the simulation before moving on to Step 3.

## Step 4: Run the Automated Tests Locally
To run the same tests that will be used as part of the automatic code testing system, run the `tests.bash` script:

```
cd ~/ros
./tests.bash
```

All of the automated tests should run automatically, and if everything is working correctly, the result should show no test failures.

## Step 5: Commit and Push Changes for Automatic Review
Once you verify that the automated tests pass on your local machine, you should be ready to push the changes to GitHub for official grading.

Using Git Cola:

- Create a new branch called 'solution'
- Commit all changes to the 'solution' branch with a meaningful message
- Push the 'solution' branch to GitHub

After pushing the changes, you can verify that the automatic code check passed by looking at the commit list of the 'solution' branch on your assignment repository on GitHub.

## Step 6: Open a Pull Request
After making sure the automated code check passed on your commit, you can officially submit the assignment by opening a pull request on GitHub to merge your 'solution' branch into the 'master' branch of your assignment repository.