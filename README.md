# Robotic-Home-Assistant

Localization             |  Pick up
:-------------------------:|:-------------------------:
![preplan](/preplan.png)  |  ![pickup](/pickup.png)
Delivery - In Transit | Drop off
![deliver](/delivery.png)  |  ![dropoff](/dropoff.png)

This project is the final project for Udacity's Robotics Software Engineer Nanodegree program. The goal was to create a map of my custom environment using SLAM, then use that map to plan a trajectory to pick up an object and place it somewhere else within the environment. The following are the steps I took and the ROS packages I used.

## Step 1: Setup the environment

For this project I'm using ROS version Kinetic on a Lubuntu virtual machine. The following necessary components can be downloaded from my github repository, but this is where I got them from.

First I downloaded the turtlebot_gazebo folder from the https://github.com/turtlebot/turtlebot_simulator.git (branch: indigo) repository. I kept the whole repository, but I primarily used the turtlebot_gazebo folder.

Then I've downloaded and kept the turtlebot_teleop folder from https://github.com/turtlebot/turtlebot.git (branch: kinetic). I've used this to drive the turtlebot around while mapping the world or while testing the environment.

I've also downloaded the turtlebot_rviz_launchers folder from https://github.com/turtlebot/turtlebot_interactions.git (branch: indigo). This folder is for convience because it opens RVIZ for you with everything configured for the turtlebot and its environment. 

The rest of the components I've created myself such as the pick_objects and add_markers catkin packages.

## Step 2: Create a Map

The turtlebot uses a RGB-D camera such as the X-box Kinect to perform a laser based SLAM (Simultaneous Localization and Mapping) which transforms the laser data into the robot's odometry tf-frame. From this data you can create an occupancy gridmap or floorplan.

I used the gslam that comes with the turtlebot_gazebo repository, but a repository that contains a ROS wrapper for OpenSLAM's Gmapping is also at https://github.com/ros-perception/slam_gmapping.git (branch: hydro-devel).

You can run the shell script located at Robotic-Home-Assistant/catkin_ws/src/scripts/test_slam.sh to open the environment, drive around and perform slam. After you created a satisfactory map, check in RVIZ, then you can run `rosrun map_server map_saver` in another terminal to save the map.

## Step 3: Include the Map and Plan the Trajectories

Once I saved the map files in the Robotic-Home-Assistant/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/maps/ folder, I was able to use the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan the robot trajectory from start to goal position.

I've included the test_navigation.sh script in which you can see the map and manually giving the robot destination's with RVIZ's 2d destination picker.

## Step 4: Create and Run the Home Assistant Script

I've created pick_objects and add_markers catkin packages to simulate a home assistant that travels to a destination, picks up an object, and drops off the object at another destination. Run the script home_service.sh to see it.

