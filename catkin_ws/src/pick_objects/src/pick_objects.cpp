#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  float goal_targets[2][3] = {{-1.0, 3.96396, -1.0},{0.44319102, -2.86513328, -1.0}};

  int number_goals = 2;

  for (int i = 0; i < number_goals; i++)
  {
    goal.target_pose.pose.position.x = goal_targets[i][0];
    goal.target_pose.pose.position.y = goal_targets[i][1];
    goal.target_pose.pose.orientation.w = goal_targets[i][2];

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("The robot has arrived at its destination.");
      ros::Duration(5).sleep();
    }
  else{
    ROS_INFO("The robot has not arrived at its destination.");
    }
  }
  
  return 0;
}

