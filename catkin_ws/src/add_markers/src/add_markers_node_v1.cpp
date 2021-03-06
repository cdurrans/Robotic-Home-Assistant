#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

float odom_x = 0.0;
float odom_y = 0.0;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ::odom_x = msg->pose.pose.position.x;
    ::odom_y = msg->pose.pose.position.y;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odometry_sub = n.subscribe("/odom", 200, odom_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  int number_goals = 2;
  float goal_targets[number_goals][3] = {{-1.0, 3.96396, -1.0},{0.44319102, -2.86513328, -1.0}};
  int pick_up_goal = 0;
  int drop_off_goal = 1;
  int current_goal = pick_up_goal;
  int goal_tolerance = 0.7;
  bool has_marker = false;
  
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    bool x_bool = false;
    bool y_bool = false;
    ROS_INFO("Odom data: %f, %f", odom_x, odom_y);

    if (fabs(odom_x - goal_targets[current_goal][0]) < goal_tolerance) 
    {
      x_bool = true;
      ROS_INFO("x_bool is true.");
    }
    if (fabs(odom_y - goal_targets[current_goal][1]) < goal_tolerance)
    {
      y_bool = true;
      ROS_INFO("y_bool is true.");
    }
    if (x_bool == true && y_bool == true)
    {
      if (current_goal == pick_up_goal) 
      {
        current_goal = drop_off_goal;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        has_marker = true;
      }
      else 
      {
        current_goal = pick_up_goal;
      }
    }

    if (!has_marker) 
    {
      marker.pose.position.x = goal_targets[current_goal][0];
      marker.pose.position.y = goal_targets[current_goal][1];
      marker.pose.position.z = goal_targets[current_goal][2];
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }
    // Cycle between different shapes
    ros::spinOnce();

    r.sleep();

  }
}
