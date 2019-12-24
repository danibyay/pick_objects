#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pick_up_goal;
  move_base_msgs::MoveBaseGoal drop_off_goal;

  // set up the frame parameters
  pick_up_goal.target_pose.header.frame_id = "map";
  pick_up_goal.target_pose.header.stamp = ros::Time::now();
  drop_off_goal.target_pose.header.frame_id = "map";
  drop_off_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pick_up_goal.target_pose.pose.position.x = 1.0;
  pick_up_goal.target_pose.pose.orientation.w = 1.0;
  drop_off_goal.target_pose.pose.position.x = -0.31;
  drop_off_goal.target_pose.pose.position.y = -0.22;
  drop_off_goal.target_pose.pose.orientation.z = 0.99;
  drop_off_goal.target_pose.pose.orientation.w = 0.009;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick_up_goal");
  ac.sendGoal(pick_up_goal);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved to the pick up goal");
  }
  else {
    ROS_INFO("The base failed to move to the pick up goal");
    return 0;
  }

  // wait for 5 seconds
  ros::Duration(5.0).sleep();

  ROS_INFO("Sending drop_off_goal");
  ac.sendGoal(drop_off_goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved to the drop off goal");
  }
  else {
    ROS_INFO("The base failed to move to the drop off goal");
    return 0;
  }

  return 0;
}
