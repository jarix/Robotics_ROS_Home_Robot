#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for Pickup zone (goal #1)
  goal.target_pose.pose.position.x = 4.0;
  goal.target_pose.pose.position.y = 0.5;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal #1 to Pickup zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal #1
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The Robot successfully reached Pickup zone (goal #1)");
  } else {
    ROS_WARN("The Robot failed to reach goal Pickup zone (goal #1)");
  }

  //-----------------------------------------------------------

  ROS_INFO("Pausing for 5 seconds ...");
  ros::Duration(5.0).sleep();
  ROS_INFO("Proceeding to Drop-off zone (goal #2)");

  // Define a position and orientation for Drop-off zone (goal #2)
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = -3.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal #2 to Drop off zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal #2
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The Robot successfully reached Drop off zone (goal #2)");
  } else {
    ROS_WARN("The Robot failed to reach goal Drop off zone (goal #2)");
  }

  return 0;
}