#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>


// Globals
static float g_fPickUp_x = 4.0;
static float g_fPickUp_y = 0.5;
static float g_fDropOff_x = 0.0;
static float g_fDropOff_y = -3.0;
static bool g_bInPickUpZone = false;
static bool g_bInDropOffZone = false;

// Odometry Callback
void callback_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
  double x_pos = msg->pose.pose.position.x;
  double y_pos = msg->pose.pose.position.y;

  // Check Euclidian Distance to the pick up point
  float pickUpDist = sqrt(pow((x_pos-g_fPickUp_x), 2) + pow((y_pos - g_fPickUp_y), 2) );  
  //ROS_INFO("Pickup Distane x=%f, y=%f -> d=%f", x_pos, y_pos, pickUpDist);
  if (std::abs(pickUpDist) < 0.7) {
    //ROS_WARN_ONCE("Robot arrived in Pickup Zone ...");
    ROS_INFO_STREAM_ONCE("Robot arrived in Pickup Zone ...");
    g_bInPickUpZone = true;
  }

  // Check Euclidian Distance to the drop off point
  float dropOffDist = sqrt(pow((x_pos-g_fDropOff_x), 2) + pow((y_pos - g_fDropOff_y), 2) );  
  //ROS_INFO("Pickup Distane x=%f, y=%f -> d=%f", x_pos, y_pos, dropOffDist);
  if (std::abs(dropOffDist) < 0.4) {
    ROS_INFO_STREAM_ONCE("Robot arrived in Drop Off Zone ...");
    g_bInDropOffZone = true;
  }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(2);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscribe to robot odometry 
  ros::Subscriber odometry_sub = n.subscribe("/odom", 100, callback_odometry);

#if 1

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Instantiate Visualization Marker
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  //marker.header.frame_id = "/my_frame";
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the object.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the object
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      ROS_WARN("ROS returned not ok while waiting for a subcriber");
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  if (!ros::ok()) {
    ROS_WARN("ROS returned not ok after getting a subcriber");
    return 0;
  } else {
    ROS_INFO("add_marker subscribed ...");
  }

  // Set Marker Pickup Position
  marker.pose.position.x = g_fPickUp_x;
  marker.pose.position.y = g_fPickUp_y;
  marker.pose.position.z = 0;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Object located at Pickup Zone ...");

  bool bObjectPickedUp = false;
  bool bObjectDroppedOff = false;

  while(ros::ok())
  {
    // Robot arrived to pickup zone but has not yet picked up the object
    if (g_bInPickUpZone && !bObjectPickedUp) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ROS_INFO("Object picked up from Pickup Zone ...");
      bObjectPickedUp = true;
      ros::Duration(5.0).sleep();
    }

    // Robot arrived to drop off zone but has not yet dropped off the object
    if (g_bInDropOffZone && !bObjectDroppedOff) {
      // Set Marker Drop off Position
      marker.pose.position.x = g_fDropOff_x;
      marker.pose.position.y = g_fDropOff_y;
      marker.pose.position.z = 0;
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
      ROS_INFO("Object dropped off at the Drop Off Zone ...");
      bObjectDroppedOff = true;
      ros::Duration(5.0).sleep();
    }
    ros::spinOnce();
    r.sleep();
  }

#else

  ros::spin();

#endif
}
