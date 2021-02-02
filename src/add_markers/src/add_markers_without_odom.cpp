#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marke
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.4;

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
  }

  // Set Marker Pickup Position
  marker.pose.position.x = 4.0;
  marker.pose.position.y = 0.5;
  marker.pose.position.z = 0;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Marker Published at Pickup Zone, pause for 5 seconds ...");
  ros::Duration(5.0).sleep();

  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Marker hidden from Pickup Zone, pause for 5 seconds ...");
  ros::Duration(5.0).sleep();

  // Set Marker Drop off Position
  marker.pose.position.x = 0;
  marker.pose.position.y = -3.0;
  marker.pose.position.z = 0;

  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Marker Published at Drop off Zone, pause for 5 seconds ...");
  ros::Duration(5.0).sleep();

  ROS_INFO("Exiting ...");
  return 0;
}
