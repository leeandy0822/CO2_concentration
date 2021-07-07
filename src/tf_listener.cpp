#include <stdlib.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>



using namespace std;
using namespace ros;

tf::TransformListener *tf_listener;
tf::StampedTransform robot_transform;
tf::Quaternion q;
tf::Vector3 v;


void listener(){
  // use tf_listener to get the transformation from camera_link to tag 0

  string child_id = "base_link";
  string parent_id = "map";

  tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(), ros::Duration(0.07));
  try {

    tf_listener->lookupTransform(parent_id, child_id, ros::Time(0), robot_transform);
    cout << "Frame id:" << robot_transform.frame_id_ << ", Child id:" << robot_transform.child_frame_id_ << endl;
    double yaw, pitch, roll;
    robot_transform.getBasis().getRPY(roll, pitch, yaw);
    q = robot_transform.getRotation();
    v = robot_transform.getOrigin();
    std::cout << "- Robot Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
              << q.getZ() << ", " << q.getW() << "]" << std::endl;


    
    }

    catch (tf::TransformException& ex)
    {
      std::cout << "Exception thrown:" << ex.what() << std::endl;
    }
    
}



int main(int argc, char** argv){

  ros::init(argc, argv, "co2_concentration");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  uint32_t shape = visualization_msgs::Marker::CUBE;

  tf_listener = new tf::TransformListener();


  while (ros::ok())
  {
    listener();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "my_frame";
    marker.header.stamp = ros::Time::now();
  
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = v.getX();
    marker.pose.position.y = v.getY();
    marker.pose.position.z = v.getZ();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    r.sleep();


  }
 
}

