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
#include "std_msgs/Float32.h"


using namespace std;
using namespace ros;

tf::TransformListener *tf_listener;
tf::StampedTransform robot_transform;
tf::Quaternion q;
tf::Vector3 v;
float color[3] = {0,0,0};
int marker_id = 0;


void listener(){
  // use tf_listener to get the transformation from camera_link to tag 0

  string child_id = "base_link";
  string parent_id = "map";

  tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(), ros::Duration(0.04));
  try {

    tf_listener->lookupTransform(parent_id, child_id, ros::Time(0), robot_transform);
    cout << "Frame id:" << robot_transform.frame_id_ << ", Child id:" << robot_transform.child_frame_id_ << endl;
    double yaw, pitch, roll;
    robot_transform.getBasis().getRPY(roll, pitch, yaw);
    q = robot_transform.getRotation();
    v = robot_transform.getOrigin();
    // std::cout << "- Robot Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    // std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
    //           << q.getZ() << ", " << q.getW() << "]" << std::endl;
    // 
    }
    catch (tf::TransformException& ex)
    {
      std::cout << "Exception thrown:" << ex.what() << std::endl;
    }
}

void co2_callback(const std_msgs::Float32::ConstPtr& msg){
    listener();

    float data = msg->data;
    if (data >10000){
      data = 10000;
    }
    float percent = data/10000;
    cout << "CO2:" << percent<<endl;
    if( percent <= 0.5){
      color[0]= 0.5+percent;
      color[1]= 1.0;
      color[2]= 0.4;
    }
    if (percent > 0.5){
      color[0]= 1;
      color[1]= 0.8-percent*0.3;
      color[2]= 0.6;
    }
}

int main(int argc, char** argv){

  ros::init(argc, argv, "co2_marker");
  ros::NodeHandle n;

  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("CO2_ppm", 1, co2_callback);

  uint32_t shape = visualization_msgs::Marker::SPHERE;

  tf_listener = new tf::TransformListener();
  




  while (ros::ok())
  {
    ros::spinOnce();
    listener();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
  
    marker.ns = "basic_shapes";
    marker.id = marker_id;
    marker.type = shape;

    // position
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = v.getX();
    marker.pose.position.y = v.getY();
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.5;

    // size
    marker.scale.x = 2.5;
    marker.scale.y = 2.5;
    marker.scale.z = 0.01;
    // color
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(1000);

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
    marker_id ++;


  }
 
}

