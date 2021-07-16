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
float color[3] = {1,1,1};
int marker_id = 0;
int height = 0;

void listener(){
  // use tf_listener to get the transformation from camera_link to tag 0

  string child_id = "base_link";
  string parent_id = "map";

  tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(), ros::Duration(0.04));
  try {

    tf_listener->lookupTransform(parent_id, child_id, ros::Time(0), robot_transform);
    // cout << "Frame id:" << robot_transform.frame_id_ << ", Child id:" << robot_transform.child_frame_id_ << endl;
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

// turn co2 concentration more smoothly
void HSVtoRGB(float H){
    float s = 1;
    float v = 1;
    float C = s*v;
    float X = C*(1-abs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }
    color[0] = (r+m);
    color[1] = (g+m);
    color[2] = (b+m);
}


void co2_callback(const std_msgs::Float32::ConstPtr& msg){
    listener();

    float data = msg->data;
    float concentration = (data-0)/0.2*100;
    if (concentration > 100){
        concentration = 100;
    }
    cout << "CO2: " << data<<" ppm" <<endl;
    HSVtoRGB(100-concentration);
}


int main(int argc, char** argv){

  ros::init(argc, argv, "co2_marker");
  ros::NodeHandle n;

  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("Radiation_arduino", 1, co2_callback);

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
    marker.pose.position.z = height;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.5;

    // size
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.001;
    // color
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0);

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
    
    if (marker_id%20 == 0){
        height += 0.05;
    }
  }
 
}

