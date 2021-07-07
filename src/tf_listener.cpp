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



using namespace std;
using namespace ros;

tf::TransformListener *tf_listener;
tf::StampedTransform robot_transform;



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
    tf::Quaternion q = robot_transform.getRotation();
    tf::Vector3 v = robot_transform.getOrigin();
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

  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf_listener = new tf::TransformListener();

  while (ros::ok())
  {
    ros::spinOnce();
    listener();
  }
 
}

