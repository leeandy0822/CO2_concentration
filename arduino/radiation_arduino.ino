#include "RadiationWatch.h"

#include <ros.h>
#include <std_msgs/Float32.h>


// ROS node handler
ros::NodeHandle nh;

// ROS topics object definitions
std_msgs::Float32 rad_msg;
ros::Publisher pub_rad("Radiation_arduino", &rad_msg);



RadiationWatch radiationWatch;
float value;
void onRadiation()
{
//  Serial.println("A wild gamma ray appeared");

  Serial.print(radiationWatch.uSvh());
  value = radiationWatch.uSvh();
  
  rad_msg.data = float(radiationWatch.uSvh());
  
  pub_rad.publish(&rad_msg);
  
  nh.spinOnce();
  delay(200);
  
}


void onNoise()
{
  Serial.println("Argh, noise, please stop moving");
}


void setup()
{
  Serial.begin(9600);
  //SERIAL.println("SCD30 Raw Data");
  // Setup ROS
  nh.initNode();
  nh.advertise(pub_rad);
  radiationWatch.setup();
  
  // Register the callbacks.
  radiationWatch.registerRadiationCallback(&onRadiation);
  radiationWatch.registerNoiseCallback(&onNoise);
}

void loop()
{
  radiationWatch.loop();
}
