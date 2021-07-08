#include <ArduinoTcpHardware.h>
#include <ArduinoHardware.h>
#include <ros.h>

#include "SCD30.h"
#include <ros.h>
#include <std_msgs/Float32.h>


// ROS node handler
ros::NodeHandle nh;

// ROS topics object definitions
std_msgs::Float32 CO2_msg;
ros::Publisher pub_CO2("CO2_concentration", &CO2_msg);
  
void setup()
{
    Wire.begin();
    Serial.begin(115200);
    //SERIAL.println("SCD30 Raw Data");
    scd30.initialize();

    // Setup ROS
    nh.initNode();
    nh.advertise(pub_CO2);
}

void loop()
{

  float result[3] = {0};  

  if(scd30.isAvailable())
  {
    scd30.getCarbonDioxideConcentration(result);
    //SERIAL.print("Carbon Dioxide Concentration is: ");
    Serial.println(result[0]);
/*    SERIAL.println(" ppm");
    SERIAL.println(" ");
    SERIAL.print("Temperature = ");
    SERIAL.print(result[1]);
    SERIAL.println(" ℃");
    SERIAL.println(" ");
    SERIAL.print("Humidity = ");
    SERIAL.print(result[2]);
    SERIAL.println(" %");
    SERIAL.println(" ");
    SERIAL.println(" ");
    SERIAL.println(" ");*/
    // Publish CO2 concentration
    CO2_msg.data = result[0];
    pub_CO2.publish(&CO2_msg);
  }    

  nh.spinOnce();
  delay(100);
}
