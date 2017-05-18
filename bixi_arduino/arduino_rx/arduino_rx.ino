#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 msg;

ros::Publisher to_ros_chatter("to_ros_chatter", &msg);


void setup()
{
  
  nh.initNode();
  nh.advertise(to_ros_chatter);
  Serial3.begin(9600);
}

void loop()
{
  if (Serial3.available() > 0)
  {
    msg.data = int(Serial3.read());
    if (msg.data != 0) to_ros_chatter.publish(&msg);
    
  }
 nh.spinOnce();
}

