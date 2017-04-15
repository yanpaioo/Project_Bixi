/*
 This program enables the reading of DBUS 25 bits input 
 and output to DBUS 18 bit or 25 bit

  reading result = dBus.channels[0-7]

*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include "DJI_DBUS.h"

//there are two versions of DJI controller protocol
// one is the latestest firmware with 25 bits with 7 channel
//another is the old one with 6 channel of 18 bits
//please use America coding methods
/*
 * *
for each data 
mid value = bin 10000000000   (1024)   0x400
max value = bin 11010010100   (1684)   0x694
max value = bin 00101101100   (364)   0x16C
 */
void write18BitsDbusData();
boolean running = false;
const byte channel = 6;
//output format

//channel define
#define LEFT_UD 3
#define LEFT_LR 2
#define RIGHT_UD 1
#define RIGHT_LR 0
#define CHANNEL_L 5
#define CHANNEL_R 4
#define CHANNEL_UP 1
#define CHANNEL_MID 3
#define CHANNEL_DOWN 2
static uint16_t DBus_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_UP};
static uint16_t ROS_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
static uint16_t DBus_Final_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
static int32_t ROS_Upload[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
//the value of channel can only be 1-3 , the sequence is 01,11,10
//DBus_Output[LEFT_U] is the value of left up and down


DJI_DBUS dBus(Serial1);
int led = 13; 
uint32_t currTime, displayTime = 0;
uint8_t i;

//ros
ros::NodeHandle  nh;
void joy_cb( const sensor_msgs::Joy& joy);
sensor_msgs::Joy joy_msg;
ros::Subscriber<sensor_msgs::Joy> sub_joy("/joy_cmd", joy_cb);
ros::Publisher pub_joy( "/joy_msg", &joy_msg);
char frameid[] = "/joy_msg";


void setup(){
  pinMode(led, OUTPUT);
//  Serial.println("DBUS Status");
  Serial2.begin(100000,SERIAL_8E1);
  dBus.begin();
  //ros
  nh.initNode();
  nh.advertise(pub_joy);
  nh.subscribe(sub_joy);
  joy_msg.header.frame_id =  frameid;
  joy_msg.buttons_length=channel;
}

void loop(){
  
  if (dBus.Failsafe() == DBUS_SIGNAL_OK) digitalWrite(led, HIGH);
  dBus.FeedLine();
  digitalWrite(led, LOW);
  currTime = millis();

  if (dBus.toChannels == 1){
    dBus.UpdateChannels();
    dBus.toChannels = 0;
    
    for(i=0;i<channel;i++){
        DBus_Output[i] = dBus.channels[i];
    }
    
  }
  
  if(displayTime < currTime) {
      displayTime = currTime + 7;
      publish_joy();
      write18BitsDbusData();
      nh.spinOnce();
  }

  
}

void joy_cb( const sensor_msgs::Joy& joy){
  //left button up means auto
  //sequence : left  right_UD  right_LR
    ROS_Output[LEFT_LR] = (uint16_t)(joy.buttons[0]);
    ROS_Output[RIGHT_UD] = (uint16_t)(joy.buttons[1]);
    ROS_Output[RIGHT_LR] = (uint16_t)(joy.buttons[2]);
    //publish_joy();
}
void publish_joy(){
  joy_msg.header.stamp = nh.now();
  joy_msg.buttons=ROS_Upload;
  pub_joy.publish(&joy_msg);
  
}
void printDBUSStatus()
{
  Serial.print("Thr ");
  Serial.print(dBus.channels[2]);
  Serial.print(" Ail ");
  Serial.print(dBus.channels[0]);
  Serial.print(" Ele ");
  Serial.print(dBus.channels[1]);
  Serial.print(" Rud ");
  Serial.print(dBus.channels[3]);
  Serial.print(" Channel1 ");
  Serial.print(dBus.channels[5]);
  Serial.print(" Channel2  ");
  Serial.print(dBus.channels[4]);
  Serial.print(" Stat ");
  if (dBus.Failsafe() == DBUS_SIGNAL_FAILSAFE) {
    Serial.print("FailSafe");
  } else if (dBus.Failsafe() == DBUS_SIGNAL_LOST) {
    Serial.print("Signal Lost");
  } else if (dBus.Failsafe() == DBUS_SIGNAL_OK) {
    Serial.print("OK");
  }
  Serial.println(".");
  
}

void write18BitsDbusData(){
  //05 04 20 00 01 D8 00 00 00 00 00 00 00 00 00 00 00 00 original 
  //00 04 20 00 01 58 00 00 00 00 00 00 00 00 00 00 00 00 adjusted
      
  
  
  DBus_Final_Output[LEFT_UD] = DBus_Output[LEFT_UD];
  DBus_Final_Output[CHANNEL_R] = DBus_Output[CHANNEL_R];
  DBus_Final_Output[CHANNEL_L] = DBus_Output[CHANNEL_L];
  if(DBus_Output[CHANNEL_L] == CHANNEL_UP){
    //auto mode
    DBus_Final_Output[LEFT_LR] = ROS_Output[LEFT_LR];
    DBus_Final_Output[RIGHT_UD] = ROS_Output[RIGHT_UD];
    DBus_Final_Output[RIGHT_LR] = ROS_Output[RIGHT_LR];
  }else{
    DBus_Final_Output[LEFT_LR] = DBus_Output[LEFT_LR];
    DBus_Final_Output[RIGHT_UD] = DBus_Output[RIGHT_UD];
    DBus_Final_Output[RIGHT_LR] = DBus_Output[RIGHT_LR];
  }
  
  for(i = 0;i<channel;i++){
    ROS_Upload[i] = DBus_Final_Output[i];
  }


  Serial2.write((uint8_t) ( ((DBus_Final_Output[0]&0x00FF)>>0) ) );//data1 0-7
  Serial2.write((uint8_t) ( ((DBus_Final_Output[0]&0x0700)>>8) | ((DBus_Final_Output[1]&0x001F)<<3) ) );//data1 8-10 data2 0-4 
  Serial2.write((uint8_t) ( ((DBus_Final_Output[1]&0x07E0)>>5) | ((DBus_Final_Output[2]&0x0003)<<6) ) );// data2 5-10 data3 0-1
  Serial2.write((uint8_t) ( ((DBus_Final_Output[2]&0x03FC)>>2) ) );// data3 2-9
  Serial2.write((uint8_t) ( ((DBus_Final_Output[2]&0x0400)>>10) | ((DBus_Final_Output[3]&0x007F)<<1) ) );// data3 10 data 4 0-6
  Serial2.write((uint8_t) ( ((DBus_Final_Output[3]&0x0780)>>7) | ((DBus_Final_Output[4]&0x0003)<<4) | ((DBus_Final_Output[5]&0x0003)<<6) ) );// data3 10 data 4 0-6
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
}

