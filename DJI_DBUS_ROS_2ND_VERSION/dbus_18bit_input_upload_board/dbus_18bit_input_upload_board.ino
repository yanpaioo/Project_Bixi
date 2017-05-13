/*
 This program enables the reading of DBUS 25 bits input 
 and output to DBUS 18 bit or 25 bit

  reading result = dBus.channels[0-7]

*/
/*
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
*/
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

static uint16_t ROS_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
int32_t ROS_Upload[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
float ROS_Localization_Upload[6]  = {0, 0, 0, 0, 0, 0};
//the value of channel can only be 1-3 , the sequence is 01,11,10
//DBus_Output[LEFT_U] is the value of left up and down

static union
{
  uint16_t joyData[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
  byte toByte[12];
}joy_read;
static union
{
  float localizationData[3]  = {0, 0, 0};
  byte toByte[12];
}localization_read;



int led = 13; 
uint32_t currTime, displayTime = 0;
uint8_t i;
/*
//ros
ros::NodeHandle nh;
void joy_cb( const sensor_msgs::Joy& joy);
void publish_joy(void);
void readLocalizationSystem(void);
sensor_msgs::Joy joy_msg;
ros::Subscriber<sensor_msgs::Joy> sub_joy("/joy_cmd", joy_cb);
ros::Publisher pub_joy( "/joy_msg", &joy_msg);
char frameid[] = "/joy_msg";
*/

void read_data(void);
void publish_data(void);

void setup(){
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
//  Serial.println("DBUS Status");

  /*
  //ros
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_joy);
  nh.subscribe(sub_joy);
  joy_msg.header.frame_id =  frameid;
  joy_msg.buttons_length=channel;
  //joy_msg.axes_length=6;

  */
}


void loop(){
  
  digitalWrite(led, LOW);
  currTime = millis();

  
  read_data();
  if(displayTime < currTime) {
      displayTime = currTime + 20;
      /*publish_data();
      nh.spinOnce();*/
      //printDBUSStatus();
  }

  
}
/*
void joy_cb( const sensor_msgs::Joy& joy){
  //left button up means auto
  //sequence : left  right_UD  right_LR
    ROS_Output[LEFT_LR] = (uint16_t)(joy.buttons[0]);
    ROS_Output[RIGHT_UD] = (uint16_t)(joy.buttons[1]);
    ROS_Output[RIGHT_LR] = (uint16_t)(joy.buttons[2]);

}
void publish_joy(void){
  //upload_data_display();
  joy_msg.header.stamp = nh.now();
  joy_msg.buttons = ROS_Upload;
  //joy_msg.axes = ROS_Localization_Upload;
  pub_joy.publish(&joy_msg);
  
}*/
int count=0;
void read_data(void){
  static int read_data_count =0;
  while(Serial1.available()>0){
     unsigned char ch = Serial1.read();
     switch(read_data_count)
     {
       case 0:
         if(ch==0xFA)
           read_data_count++;
         else
           read_data_count=0;
         break;
         
       case 1:
         if(ch==0xBC)
         {
           count=0;
           read_data_count++;
         }else
           read_data_count=0;
         break;
       case 2:
        if(count < 12){
          joy_read.toByte[i]=(byte)ch;
        }else{
          localization_read.toByte[count-12] =(byte)ch;    
        }
        count++;
        if(count>=24){
           count=0;
           read_data_count++;
        }
       break;
         
       case 3:
         if(ch==0xDE){
           read_data_count=0;
           for(i=0;i<6;i++){
            ROS_Upload[i] = joy_read.joyData[i];
           }
           for(i=0;i<3;i++){
            ROS_Localization_Upload[i] = localization_read.localizationData[i];
           }
           printDBUSStatus();
         }else
           read_data_count=0;
         break;
       default:
         read_data_count=0;
         break;    
     } 
   }
}
void printDBUSStatus()
{
  for(i=0;i<12;i++){
   Serial.write(joy_read.toByte[i]);
  }
  for(i=0;i<12;i++){
     Serial.write(localization_read.toByte[i]);
  }
   
  /*
  Serial.print("Thr ");
  Serial.print(ROS_Upload[2]);
  Serial.print(" Ail ");
  Serial.print(ROS_Upload[0]);
  Serial.print(" Ele ");
  Serial.print(ROS_Upload[1]);
  Serial.print(" Rud ");
  Serial.print(ROS_Upload[3]);
  Serial.print(" Channel1 ");
  Serial.print(ROS_Upload[5]);
  Serial.print(" Channel2  ");
  Serial.print(ROS_Upload[4]);
  Serial.print(" Stat ");
  Serial.println(".");
  */
}
