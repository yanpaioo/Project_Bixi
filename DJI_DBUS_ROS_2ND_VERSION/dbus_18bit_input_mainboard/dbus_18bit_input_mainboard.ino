/*
 This program enables the reading of DBUS 25 bits input 
 and output to DBUS 18 bit or 25 bit

  reading result = dBus.channels[0-7]

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

//the value of channel can only be 1-3 , the sequence is 01,11,10
//DBus_Output[LEFT_U] is the value of left up and down

static union
{
  uint16_t joyData[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
  byte toByte[12];
}ROS_Upload;
static union
{
  float localizationData[6]  = {0, 0, 0, 0, 0, 0};
  byte toByte[24];
}ROS_Localization_Upload;


DJI_DBUS dBus(Serial1);
int led = 13; 
uint32_t currTime, displayTime = 0;
uint8_t i;

//ros
void publish_data(void);
void publish_joy(void);
void publish_localization(void);
void readLocalizationSystem(void);
void read_joy_cmd(void);
void setup(){
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  Serial.println("DBUS Status");
  Serial2.begin(100000,SERIAL_8E1);
  dBus.begin();
  Serial3.begin(115200);
  
}

int joy_pub_count =0;
void loop(){
  
  if (dBus.Failsafe() == DBUS_SIGNAL_OK) digitalWrite(led, HIGH);
  dBus.FeedLine();
  digitalWrite(led, LOW);
  currTime = millis();
  read_joy_cmd();
  if (dBus.toChannels == 1){
    dBus.UpdateChannels();
    dBus.toChannels = 0;
    
    for(i=0;i<channel;i++){
        DBus_Output[i] = dBus.channels[i];
    }
    
  }
  
  readLocalizationSystem();
  if(displayTime < currTime) {
      displayTime = currTime + 7;
      joy_pub_count++;
      write18BitsDbusData();
      if(joy_pub_count>2){
        joy_pub_count=0;
        publish_data();
      }
      //printDBUSStatus();
  }

  
}

void read_joy_cmd( ){
  //left button up means auto
  //sequence : left  right_UD  right_LR
  static int joy_cmd_flag=0;
  static int joy_cmd_count=0;
  static union
  {
    uint16_t joyData[3]  = {1024, 1024, 1024};
    byte toByte[12];
  }joy_cmd;
  while(Serial.available()>0)   
  {
    unsigned char ch = Serial.read();
    switch(joy_cmd_flag)
    {
      case 0:
        if(ch==0xFA)
          joy_cmd_flag++;
        else
          joy_cmd_flag=0;
        break;
      case 1:
        if(ch==0xBC)
        {
          joy_cmd_count=0;
          joy_cmd_flag++;
        }
        else
          joy_cmd_flag=0;
        break;
      case 2:
        joy_cmd.toByte[i]=ch;
        joy_cmd_count++;
        if(joy_cmd_count>=6){
          joy_cmd_count=0;
  	  joy_cmd_flag++;
	}
	break;
      case 3:
	if(ch==0xDE)
	  joy_cmd_flag++;
	else
	  joy_cmd_flag=0;
	break;
      case 4:
      	ROS_Output[LEFT_LR] = joy_cmd.joyData[0];
        ROS_Output[RIGHT_UD] = joy_cmd.joyData[1];
        ROS_Output[RIGHT_LR] = joy_cmd.joyData[2];
	joy_cmd_flag=0;
        break;
			 
      default:
	joy_cmd_flag=0;
	break;		 
      }
  }

}
int test_count=0;
void publish_data(void){
  Serial.write(0xFA);
  Serial.write(0xBC);
  publish_joy();
  publish_localization();
  Serial.write(0xDE);
}
void publish_localization(void){
  for(i=0;i<8;i++){
    Serial.write(ROS_Localization_Upload.toByte[i]);
  }
  for(i=20;i<24;i++){
    Serial.write(ROS_Localization_Upload.toByte[i]);
  }

}
void publish_joy(void){
  //upload_data_display();
  for(i=0;i<12;i++){
    Serial.write(ROS_Upload.toByte[i]);
    
  }
}
float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;
void readLocalizationSystem(void){
  static union
  {
	 unsigned char data[24];
	 float ActVal[6];
  }posture;
  static unsigned char count=0;
  static unsigned char i=0;

	while(Serial3.available()>0)   
	{
		 unsigned char ch = Serial3.read();
		 switch(count)
		 {
			 case 0:
				 if(ch==0x0d)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 1:
				 if(ch==0x0a)
				 {
					 i=0;
					 count++;
				 }
				 else if(ch==0x0d);
				 else
					 count=0;
				 break;
				 
			 case 2:
				 posture.data[i]=ch;
			   	 i++;
			  	 if(i>=24)
				 {
					 i=0;
					 count++;
				 }
				 break;
				 
			 case 3:
				 if(ch==0x0a)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 4:
				 if(ch==0x0d)
				 {
      				  zangle=posture.ActVal[0];
      	  		   	  xangle=posture.ActVal[1];
      		  	   	  yangle=posture.ActVal[2];
      			    	  pos_x =posture.ActVal[3];
      			    	  pos_y =posture.ActVal[4];
      			     	  w_z   =posture.ActVal[5];
                    ROS_Localization_Upload.localizationData[0] = pos_x/1000.0;
                    ROS_Localization_Upload.localizationData[1] = pos_y/1000.0;
                    ROS_Localization_Upload.localizationData[2] = w_z;
                    ROS_Localization_Upload.localizationData[3] = xangle;
                    ROS_Localization_Upload.localizationData[4] = yangle;
                    ROS_Localization_Upload.localizationData[5] = zangle;
				 }
			         count=0;
                                //data updated
                                 return;
				 break;
			 
			 default:
				 count=0;
			   break;		 
		 } 
	 }
  return;
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
    ROS_Upload.joyData[i] = DBus_Final_Output[i];
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
void upload_data_display()
{
  for(i =0;i<6;i++){
    Serial.print(ROS_Upload.joyData[i]);
    Serial.print("\t");
  }
  Serial.println(".");
  
}
void printDBUSStatus()
{
  Serial.print("Thr ");
  Serial.print(DBus_Output[2]);
  Serial.print(" Ail ");
  Serial.print(DBus_Output[0]);
  Serial.print(" Ele ");
  Serial.print(DBus_Output[1]);
  Serial.print(" Rud ");
  Serial.print(DBus_Output[3]);
  Serial.print(" Channel1 ");
  Serial.print(DBus_Output[5]);
  Serial.print(" Channel2  ");
  Serial.print(DBus_Output[4]);
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
