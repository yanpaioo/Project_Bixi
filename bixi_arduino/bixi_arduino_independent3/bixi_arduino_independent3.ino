#include <ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

// SBT dual motor driver
#define SBT_MOTOR1_FULL_FORWARD 127
#define SBT_MOTOR1_FULL_REVERSE 1
#define SBT_MOTOR2_FULL_FORWARD 255
#define SBT_MOTOR2_FULL_REVERSE 128
#define SBT_ALL_STOP  0

// ir pins
#define IR_FRONT_LEFT   A0
#define IR_FRONT_RIGHT  A1
#define IR_SIDE_LEFT    A2
#define IR_SIDE_RIGHT   A3

// limit switch pins
#define LS_STEPPER_TOP  2
#define LS_STEPPER_BOT  3
#define LS_BOX_LEFT     4
#define LS_BOX_RIGHT    5
#define LS_RAIL_TOP     6
#define LS_RAIL_BOT     7

// stepper pins
#define PUp             22
#define PUn             24
#define DRp             26
#define DRn             28
#define MFp             30
#define MFn             32
#define PULSE_HIGH_TIME 100 // in ms
#define PULSE_LOW_TIME  500-PULSE_HIGH_TIME

#define SERVO           52

#define LINEAR_WAIT_TIME  5000  // ms

Servo servo;


std_msgs::Bool msg;
std_msgs::Bool sense;
geometry_msgs::Twist ir_msg;

ros::NodeHandle nh;
ros::Publisher limit_sense("limit_sense", &sense);
ros::Publisher job_status("job_status", &msg);
ros::Publisher ir_msg_pub("ir_msg", &ir_msg);

void servo_motor (bool x);
void linear_motor (bool x);
void stepper_motor (int x);
void linear_rail (bool x);
void ir_update_ros();



void setup()
{
  // ros initialization
  nh.initNode();
  nh.advertise(job_status);
  nh.advertise(limit_sense);
  nh.advertise(ir_msg_pub);

  // IO initialization
  pinMode(IR_FRONT_LEFT,    INPUT);
  pinMode(IR_FRONT_RIGHT,   INPUT);
  pinMode(IR_SIDE_LEFT,     INPUT);
  pinMode(IR_SIDE_RIGHT,    INPUT);

  pinMode(LS_STEPPER_TOP,   INPUT);
  pinMode(LS_STEPPER_BOT,   INPUT);
  pinMode(LS_BOX_LEFT,      INPUT);
  pinMode(LS_BOX_RIGHT,     INPUT);
  pinMode(LS_RAIL_TOP,      INPUT);
  pinMode(LS_RAIL_BOT,      INPUT);

  pinMode(PUp,              OUTPUT);
  pinMode(PUn,              OUTPUT);
  pinMode(DRp,              OUTPUT);
  pinMode(DRn,              OUTPUT);
  pinMode(MFp,              OUTPUT);
  pinMode(MFn,              OUTPUT);
  digitalWrite(PUp,         HIGH);
  digitalWrite(DRp,         HIGH);
  digitalWrite(MFp,         HIGH); // initialize with no actuation for stepper)
  digitalWrite(MFn,         LOW);

  servo.attach(SERVO);

  Serial1.begin(9600);

  // motors reset
  servo_motor(0);
  linear_motor(0);
  // stepper_motor(0);
  digitalWrite(MFn, LOW); // free stepper motor
}

void loop()
{
  ir_update_ros();
  nh.spinOnce();
}

void servo_motor(bool x)
{
  nh.spinOnce();
  switch(x)
  {
    case false:
      servo.writeMicroseconds(500);
      break;
      
    case true:
      servo.writeMicroseconds(2500);
      break;

    default:
    break;
  }
}

void linear_motor(bool x)
{
  nh.spinOnce();
  switch(x)
  {
    case true:
      Serial1.write(SBT_MOTOR1_FULL_FORWARD);
      delay(LINEAR_WAIT_TIME);
      Serial1.write(SBT_ALL_STOP);
      break;
      
    case false:
      Serial1.write(SBT_MOTOR1_FULL_REVERSE);
      delay(LINEAR_WAIT_TIME);
      Serial1.write(SBT_ALL_STOP);
      break;
      
    default:
    break;
      
  }
}

void stepper_motor(int x)
{
  nh.spinOnce();
  digitalWrite(MFn, HIGH);

  switch(x)
  {
    case 0:
      digitalWrite(DRn, HIGH);
      delayMicroseconds(10);
      while(digitalRead(LS_STEPPER_BOT) == LOW)
      {
        digitalWrite(PUn, HIGH);
        delayMicroseconds(PULSE_HIGH_TIME);
        digitalWrite(PUn, LOW);
        delayMicroseconds(PULSE_LOW_TIME);
        nh.spinOnce();
      }
      break;
      
    case 1:
      digitalWrite(DRn, LOW);
      delayMicroseconds(10);
      while(digitalRead(LS_STEPPER_TOP) == LOW)
      {
        digitalWrite(PUn, HIGH);
        delayMicroseconds(PULSE_HIGH_TIME);
        digitalWrite(PUn, LOW);
        delayMicroseconds(PULSE_LOW_TIME);
        nh.spinOnce();
      }
      break;
      
    default:
    break;
  }  
}

void linear_rail (bool x)
{
  nh.spinOnce();
  switch(x)
  {
    case 0:
      while(digitalRead(LS_RAIL_BOT) != true)
      {
        Serial1.write(SBT_MOTOR2_FULL_REVERSE);
        delay(100);
        nh.spinOnce();
      }
      Serial1.write(SBT_ALL_STOP);
      break;
      
    case 1:
      while(digitalRead(LS_RAIL_TOP) != true)
      {
        Serial1.write(SBT_MOTOR2_FULL_FORWARD);
        delay(100);
        nh.spinOnce();
      }
      Serial1.write(SBT_ALL_STOP);
      break;
      
    default:
    break;
  }
}

void ir_update_ros()
{
  nh.spinOnce();
  unsigned int no_times = 100;
  ir_msg.linear.x = 0;
  ir_msg.linear.y = 0;
  ir_msg.angular.x = 0;
  ir_msg.angular.y = 0;
  for (int i = 1; i <= no_times; i++)
  {
    ir_msg.linear.x += analogRead(IR_FRONT_LEFT);
    ir_msg.linear.y += analogRead(IR_FRONT_RIGHT);
    ir_msg.angular.x += analogRead(IR_SIDE_LEFT);
    ir_msg.angular.y += analogRead(IR_SIDE_RIGHT);
  }
  ir_msg.linear.x = int(ir_msg.linear.x / no_times);
  ir_msg.linear.y = int(ir_msg.linear.y / no_times);
  ir_msg.angular.x = int(ir_msg.angular.x / no_times);
  ir_msg.angular.y = int(ir_msg.angular.y / no_times);
  ir_msg_pub.publish(&ir_msg);
  delay(50);
}

