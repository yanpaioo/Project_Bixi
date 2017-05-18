#define SBT_MOTOR1_FULL_FORWARD 127
#define SBT_MOTOR1_FULL_REVERSE 1
#define SBT_MOTOR1_STOP 64
#define SBT_ALL_STOP  0

#include <ros.h>
#include <std_msgs/Bool.h>
// #include <SoftwareSerial.h>

// stepper motor constants
const int stepper_speed = 80; // speed 0-100
const int PUp = 2;
const int PUn = 3;
const int DRp = 4;
const int DRn = 5;
const int MFp = 7;
// const int MFn = 8;

// linear actuator power pin
const int linear_power_pin = 13;


// sensors
const int sensor1 = 8;
const int sensor2 = 9;
const int limit_switch1 = A5;
const int limit_switch2 = 41;
const int power1 = A8;
const int power2 = 33;


std_msgs::Bool msg;
std_msgs::Bool sense;

ros::NodeHandle nh;
ros::Publisher limit_sense("limit_sense", &sense);
ros::Publisher job_status("job_status", &msg);


int linear_wait_time = 5500; // in ms
float y = map(stepper_speed, 0, 100, 1000, 230);
int box_count = 0;
const int box_count_limit = 9;

void killMotors(){
  Serial.write(SBT_ALL_STOP);   //kill motors for 0.5 second
  Serial.println("kill motors for half a second");
  delay(500);  
}

void linear_motor(bool a)
{
  if (a == true) Serial.write(SBT_MOTOR1_FULL_FORWARD);
  else Serial.write(SBT_MOTOR1_FULL_REVERSE);

  delay(linear_wait_time);
}

void stepper_motor(int x)
{
  if (x == 0)
  {
    digitalWrite(DRn, HIGH);
    delayMicroseconds(10);
    while (digitalRead(sensor2) == LOW)
    {
      digitalWrite(PUn, HIGH);
      delayMicroseconds(y);
      digitalWrite(PUn, LOW);
      delayMicroseconds(y);
    }
  }
  else if (x == 1)
  {
    digitalWrite(DRn, LOW);
    delayMicroseconds(10);
    while (digitalRead(sensor1) == LOW)
    {
      digitalWrite(PUn, HIGH);
      delayMicroseconds(y);
      digitalWrite(PUn, LOW);
      delayMicroseconds(y);
    }
  }
  else if (x == 2) // go down a bit
  {
    digitalWrite(DRn, HIGH);
    int pose_motor = 0;
    delayMicroseconds(10);
    while (pose_motor < 5000)
    {
      digitalWrite(PUn, HIGH);
      delayMicroseconds(y);
      digitalWrite(PUn, LOW);
      delayMicroseconds(y);
      pose_motor++;
    }
  }
  
}


void setup()
{
  // ros initialize
  nh.initNode();
  nh.advertise(job_status);
  nh.advertise(limit_sense);

  // initialize linear motor power pin
  pinMode(linear_power_pin, OUTPUT);
  digitalWrite(linear_power_pin, HIGH);
  
  // initialize stepper motor
  pinMode(PUp, OUTPUT);
  pinMode(PUn, OUTPUT);
  pinMode(DRp, OUTPUT);
  pinMode(DRn, OUTPUT);
  pinMode(MFp, OUTPUT);
  // pinMode(MFn, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(limit_switch1, INPUT);
  pinMode(limit_switch2, INPUT);
  pinMode(power1, OUTPUT);
  pinMode(power2, OUTPUT);
  digitalWrite(power1, HIGH);
  digitalWrite(power2, HIGH);
  
  digitalWrite(PUp, HIGH);
  digitalWrite(DRp, HIGH);
  // digitalWrite(DRn, LOW); // low - up, high - down
  digitalWrite(MFp, HIGH);
  // digitalWrite(MFn, LOW);
  digitalWrite(PUn, HIGH);

  Serial.begin(9600);
  killMotors();

  linear_motor(0); // retract linear motor
  stepper_motor(0); // stepper down
}


void loop()
{
  if (digitalRead(limit_switch1) && digitalRead(limit_switch2))
  {
    if (box_count == 0)
    {
      sense.data = true;
      limit_sense.publish(&sense);
      delay(1000);
      linear_motor(1);  // extend linear actuator
      stepper_motor(1); // stepper goes all the way up
      msg.data = true;
      job_status.publish(&msg);
      box_count++;
      
    }
    else if (box_count < box_count_limit)
    {
      sense.data = true;
      limit_sense.publish(&sense);
      delay(1000);
      
      stepper_motor(2); // stepper goes down a bit
      linear_motor(0);  // retract linear actuator
      stepper_motor(0); // stepper goes all the way down
      linear_motor(1);  // extend linear actuator
      stepper_motor(1); // stepper goes all the way up
  
      msg.data = true;
      job_status.publish(&msg);
      box_count++;
    }
    else if (box_count == box_count_limit)
    {
      sense.data = true;
      limit_sense.publish(&sense);
      delay(1000);
      stepper_motor(2); // stepper goes down a bit
      linear_motor(0);  // retract linear actuator
      msg.data = true;
      job_status.publish(&msg);
      box_count++;
    }
    
  }
  delay(100);
}




