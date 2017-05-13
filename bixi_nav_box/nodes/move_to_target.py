#!/usr/bin/env python
import roslib
roslib.load_manifest('move_to_target')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

## PID constants
Del_T = 50.0   # time step in ms
P_ang = 1.0
I_ang = 0.0002
D_ang = 1.0
P_lin = 1.0
I_lin = 1.0005
D_lin = 1.0
Lin_vel_thres = 0.25    ## m/s
Ang_vel_thres = 0.2     ## rad/s
X_err_thres = 0.01

## PID variables
Integral_yaw = 0.0
Integral_x = 0.0
Integral_y = 0.0
Pre_yaw_err = 0.0
Pre_x_err = 0.0
Pre_y_err = 0.0

## global publisher
Pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)

## global variables
Odom = Odometry()
Box_pose = Pose()   ## box pose relative to robot
Stop_pose = Pose()      ## stop pose relative to laser and box
Stop_pose.position.x = 0.0
Stop_pose.position.y = 0.0
Stop_pose.position.z = 0.0
State = 0
Stop_iteration = 50




## update odom data
def odomCallback(data):
    print("Box pose\n" + data.data)
    Odom = data.data


## update relative box pose data
def boxCallback(data):
    print("Odom\n" + data.data)
    Box_pose = data.data


## PID yaw
def PID_yaw_move():

    global Integral_yaw, Pre_yaw_err

    cur_err = Box_pose.position.z
    integral = Integral_yaw + (cur_err * Del_T)
    derivative = (cur_err - Pre_yaw_err) / Del_T

    yaw_out = (P_ang*cur_err) + (I_ang*integral) + (D_ang*derivative)

    if yaw_out > Ang_vel_thres: yaw_out = Ang_vel_thres
    elif yaw_out < -Ang_vel_thres: yaw_out = -Ang_vel_thres

    Pre_yaw_err = cur_err
    Integral_yaw = integral

    return yaw_out


## PID_y
def PID_y_move():

    global Integral_y, Pre_y_err

    cur_err = Box_pose.position.y
    integral = Integral_y + (cur_err * Del_T)
    derivative = (cur_err - Pre_y_err) / Del_T

    y_out = (P_lin*cur_err) + (I_lin*integral) + (D_lin*derivative)

    if y_out > Lin_vel_thres: y_out = Lin_vel_thres
    elif y_out < -Lin_vel_thres: y_out = -Lin_vel_thres

    Pre_y_err = cur_err
    Integral_y = integral

    return y_out


## PID_x
def PID_x_move():

    global Integral_x, Pre_x_err

    cur_err = Box_pose.position.x
    integral = Integral_x + (cur_err * Del_T)
    derivative = (cur_err - Pre_x_err) / Del_T

    x_out = (P_lin*cur_err) + (I_lin*integral) + (D_lin*derivative)

    if x_out > Lin_vel_thres: y_out = Lin_vel_thres
    elif x_out < -Lin_vel_thres: y_out = -Lin_vel_thres

    Pre_x_err = cur_err
    Integral_x = integral

    return x_out



## move robot
def move_robot():
    print("Robot moving.....")
    cmd_vel = Twist()

    cmd_vel.angular.z = PID_yaw_move()
    cmd_vel.linear.y = PID_y_move()
    cmd_vel.angular.z = PID_yaw_move()
    Pub_cmd_vel.publish(cmd_vel) 


def listener():

    rospy.init_node('move_to_target', anonymous=True)
    rospy.Subscriber("box_pose", Pose, boxCallback)
    rospy.Subscriber("odom", Odometry, odomCallback)
    
    print("-----move_to_target initialized-----")

    rate = rospy.Rate(1/(Del_T/1000))
    while not rospy.is_shutdown():
        move_robot()
        rate.sleep()

    print("Robot stops.....")
    rospy.spin()

if __name__ == '__main__':
    listener()