#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler, euler_from_quaternion


import math

class GoToGoal(object):
    x0, y0, yaw0= 0, 0, 0
    goal_des=[0, 0, 0]
    initialize=True

    ## PID constants
    Del_T = 100.0   # time step in ms
    P_ang = 100.0
    D_ang = 10.0
    P_lin = 100.0
    D_lin = 10.0
    Lin_vel_thres = 400.0 # max 660
    Ang_vel_thres = 400.0 # max 660
    bias = 1024.0
    Pre_ang_error = 0.0
    Pre_x_error = 0.0
    Pre_y_error = 0.0

    def __init__(self, nodename):
        heading_threshold=3*math.pi/180

        rospy.init_node('go_to_goal')

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size = 50)
        rospy.sleep(1)
        rospy.Subscriber("/target_goal", PoseStamped, self.goal_callback , queue_size=10)

        self.cmd_vel_pub=rospy.Publisher("/vel_cmd", Twist, queue_size=10)

        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            #if direction not similar, rotate
            if abs(self.yaw0-self.goal_des[2])>heading_threshold:
                self.rotate(self.goal_des[2])
            else:
                #else translate to goal    
                self.translate(self.goal_des[0], self.goal_des[1])

            r.sleep()

    def goal_callback(self, msg):

        #store target goal as private variable
        _, _, yaw_des = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.goal_des=[msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_des]


    def rotate(self, angle):
        global Pre_ang_error
        msg=Twist()
        

        ang_error=math.atan2(math.sin(angle-self.yaw0), math.cos(angle-self.yaw0))
        derivative = (ang_error - Pre_ang_error) / Del_T
        angular_vel = (P_ang * ang_error) + (D_ang * derivative)

        # if abs(ang_error)<math.pi:
        #     msg.angular.z=1024+angular_vel
        # else:
        #     msg.angular.z=1024-angular_vel

        if angular_vel > Ang_vel_thres:
            angular_vel = Ang_vel_thres
        elif angular_vel < -Ang_vel_thres:
            angular_vel = -Ang_vel_thres

        msg.angular.z = bias + angular_vel
        self.cmd_vel_pub.publish(msg)
        Pre_ang_error = ang_error

    def translate(self, x_target, y_target):
        global Pre_x_error, Pre_y_error
        msg=Twist()
        # vel=200 #must be small to avoid jerking, and secondly to avoid switching surface
        # distance_threshold=0.1

        x_error=(x_target-self.x0)*math.cos(self.yaw0)+(y_target-self.y0)*math.sin(self.yaw0)
        y_error=-(x_target-self.x0)*math.sin(self.yaw0)+(y_target-self.y0)*math.cos(self.yaw0)
        x_derivative = (x_error - Pre_x_error) / Del_T
        y_derivative = (y_error - Pre_y_error) / Del_T

        x_linear_vel = (P_lin * x_error) + (D_lin * x_derivative)
        if x_linear_vel > Lin_vel_thres:
            x_linear_vel = Lin_vel_thres
        elif x_linear_vel < -Lin_vel_thres
            x_linear_vel = -Lin_vel_thres

        y_linear_Vel = (P_lin * y_error) + (D_lin * y_derivative)
        if y_linear_vel > Lin_vel_thres:
            y_linear_vel = Lin_vel_thres
        elif y_linear_vel < -Lin_vel_thres
            y_linear_vel = -Lin_vel_thres


        msg.linear.x = bias + x_linear_vel
        msg.linear.y = bias + y_linear_vel
        self.cmd_vel_pub.publish(msg)
        Pre_x_error = x_error
        Pre_y_error = y_error

    def odom_callback(self, msg):


        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
        

        if self.initialize is True:
            self.goal_des=[self.x0, self.y0, self.yaw0]
            self.initialize=False



if __name__ == '__main__':
    try:
        GoToGoal(nodename="go_to_goal")
    except rospy.ROSInterruptException:
        rospy.loginfo("Go to goal finished.")

if __name__ == '__main__':


