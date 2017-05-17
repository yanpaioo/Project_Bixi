#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs import Bool
import math


class MissionPlanner(object):
    x0, y0, yaw0= 0, 0, 0

    #motion tolerance
    translation_tolerance=0.2
    angle_tolerance=5*math.pi/180
    
    #stores ideal positions of boxes
    pushing_pos=[]
    stacking_pos=[]
    desired_heading=0 #desired hole normal is opposite

    #stores detected boxes
    box_centers=[]
    clustered_box_centers=[]

    #limit switch
    limit_switch=False

    def __init__(self, nodename):        
        rospy.init_node('mission_planner')

        #initialise ideal positions 
        self.initialise_pos()

        rospy.Subscriber("/odometry", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/limit_switch", Bool, self.limit_switch_callback, queue_size = 10)

        self.target_goal_pub=rospy.Publisher("/target_goal", Twist, queue_size=10)
        self.stack_pub=rospy.Publisher("/stack", Bool, queue_size=5)
        self.cmd_vel_pub=rospy.Publisher("/vel_cmd", Joy, queue_size=10)

        push_index=0
        stack_index=0

        while not rospy.is_shutdown():
            #if still have boxes to be pushed, push according to list
            if len(self.pushing_pos) != push_index+1:
                if push_index!=len(self.pushing_pos)-1:
                    self.push_box(self.pushing_pos[push_index],  self.stacking_pos[push_index] True)
                else:
                    self.push_box(self.pushing_pos[push_index],  self.stacking_pos[push_index], False)
                #delete this estimated position from list
                push_index+=1

            else:
                #if no more boxes to be pushed, perform stacking 
                if stack_index<5:
                    self.stack_box(self.stacking_pos[len(self.stack_box-stack_index-1])
                    stack_index+=1
                else:
                    #hopefully we've got 10 boxes
                    break


    def initialise_pos(self):
        #store 8 positions
        self.pushing_pos.append([])

    def push_box(self, est_pos, dest_pos, with_reverse=True):
        #go to some distance from estimated position of box
        d=0.5
        goal_1=[est_pos[0]-d*math.cos(self.desired_heading), 
                est_pos[1]-d*math.sin(self.desired_heading), 
                self.desired_heading]
        self.go_to_goal(goal_1)

        #match laser detected boxes
        real_pos=self.match(est_pos)

        #align to real position
        d=0.1
        goal_2=[real_pos[0]-d*math.cos(self.desired_heading), 
                real_pos[1]-d*math.sin(self.desired_heading), 
                self.desired_heading]
        self.go_to_goal(goal_2)

        #push to desired position
        goal_3=[dest_pos[0], 
                dest_pos[1], 
                self.desired_heading]
        self.go_to_goal(goal_3)        

        #return if needed
        if with_reverse:
            self.go_to_goal(goal_2)


    def stack_box(self, est_pos):
        #go to some distance from estimated position of box
        d=0.4
        goal_1=[est_pos[0]+d*math.cos(self.desired_heading-math.pi/2), 
                est_pos[1]+d*math.sin(self.desired_heading-math.pi/2), 
                self.desired_heading]

        self.go_to_goal(goal_1)

        #match laser detected boxes
        real_pos=self.match(est_pos)

        #align to real position
        d=0.2
        goal_2=[real_pos[0]+d*math.cos(self.desired_heading-math.pi/2), 
                real_pos[1]+d*math.sin(self.desired_heading-math.pi/2), 
                self.desired_heading]
        self.go_to_goal(goal_2)

        #move until limit switches touches
        goal_3=[real_pos[0],
                real_pos[1],
                self.desired_heading]
        self.go_to_goal(goal_3)
        self.takein_box(goal_3)
    

        #take another one on the side
        real_pos=self.match(est_pos)

        goal_4=[real_pos[0],
                real_pos[1],
                self.desired_heading]
        self.go_to_goal(goal_4)
        self.takein_box(goal_4)

    def takein_box(self):
        #move until limit switches touches
        while not True and not rospy.is_shutdown():
            msg=Twist()
            vel=150           
            msg.buttons=[1024, 1024+vel, 1024]
            self.cmd_vel_pub.publish(msg) 
            if self.limit_switch:
                break

        #initiate stacking, publish True to arduino
        msg=Bool()
        msg.data=True
        self.stack_pub.publish(True)

    def limit_switch_callback(self, msg):
        self.limit_switch = msg.data

    def match(self, box_pos):
        #find nearest box and check if within tolerance, if cannot find, sleep a while until box seen.
        tolerance=0.3
        min_length=100
        while box_found is False and not rospy.is_shutdown():
            for box in self.clustered_box_centers:
                d=math.sqrt((box[0]-box_pos[0])**2+(box[1]-box_pos[1])**2)
                if d<min_length:
                    min_length=d
                    nearest_box=box

            if min_length<tolerance:
                return nearest_box
    
            rospy.sleep(1)


    def edgeCallback(self, msg):
        #for a detected edge, add it into the list. If list is full, replace the first element.
        if len(self.box_centers)==n_edge:
            #remove the first element
            del self.box_centers[0]

        _, _, yaw_angle = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        self.box_centers.append([msg.pose.x, msg.pose.y])

        #perform clustering to edges
        X=np.asarray(self.box_centers)
        bandwidth = estimate_bandwidth(X, quantile=0.4)
        ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
        ms.fit(X)

        self.clustered_box_centers = ms.cluster_centers_

    def go_to_goal(self, goal):
        msg=PoseStamped()

        msg.header.frame_id="odom"
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        q_angle = quaternion_from_euler(0, 0, goal[2])
        msg.pose.orientation = Quaternion(*q_angle)

        while not rospy.is_shutdown():
            #at least publish once, 
            self.target_goal_pub.publish(msg)
            #if goal achieved
            if math.sqrt((self.x0-goal[0])**2+(self.y0-goal[1])**2)<self.translation_tolerance**2 and abs(math.atan2(math.sin(self.yaw0-goal[2]), math.cos(self.yaw0-goal[2])))<self.angle_tolerance:
                break

    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
        

if __name__ == '__main__':
    try:
        MissionPlanner(nodename="mission_planner")
    except rospy.ROSInterruptException:
        rospy.loginfo("Mission planner finished")

if __name__ == '__main__':


