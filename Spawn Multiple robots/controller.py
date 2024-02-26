#! /usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist


x1 = 0.0
y1 = 0.0
theta1 = 0.0

def newOdom1(msg):

    global x1
    global y1
    global theta1

    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y,rot_q.z, rot_q.w])


x2 = 0.0
y2 = 0.0
theta2 = 0.0

def newOdom2(msg):

    global x2
    global y2
    global theta2

    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta2) = euler_from_quaternion([rot_q.x, rot_q.y,rot_q.z, rot_q.w])


rospy.init_node("speed_controller")

sub1_odo = rospy.Subscriber("/robot1/odom", Odometry, newOdom1)
#pub1_v = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)


vel1_x = 0.0
vel1_w = 0.0


def save_vel(msg):
    global vel1_w
    global vel1_x

    vel1_x = msg.linear.x
    vel1_w = msg.angular.z
    
init_x1 = x1
init_x2 = x2
init_y1 = y1
init_y2 = y2
init_theta1 = theta1
init_theta2 = theta2


sub1_v = rospy.Subscriber("/robot1/cmd_vel", Twist, save_vel)


sub2_odo = rospy.Subscriber("/robot2/odom", Odometry, newOdom2)
pub2_v = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=1)


r = rospy.Rate(4)

speed1 = Twist()
goal1 = Point()
goal1.x = 10
goal1.y = 10

speed2 = Twist()
goal2 = Point()
goal2.x = goal1.x - 4
goal2.y = goal1.y - 4

t = 1
i = 0 #used to increas the theta value
radius = 6
delt = 0.1

init_velocity1 = 0.5

a = 3
b = 2
epsilon = 2


while not rospy.is_shutdown():
    

    '''
    #this code is for making the robot moving to one place to another
    inc_x1 = goal1.x - x1
    inc_y1 = goal1.y - y1

    angle_to_goal1 = math.atan2(inc_y1, inc_x1)


    inc_x2 = goal2.x - x2
    inc_y2 = goal2.y - y2

    angle_to_goal2 = math.atan2(inc_y2, inc_x2)

    if abs(angle_to_goal1 - theta1)> 0.1:
        speed1.linear.x = 0.0
        speed1.angular.z = 0.3
    
    else:
        
        speed1.linear.x = 0.5
        speed1.angular.z = 0.0

    if abs(angle_to_goal2 - theta2)> 0.1:
        speed2.linear.x = 0.0
        speed2.angular.z = 0.3
    
    else:
        
        speed2.linear.x = 0.5
        speed2.angular.z = 0.0


    '''

    '''
    vel1_x = -init_velocity1*math.sin(i)
    vel1_y = init_velocity1*math.cos(i)
    

    speed1.linear.x = vel1_x
    speed1.angular.z = vel1_w
    '''

    #speed2.linear.x = round(vel1_x,1) + epsilon * ( round((math.sqrt((x1-x2)**2 + (y1-y2)**2)),2)-a)
    #speed2.angular.z = round(vel1_w,1) + epsilon*(round(theta1,1) - round(theta2,1))


    if abs((theta1) - (theta2)) <= 3*math.pi/8:

        speed2.linear.x = round(vel1_x,2) + epsilon * (((math.sqrt((x1-x2)**2 + (y1-y2)**2))-a))
        speed2.angular.z = round(vel1_w,2) + epsilon*((theta1) - (theta2))
    
    else:
        speed2.angular.z = round(vel1_w,2) + 0.3*epsilon*((theta1) - (theta2))

    print(t)
    print("difference value : ",(theta1) - (theta2))


    
    print("Radius : ",math.sqrt((x1-x2)**2 + (y1-y2)**2))
    print("Master:",vel1_x, vel1_w)
    print("Slave:",speed2.linear.x, speed2.angular.z)
    
    #print("ODOM rob 1", round(x1, 1) , round(y1,1), round(theta1,2))
    #print("ODOM rob 2", round(x2, 1) , round(y2,1), round(theta2,2))

    #error_x = abs(abs(x2) - abs(x1))# - abs(abs(init_x2) -abs(init_x1))
    #error_y = abs(abs(y2) - abs(y1)) - abs(abs(init_y1) - abs(init_y2))
    #error_theta = abs(abs(theta2) - abs(theta1))# - abs(abs(theta1) - abs(theta2))
    #error_x = abs(x2 - x1) - abs(init_x1 - init_x2)
    #print("Error", round(error_x,1), round(error_y,1), round(error_theta,2))

    #print("Error", round(error_x,3), round(error_theta,3))


    #pub1_v.publish(speed1)
    '''if abs(speed2.angular.z) <= 2 and abs(speed2.linear.x) <= 2:

        pub2_v.publish(speed2)'''
    pub2_v.publish(speed2)

    t = t+delt


    i =i + 0.01


    r.sleep()


