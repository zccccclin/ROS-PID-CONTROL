#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from math import *
import numpy as np


#Define variables
error_forward_ = 0
error_angle_ = 0
error_forward_prev_ = 0
error_angle_prev_ = 0
P_forward_ = 0
P_angle_ = 0
I_forward_ = 0
I_angle_ = 0
D_forward_ = 0
D_angle_ = 0

dt = .2
target_distance = 1.3
target_angle = 0.0
Kp_f = 2
Ki_f = 0.00
Kd_f = 0.02
Kp_a = 2
Ki_a = 0.00
Kd_a = 0.005
pillar_x = 21.5
pillar_y = 23

vel_cmd = Twist()

pos_x_ = 0
pos_y_ = 0
ang_z_ = 0




def trueCallBack(msg):
    global pos_x_, pos_y_, ang_z_
    i = 0
    while i < len(msg.name):
        if msg.name[i] == "husky::base_link":
            break
        i += 1
    if i>=len(msg.name):
        return

    pose = msg.pose[i]
    pos_x_ = pose.position.x
    pos_y_ = pose.position.y
    qz = pose.orientation.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qw = pose.orientation.w
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    ang_z_ = atan2(siny_cosp,cosy_cosp)

def pidAlgorithm():
    global P_forward_, P_angle_, I_forward_, I_angle_, D_forward_, D_angle_, error_forward_, error_angle_

    #Define Publishing topics
    vel_pub_ = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=5)
    error_forward_pub_ = rospy.Publisher('error_forward', Float32, queue_size=200)
    error_angle_pub_ = rospy.Publisher('error_angle', Float32, queue_size=1)
    control_signal_forward_pub_ = rospy.Publisher('control_signal_forward', Float32, queue_size=1)
    control_signal_angle_pub_ = rospy.Publisher('control_signal_angle', Float32, queue_size=1)

    #Define Subscribing topics
    true_sub_ = rospy.Subscriber('/gazebo/link_states', LinkStates, trueCallBack)

    Dx = pillar_x - pos_x_
    Dy = pillar_y - pos_y_
    
    error_forward_prev_ = error_forward_
    error_angle_prev_ = error_angle_
	
    error_forward_ = sqrt(Dx*Dx + Dy*Dy) - target_distance
    error_angle_ = atan2(Dy, Dx) - ang_z_

    if error_angle_ < -pi:
        error_angle_ += 2*pi
    if error_angle_ > pi:
        error_angle_ -= 2*pi
    
    #Proportional term
    P_angle_ = Kp_a * error_angle_
    P_forward_ = Kp_f * error_forward_

    #Integral term
    I_forward_ += dt * error_forward_
    I_angle_ += dt * error_angle_

    #Derivative term
    D_forward_ = (-error_forward_prev_ + error_forward_)/dt
    D_angle_ = (-error_angle_prev_ + error_angle_)/dt

    #Computation of movement
    trans_forward_ = P_forward_ + Ki_f*I_forward_ + Kd_f*D_forward_
    trans_angle_ = P_angle_ + Ki_a*I_angle_ + Kd_a*D_angle_
    trans_forward_
    if abs(error_angle_) <= 0.01:
        trans_angle_ = 0
    if abs(error_forward_) <= 0.02:
        trans_forward_ = 0
#    error_forward_prev_ = error_forward_
 #   error_angle_prev_ = error_angle_

    #Limit
    #trans_forward_ = max(-7.0, min(trans_forward_, 7.0))

    rospy.loginfo("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Linear_error: %f",  
		trans_forward_, trans_angle_, error_angle_, error_forward_)

    vel_cmd.linear.x = trans_forward_
    vel_cmd.angular.z = trans_angle_


    vel_pub_.publish(vel_cmd)

    error_forward_pub_.publish(error_forward_)
    control_signal_forward_pub_.publish(trans_forward_)
    error_angle_pub_.publish(error_angle_)
    control_signal_angle_pub_.publish(trans_angle_)

if __name__ == '__main__':
    try:
        rospy.init_node('pid_control_node_py', anonymous=True)
        rate = rospy.Rate(1/dt)
        for i in range(3):
            rospy.loginfo("%d", i)
            rospy.Rate(1).sleep()
        rospy.loginfo('Node Initialized')
        input("Press Enter to start...")
        while not rospy.is_shutdown():
            pidAlgorithm()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
