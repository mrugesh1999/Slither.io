#!/usr/bin/env python

# Importing required libraries
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty
import math
import numpy as np

# Initiationg a Gait class
class Gait:
    def get_angle_params(self, num_joints):
    	# This function calculates the angles for each link
        pass

    def update_and_publish_angles(self, t):
    	# This function will update and publish the calculated angles to topics /command
        pass
        
# The message that will appear after execution program
msg = """
Control Your Toy!
---------------------------

k : Go Forward
l : Rotate Counter Clock Wise
s : Stop smoothly
CTRL-C to quit
"""

# Function to take the user input
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


sin = math.sin
rad = math.radians

# Inheriting the class gait for moving forward
class ForwardMotion(Gait):
    def __init__(self, publishers, num_joints=10):
        self.num_joints = num_joints
        self.amplitudes, self.omegas, self.phases, self.axis_lags = self.get_angle_params(num_joints)
        self.publishers = publishers

    def get_angle_params(self, num_joints):
        amp_reduction_factor_hor = 0.5 #1.5 #0.4
        amp_reduction_factor_ver = 0.5 #1.5 #0.1

        a_hor = rad(0) * amp_reduction_factor_hor
        a_ver = rad(60) * amp_reduction_factor_ver
        amplitudes = [a_hor, a_ver]

        w_hor = rad(150)
        w_ver = rad(150)
        omegas = [w_hor, w_ver]

        ph_hor = rad(0)
        ph_ver = rad(120)
        phases = [ph_hor, ph_ver]

        axis_lag_hor = rad(0)
        axis_lag_ver = rad(0)
        axis_lags = [axis_lag_hor, axis_lag_ver]

        return amplitudes, omegas, phases, axis_lags


    def update_and_publish_angles(self, t):
        
        for i in range(self.num_joints):
            n = i+1
            a = self.amplitudes[n % 2]
            w = self.omegas[n % 2]
            ph = self.phases[n % 2]
            lag = self.axis_lags[n % 2]

            link_num = i // 2   # to calculate the ith horizontal/vertical link number

            angle = (a * sin(w*t + ph*n + lag))*math.pow(-1, link_num)
            self.publishers[i].publish(angle)

# Inheriting GAit class for Rotational movements
class Rotate(Gait):
    def __init__(self, publishers, num_joints=10):
        self.num_joints = num_joints
        self.amplitudes, self.omegas, self.phases, self.axis_lags = self.get_angle_params(num_joints)
        self.publishers = publishers

    def get_angle_params(self, num_joints):

        amp_reduction_factor_hor = 0.1 #1.5 #0.4
        amp_reduction_factor_ver = 0.1 #1.5 #0.1

        a_hor = rad(90) * amp_reduction_factor_hor
        a_ver = rad(90) * amp_reduction_factor_ver
        amplitudes = [a_hor, a_ver]

        w_hor = rad(180)
        w_ver = rad(180)
        omegas = [w_hor, w_ver]

        ph_hor = rad(90)
        ph_ver = rad(0)
        phases = [ph_hor, ph_ver]

        axis_lag_hor = rad(0)
        axis_lag_ver = rad(0)
        axis_lags = [axis_lag_hor, axis_lag_ver]

        return amplitudes, omegas, phases, axis_lags


    def update_and_publish_angles(self, t):
        
        for i in range(self.num_joints):
            n = i+1
            a = self.amplitudes[n % 2]
            w = self.omegas[n % 2]
            ph = self.phases[n % 2]
            lag = self.axis_lags[n % 2]

            link_num = i // 2   # to calculate the ith horizontal/vertical link number

            angle = (a * sin(w*t + ph*link_num + lag))*math.pow(1, link_num)
            self.publishers[i].publish(angle)


    
def get_publisher_nodes(num_joints):
    joint_publishers = []
    for i in range(num_joints):
        pub_node_name = '/slither/joint_{}_position_controller/command'.format(i+1)
        pub_node = rospy.Publisher(pub_node_name, Float64, queue_size=1000)
        joint_publishers.append(pub_node)

    return joint_publishers


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    print(msg)
    while not rospy.is_shutdown():
        rospy.init_node('slither_linear_progression', anonymous=True)
        key = getKey()
        if key == 'k':
            rospy.loginfo('Moving Forward')
            num_joints = 10
            joint_publishers = get_publisher_nodes(num_joints=num_joints)
            gait = ForwardMotion(publishers=joint_publishers, num_joints=num_joints)
            rate = rospy.Rate(10)
            time_begin = rospy.Time.now()
            while not rospy.is_shutdown():
                time = (rospy.Time.now() - time_begin).to_sec()
                gait.update_and_publish_angles(t=time)
                rate.sleep()
                key = getKey()
                if key == 's':
                    break
        if key == 'l':
            rospy.loginfo('Rotating Counter Clock Wise')
            num_joints = 10
            joint_publishers = get_publisher_nodes(num_joints=num_joints)
            gait = Rotate(publishers=joint_publishers, num_joints=num_joints)
            rate = rospy.Rate(10)
            time_begin = rospy.Time.now()
            while not rospy.is_shutdown():
                time = (rospy.Time.now() - time_begin).to_sec()
                gait.update_and_publish_angles(t=time)
                rate.sleep()
                key = getKey()
                if key == 's':
                    break
