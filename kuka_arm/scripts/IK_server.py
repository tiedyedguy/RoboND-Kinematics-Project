#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import random
from mpmath import *
from sympy import *
import numpy as np
import time
import math

#define global variables
T0_1 = 0
T1_2 = 0
T2_3 = 0
T3_4 = 0
T4_5 = 0
T5_6 = 0
T6_7 = 0
Tcorr = 0
T_total = 0
Ree = 0

#Setting up global Parameters
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
rollsym = symbols('roll')
pitchsym = symbols('pitch')
yawsym = symbols('yaw')

#Setting up global DH Table
s = {alpha0:      0, a0:       0, d1:  0.75,
     alpha1:  -np.pi/2, a1:    0.35, d2:     0, q2: q2-np.pi/2,
     alpha2:      0, a2:    1.25, d3:     0,
     alpha3:  -np.pi/2, a3:  -0.054, d4:  1.50,
     alpha4:   np.pi/2, a4:       0, d5:     0,
     alpha5:  -np.pi/2, a5:       0, d6:     0,
     alpha6:      0, a6:       0, d7: 0.303, q7: 0}

R_z = Matrix([[ cos(yawsym), -sin(yawsym), 0],
              [ sin(yawsym),  cos(yawsym), 0],
              [          0,           0,   1]])

R_y = Matrix([[  cos(pitchsym), 0, sin(pitchsym)],
              [              0, 1,             0],
              [ -sin(pitchsym), 0, cos(pitchsym)]])
              

R_x = Matrix([[  1,            0,             0],
              [  0, cos(rollsym), -sin(rollsym)],
              [  0, sin(rollsym),  cos(rollsym)]])




def setupvariables():
    global T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T_total, Ree, R_x, R_y, R_z


    print("Starting to set up all our variables")

    Ree = R_x * R_y * R_z


    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
    T6_7 = T6_7.subs(s)
    print("Starting to Simplify these Matrix-es")
    #Starting building links from base_link.
    
    T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
    
    

    #How to fix for gribber, ripped right from video
    R_z = Matrix([[ cos(np.pi), -sin(np.pi), 0,0],
              [ sin(np.pi),  cos(np.pi), 0,0],
              [          0,           0, 1,0],
              [          0,           0, 0,1]])
    R_y = Matrix([[  cos(-np.pi/2), 0, sin(-np.pi/2),0],
              [              0, 1,             0,0],
              [ -sin(-np.pi/2), 0, cos(-np.pi/2),0],
              [              0, 0,             0,1]])
    R_corr = R_z * R_y


    #T_total = simplify(T0_7 * R_corr)
    T_total = T0_7 * R_corr
    
    print("Total Homogenous Transform, ready to go!")
def calculate_error(thetas, position):
    pass

def handle_calculate_IK(req):
    global Ree
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            theta1 = theta2 = theta3 = theta4 = theta5 = theta6 = 0
            theta3 = -1.55

            
            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            
            Ree = Ree.evalf(subs={rollsym: roll, pitchsym: pitch, yawsym: yaw })

            # if that's where the EE is, where is the wrist?
            wx = (px - (d6 + d7) * Ree[0,2]).subs(s)
            wy = (py - (d6 + d7) * Ree[1,2]).subs(s)
            wz = (pz - (d6 + d7) * Ree[2,2]).subs(s)

            # Calculate joint angles using Geometric IK method
            #print("Wrist at: ",wx,wy,wz)
            theta1 = atan2(py, px)
            

            j2position = T0_1 * T1_2 * Matrix([0,0,0,1])
            j3position = T0_1 * T1_2 * T2_3 * Matrix([0,0,0,1])

            j2x = j2position[0].evalf(subs={q1:theta1})
            j2y = j2position[1].evalf(subs={q1:theta1})
            j2z = j2position[2].evalf(subs={q1:theta1})
            #print("Join2At ",j2x,j2y,j2z)
            
            #firstside = a2.subs(s)
            #secondside = d4.subs(s)
            firstside = 1.25
            secondside = 0.96
            thirdside = math.sqrt((wx - j2x)**2 + (wy - j2y)**2 + (wz - j2z)**2)
            #print("Sides:", firstside, secondside, thirdside)
            
            theta2 = math.radians(62.206 - math.degrees(acos((firstside**2 + thirdside**2 - secondside**2) / (2 * firstside * thirdside)) + cos((wx - j2x)/thirdside)))
            
            j3x = j3position[0].evalf(subs={q1:theta1, q2:theta2})
            j3y = j3position[1].evalf(subs={q1:theta1, q2:theta2})
            j3z = j3position[2].evalf(subs={q1:theta1, q2:theta2})
            
            print("Join3At", j3x, j3y, j3z)
            theta3 = math.radians(155.746 - math.degrees(acos((firstside**2 - thirdside**2 + secondside**2) / (2 * firstside * secondside)) + cos((wx - j3x)/thirdside)))



            # Populate response for the IK request

            
            #theta4 = roll
            #theta5 = pitch
            #theta6 = yaw

            # In the next line replace theta1,the
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            print (joint_trajectory_point.positions)
            joint_trajectory_list.append(joint_trajectory_point)
            calculate_error(joint_trajectory_point.positions, req.poses[x].position)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    setupvariables()
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
