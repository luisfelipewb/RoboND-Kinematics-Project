#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6  = symbols('q1:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols ('a0:7')
        d1, d2, d3, d4, d5, d6, dg = symbols('d1:7 dg')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Numerical values
        a12 = 0.35
        a23 = 1.25
        a34 = -0.054
        d01 = 0.33 + 0.42
        d34 = 0.96 + 0.54
        d6g = 0.193 + 0.11
        qg  = 0

        # Create Modified DH parameters
        dh_parameters = {alpha0:     0,  a0:   0,  d1: d01,
             alpha1: -pi/2,  a1: a12,  d2:   0,
             alpha2:     0,  a2: a23,  d3:   0,
             alpha3: -pi/2,  a3: a34,  d4: d34,
             alpha4:  pi/2,  a4:   0,  d5:   0,
             alpha5: -pi/2,  a5:   0,  d6:   0,
             alpha6:     0,  a6:   0,  dg: d6g}


        # Define Modified DH Transformation matrix
        def tf_matrix(a, alpha, d, q):
            TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                         [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                         [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                         [                0,                 0,           0,             1]])
            return TF

        # Create individual transformation matrices
        T0_1 = tf_matrix(a0, alpha0, d1, q1).subs(dh_parameters)
        T1_2 = tf_matrix(a1, alpha1, d2, q2).subs(dh_parameters)
        T2_3 = tf_matrix(a2, alpha2, d3, q3).subs(dh_parameters)
        T3_4 = tf_matrix(a3, alpha3, d4, q4).subs(dh_parameters)
        T4_5 = tf_matrix(a4, alpha4, d5, q5).subs(dh_parameters)
        T5_6 = tf_matrix(a5, alpha5, d6, q6).subs(dh_parameters)
        T6_g = tf_matrix(a6, alpha6, dg, qg).subs(dh_parameters)


        # Create auxiliary rotation matrices
        def Rot_z(z):
            return Matrix([[cos(z), -sin(z), 0, 0],
                           [sin(z),  cos(z), 0, 0],
                           [0     , 0      , 1, 0],
                           [0     , 0      , 0, 1]])
        def Rot_y(y):
            return Matrix([[ cos(y), 0, sin(y), 0],
                           [      0, 1, 0     , 0],
                           [-sin(y), 0, cos(y), 0],
                           [0      , 0, 0     , 1]])
        def Rot_x(x):
            return Matrix([[1,      0, 0      , 0],
                           [0, cos(x), -sin(x), 0],
                           [0, sin(x),  cos(x), 0],
                           [0,      0, 0      , 1]])

        R_y = Rot_y(-90*dtr)
        R_z = Rot_z(180*dtr)
        R_corr = (R_z * R_y)

        # Extract rotation matrices from the transformation matrices
        # RA_B = TA_B.row_del(3).col_del(3)
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            ROT_EE = Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll) * R_corr

            EE = Matrix([[px],[py],[pz]])
            #Calculate wrist center
            WC = EE - (0.303) * ROT_EE[:,2]

            print('EE', EE) 
            print('WC', WC) 
            # Calculate joint angles using Geometric IK method
            #
            #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
