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
        q1, q2, q3, q4, q5, q6, qg  = symbols('q1:7 qg')
        a0, a1, a2, a3, a4, a5, a6 = symbols ('a0:7')
        d1, d2, d3, d4, d5, d6, dg = symbols('d1:7 dg')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        dh_parameters = {alpha0:     0,  a0:      0,  d1:  0.75,  q1:      q1,
                         alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: q2-pi/2,
                         alpha2:     0,  a2:   1.25,  d3:     0,  q3:      q3,
                         alpha3: -pi/2,  a3: -0.054,  d4:   1.5,  q4:      q4,
                         alpha4:  pi/2,  a4:      0,  d5:     0,  q5:      q5,
                         alpha5: -pi/2,  a5:      0,  d6:     0,  q6:      q6,
                         alpha6:     0,  a6:      0,  dg: 0.303,  qg:       0 }



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
            return Matrix([[cos(z), -sin(z), 0],
                           [sin(z),  cos(z), 0],
                           [0     , 0      , 1]])
        def Rot_y(y):
            return Matrix([[ cos(y), 0, sin(y)],
                           [      0, 1, 0     ],
                           [-sin(y), 0, cos(y)]])
        def Rot_x(x):
            return Matrix([[1,      0, 0      ],
                           [0, cos(x), -sin(x)],
                           [0, sin(x),  cos(x)]])

        R_y = Rot_y(-pi/2)
        R_z = Rot_z(pi)
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

            # Calculate joint angles using Geometric IK method
            # theta1
            theta1 = atan2(WC[1], WC[0])
            #

            # theta2
            side_1 = sqrt(WC[0]**2 + WC[1]**2) - 0.35
            side_2 = WC[2] - 0.75
            angle_1 = atan2(side_2,side_1)

            side_a = sqrt(1.5**2 + 0.054**2)
            side_b = sqrt(side_1**2 + side_2**2)
            side_c = 1.25

            alpha = acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b*side_c))

            theta2 = pi/2 - alpha - angle_1

            # theta3
            angle_2 = atan2(0.054,1.5)
            beta = acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a*side_c))

            theta3 = pi/2 - beta - angle_2


            T0_3 = T0_1 * T1_2 * T2_3 # does not need to be calculated everytime
            R0_3 = T0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})[0:3, 0:3]
            R3_6 = R0_3.transpose() * ROT_EE


            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

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
