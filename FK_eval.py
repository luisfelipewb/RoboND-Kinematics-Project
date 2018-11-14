from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
import numpy as np

# Define symbols
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

# DH Parameters
s = {alpha0:     0,  a0:   0,  d1: d01,
     alpha1: -pi/2,  a1: a12,  d2:   0,
     alpha2:     0,  a2: a23,  d3:   0,
     alpha3: -pi/2,  a3: a34,  d4: d34,
     alpha4:  pi/2,  a4:   0,  d5:   0,
     alpha5: -pi/2,  a5:   0,  d6:   0,
     alpha6:     0,  a6:   0,  dg: d6g}

#### Homogeneous Transforms
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

T6_g = Matrix([[             cos(qg),            -sin(qg),            0,              a6],
               [ sin(qg)*cos(alpha6), cos(qg)*cos(alpha6), -sin(alpha6), -sin(alpha6)*dg],
               [ sin(qg)*sin(alpha6), cos(qg)*sin(alpha6),  cos(alpha6),  cos(alpha6)*dg],
               [                   0,                   0,            0,               1]])
T6_g = T6_g.subs(s)

print ("T0_1", T0_1)
print ("T1_2", T1_2)
print ("T2_3", T2_3)
print ("T3_4", T3_4)
print ("T4_5", T4_5)
print ("T5_6", T5_6)
print ("T6_g", T6_g)


#TODO add matrix to rotate gripper and adjust coords


#TODO Evaluate total transformation matrix

