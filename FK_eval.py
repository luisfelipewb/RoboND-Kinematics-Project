from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
import numpy as np

def tf_matrix(a, alpha, d, q):
    TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [                0,                 0,           0,             1]])
    return TF

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

T0_1 = tf_matrix(a0, alpha0, d1, q1).subs(s)
T1_2 = tf_matrix(a1, alpha1, d2, q2).subs(s)
T2_3 = tf_matrix(a2, alpha2, d3, q3).subs(s)
T3_4 = tf_matrix(a3, alpha3, d4, q4).subs(s)
T4_5 = tf_matrix(a4, alpha4, d5, q5).subs(s)
T5_6 = tf_matrix(a5, alpha5, d6, q6).subs(s)
T6_g = tf_matrix(a6, alpha6, dg, qg).subs(s)

Trx = Matrix([[ 1,       0,         0, 0],
              [ 0,  cos(pi), -sin(pi), 0],
              [ 0,  sin(pi),  cos(pi), 0],
              [ 0,        0,        0, 1]])

Try = Matrix([[  cos(pi/2), 0, sin(pi/2), 0],
              [          0, 1,         0, 0],
              [ -sin(pi/2), 0, cos(pi/2), 0],
              [          0, 0,         0, 1]])

Trot = (Trx * Try)

print ("T0_1", T0_1)
print ("T1_2", T1_2)
print ("T2_3", T2_3)
print ("T3_4", T3_4)
print ("T4_5", T4_5)
print ("T5_6", T5_6)
print ("T6_g", T6_g)
print ("Trx", Trx)
print ("Try", Try)
print ("Trot", Trot)

T0_2 = (T0_1 * T1_2)
#print ("EVAL T0_2")
#print(T0_2.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))

T0_3 = (T0_2 * T2_3)
#print ("EVAL  T0_3")
#print(T0_3.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))

T0_5 = (T0_3 * T3_4 * T4_5)
#print ("EVAL  T0_5")
#print(T0_5.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))

T0_g = (T0_5 * T5_6 * T6_g * Trot)
print ("EVAL  T0_g")
print(T0_g.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))

print(T0_g.evalf(subs={q1:0.48, q2:1.03, q3:-1.24, q4:-0.49, q5:-0.52, q6:2.81}))

#T0_g = simplify (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_g * Trot)
#print ("T0_g", T0_g)
#print(T0_g.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))


