#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp
from sympy import *
from math import pi

# To make display prety
#printing.init_printing(use_latex = True)

alpha = Symbol("alpha")
beta = Symbol("beta")
gamma = Symbol("gamma")
x = Symbol("x")

# Rotation matrices
R_z = Matrix([[cos(alpha), -sin(alpha), 0],
          [sin(alpha), cos(alpha), 0],
          [0, 0, 1]])


R_y = Matrix([[cos(beta), 0, sin(beta)],
          [0, 1, 0],
          [-sin(beta), 0, cos(beta)]])


R_x = Matrix([[1, -0, 0],
          [0, cos(gamma), -sin(gamma)],
          [0, sin(gamma), cos(gamma)]])

# Translation Matirx
T_x = Matrix([[x], [0], [0]])
T_x = eye(3).row_join(T_x)
uniform = Matrix([[0,0,0, 1]])

# Transforms
theta_1 = Symbol("theta_1")
theta_2 = Symbol("theta_2")
theta_3 = Symbol("theta_3")
r_1 = Symbol("r_1")
r_2 = Symbol("r_2")
r_3 = Symbol("r_3")

A01 = (R_x*R_y).row_join(Matrix([[0],[0],[0]])).col_join(uniform).replace(gamma, pi/2).replace(beta, theta_1)
A12 = (R_z*T_x).col_join(uniform).replace(alpha, theta_2).replace(x, r_2)
A23 = (R_z*T_x).col_join(uniform).replace(alpha, theta_3).replace(x, r_3)

preview(A01, viewer='file', filename="fk_imgs/fk_A01.png", dvioptions=['-D','300'])
preview(A12, viewer='file', filename="fk_imgs/fk_A12.png", dvioptions=['-D','300'])
preview(A23, viewer='file', filename="fk_imgs/fk_A23.png", dvioptions=['-D','300'])

A03 = A01*A12*A23
preview(A03, viewer='file', filename="fk_imgs/fk_A03.png", dvioptions=['-D','300'])

A01simp = trigsimp(A01)
A12simp = trigsimp(A12)
A23simp = trigsimp(A23)
A03simp = trigsimp(A03)
preview(A01simp, viewer='file', filename="fk_imgs/fk_A01_simp.png", dvioptions=['-D','300'])
preview(A12simp, viewer='file', filename="fk_imgs/fk_A12_simp.png", dvioptions=['-D','300'])
preview(A23simp, viewer='file', filename="fk_imgs/fk_A23_simp.png", dvioptions=['-D','300'])
preview(A03simp, viewer='file', filename="fk_imgs/fk_A03_simp.png", dvioptions=['-D','300'])


A01simp_eval = A01simp.replace(r_1, 0).replace(r_2, 1).replace(r_3, 1)
A12simp_eval = A12simp.replace(r_1, 0).replace(r_2, 1).replace(r_3, 1)
A23simp_eval = A23simp.replace(r_1, 0).replace(r_2, 1).replace(r_3, 1)
A03simp_eval = A03simp.replace(r_1, 0).replace(r_2, 1).replace(r_3, 1)
preview(A01simp_eval, viewer='file', filename="fk_imgs/fk_A01_simp_eval.png", dvioptions=['-D','300'])
preview(A12simp_eval, viewer='file', filename="fk_imgs/fk_A12_simp_eval.png", dvioptions=['-D','300'])
preview(A23simp_eval, viewer='file', filename="fk_imgs/fk_A23_simp_eval.png", dvioptions=['-D','300'])
preview(A03simp_eval, viewer='file', filename="fk_imgs/fk_A03_simp_eval.png", dvioptions=['-D','300'])


s1 = sin(theta_1)
s2 = sin(theta_2)
c1 = cos(theta_1)
c2 = cos(theta_2)
s23 = sin(theta_2 + theta_3)
c23 = cos(theta_2 + theta_3)

DH_Matrix_Generic = Matrix([[c1*c23, -s23*c1, s1, c1*(r_2*c2 + r_3*c23)],
                            [s1*c23, -s1*s23, -c1, s1*(r_2*c2 + r_3*c23)],
                            [s23, c23, 0, r_2*s2 + r_3*s23],
                            [0,0,0,1]])

DH = simplify(DH_Matrix_Generic)

# Save to local File
r1 = 0.0
r2 = 1.0
r3 = 1.0

preview(DH, viewer='file', filename="fk_imgs/DH.png", dvioptions=['-D','300'])

theta1 = 2.356194490192345
theta2 = 0.5074842211955768
theta3 = -2.2459278597319283

DH_eval = DH.replace(theta_1, theta1).replace(theta_2, theta2).replace(theta_3, theta3).replace(r_1, r1).replace(r_2, r2).replace(r_3, r3)
preview(DH_eval, viewer='file', filename="fk_imgs/DH_eval1.png", dvioptions=['-D','300'])


theta1 = 0
theta2 = 0
theta3 = 0

DH_eval = DH.replace(theta_1, theta1).replace(theta_2, theta2).replace(theta_3, theta3).replace(r_1, r1).replace(r_2, r2).replace(r_3, r3)
preview(DH_eval, viewer='file', filename="fk_imgs/DH_eval2.png", dvioptions=['-D','300'])