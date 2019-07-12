#! /usr/bin/env python
# steps for deciding the racket orientation
##  know the desired point and orientation for the racket
# 1 Y-X-Z euler representation is used to represent the desired point and orientation
# 2

import numpy as np
import math

et = 0.93  # elastic rebound
mu = 0.25  # coefficient of friction
er = 0.81
#Vb = np.zeros((3, 1))  # velocity of the ball, to be determined from the ball trajectory model
#Wb = np.zeros((3, 1))  # angular velocity of the ball, to be determined from the ball trajectory model
Av_a = np.zeros((3, 3))
Bv_a = np.zeros((3, 3))
Aw_a = np.zeros((3, 3))
Bw_a = np.zeros((3, 3))
kp = 0.0019
m = 0.0027
r = 0.02

def Coefficients(a,mode):
    global Av_a,Bv_a,Aw_a,Bw_a,et,er,r

    if mode == 'table':
        e = et
    elif mode == 'racket':
        e = er

    Av_a[0, 0] = 1.0 - a
    Av_a[1, 1] = 1.0 - a
    Av_a[2, 2] = -e

    Bv_a[0, 1] = a*r
    Bv_a[1, 0] = -a*r

    Aw_a[0, 1] = -(3.0 * a) / (2.0*r)
    Aw_a[1, 0] = (3.0 * a) / (2.0*r)

    Bw_a[0, 0] = 1.0 - ((3.0 * a) / 2.0)
    Bw_a[1, 1] = 1.0 - ((3.0 * a) / 2.0)
    Bw_a[2, 2] = 1.0
    return

def Rotation(roll, pitch, yaw): # one of these angles may be redundant.

    #alpha and beta are degree inputs
    #this function is used to transform points from the base to the racket frame.
    #These are post multiplied so they are about the current frame
    #post multiblication only considers the rotation matrices' order with respect to each other
    #   not with respect to the rotated point, in fact there postion with respect to the rotated
    #   point is pre if(3,1) and post if(1,3) to be ableto dot product :D. as a physical meaning
    #   it means that we rotate the point in reverse rotations
    #roll : rotation about x - axis
    #pitch : rotation about y - axis
    #yaw : rotation about z - axis

    roll_rad = math.pi * roll / 180.0
    pitch_rad = math.pi * pitch / 180.0
    yaw_rad = math.pi * yaw / 180.0

    Rotx = np.array([[1,0,0],
                     [0, math.cos(roll_rad), - math.sin(roll_rad)],
                     [0, math.sin(roll_rad), math.cos(roll_rad)]])

    Roty = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)],
                     [0, 1, 0],
                     [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])

    Rotz = np.array([[math.cos(yaw_rad),- math.sin(yaw_rad), 0],
                     [math.sin(yaw_rad),math.cos(yaw_rad),0],
                     [0, 0, 1]])


    Rot = np.dot(Rotz , np.dot(Roty , Rotx ))
    print(Rot)
    return Rot

def impactModel(mode, Vb, Wb, roll=0, pitch=0,yaw=0, Vr=0):


    # the z-axis is the normal direction of the table
    # the x-axis is along the length of the table
    # the y-axis along the depth

    Vb = np.array([[Vb[0]], [Vb[1]], [Vb[2]]])
    Wb = np.array([[Wb[0]], [Wb[1]], [Wb[2]]]) * (2*math.pi/60.0)

    if mode == 'table':
         Vs = 1.0 - (5.0/2.0)* mu * (1.0 + et) * (abs(float(Vb[2])) / np.linalg.norm([Vb[0],Vb[1]]))
         if Vs > 0:
            a = mu * (1.0 + et) * (abs(float(Vb[2]))/np.linalg.norm([Vb[0], Vb[1]]))
         else:
            a = 2.0 / 5.0
         Coefficients(a,mode)
         print(Vb)
         print(Wb*(60.0/(2*math.pi)))
         Vb_new = np.dot(Av_a, Vb) + np.dot(Bv_a, Wb)
         Wb_new = np.dot(Aw_a, Vb) + np.dot(Bw_a, Wb)
         return Vb_new, Wb_new

    elif mode == 'racket':
         a = kp / m
         Coefficients(a,mode)
         R = Rotation(roll, pitch, yaw)
         print(Vb)
         print(np.dot(R.T,Vb))
         #print(Wb*(60.0/(2*math.pi)))
         Vb_new = np.dot(np.dot(R, Av_a), np.dot(R.T, (Vb - Vr))) + np.dot(np.dot(R, Bv_a), np.dot(R.T, Wb)) + Vr
         Wb_new = np.dot(np.dot(R, Aw_a), np.dot(R.T, (Vb - Vr))) + np.dot(np.dot(R, Bw_a), np.dot(R.T, Wb)) # prove these two equations by testing
         return Vb_new, Wb_new

    else:
        print('error no such option called: ', mode)
        return








