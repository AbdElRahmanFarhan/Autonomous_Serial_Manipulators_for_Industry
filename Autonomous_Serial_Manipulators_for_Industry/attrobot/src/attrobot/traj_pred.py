#! /usr/bin/env python

from scipy.integrate import RK45
import numpy as np
import math
import timeit

m = float(2.7*math.pow(10,-3))
Cd = float(0.54)
Cm = float(0.069)
r = float(20.0*math.pow(10, -3))
g = np.zeros(shape=[3], dtype=float)
# changed
g[:] = [0.0, 0, -9.81]
row = float(1.184)
pi = float(math.pi)
W0 = np.zeros(shape=[3], dtype=float)
V0 = np.zeros(shape=[3], dtype=float)
S0 = np.zeros(shape=[3], dtype=float)

t0 = 0
p_count = 0
v_count = 0


def acc(t, Vn):

    global W0
    a_m = m * g - 0.5 * Cd * row * pi * math.pow(r, 2) * np.linalg.norm(Vn) * Vn \
        + (4.0 / 3.0) * Cm * pi * math.pow(r, 3) * row * np.cross(W0, Vn) * (2.0 * pi / 60.0)

    return a_m/m


def dist(t, s, v, w):

    global V0, S0, W0
    S0 = s
    W0 = w
    V0 = v
    var = RK45(vel, 0, S0, t, max_step=(1.0/20.0))
    return var


def vel(t, Vn):

    global t0, V0

    v1 = RK45(acc, t0, V0, t)

    while v1.status != "finished":
        v1.step()
    V0 = v1.y
    t0 = t
    return V0


def mains(mode, s_0, v_0, w_0):
    global t0

    t0 = 0
    start = timeit.default_timer()
    xyz_list = []
    var = dist(2.0, s_0, v_0, w_0)

    while var.status != "finished":

        var.step()
        xyz_list.append(var.y)
        if mode == 'table':
            if var.y[2] <= 0.1:
                break
        elif mode == 'racket':
            if var.y[0] <= 0.1:
                break
        elif mode == 'free':
            if var.y[2] <= 0:
                break
        else:
            print('ERROR::Invalid Mode Input')
            return
    stop = timeit.default_timer()
    #print(stop-start)
    xyz = np.asarray(xyz_list, dtype=float)
    # xyz.reshape(xyz,(int(p_count),3))

    return xyz, var.y, V0
