#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import plotly.plotly as py
import plotly.graph_objs as go
from scipy.interpolate import interp1d
from scipy import signal


def inter_and_smooth(horizontal_axis, vertical_axis, res, inter_kind, final_point):
    inter_y_t = interp1d(horizontal_axis, vertical_axis, kind=inter_kind)
    t_new = np.linspace(0, final_point, num=res, endpoint=True)
    if inter_kind == 'linear':
        n = 1
    elif inter_kind == 'quadratic':
        n = 2
    elif inter_kind == 'cubic':
        n = 3
    smoothed_result = go.Scatter(
        x=t_new,
        y=signal.savgol_filter(inter_y_t(t_new), res, n),
        mode='markers',
        marker=dict(
            size=6,
            color='#C190F0',
            symbol='triangle-up'
        ),
        name='Savitzky-Golay'
    )
    return smoothed_result


def index_query(time_axis, value):

    for point in range(len(time_axis)):

        if time_axis[point] >= value:
            return point


def post_process(x_points, y_points, z_points, time):


    time = np.divide(np.subtract(time,time[0]), 1000.0)
    time = np.cumsum(time)
    #print(time)
    print("REAL")
    print("X")
    print(x_points)
    print("Y")
    y_points=np.flip(y_points)
    print(y_points)
    #print(z_points)
    resolution = 45
    final_time = time[-1] -0.001
    inter_type_y = 'quadratic'
    inter_type_x = 'linear'
    inter_type_z = 'linear'
    y_t = inter_and_smooth(time, y_points, resolution, inter_type_y, final_time)
    x_t = inter_and_smooth(time, x_points, resolution, inter_type_x, final_time)
    z_t = inter_and_smooth(time, z_points, resolution, inter_type_z, final_time)
    
    fig1 = plt.figure(1)
    plt.plot(x_t.y,y_t.y)
    print("Post Processed")
    print("X")
    print(x_t.y)
    print("Y")
    print(y_t.y)
    plt.plot(x_points,y_points)
    plt.show()

    index = index_query(y_t.x, final_time)
    v_x = (x_t.y[index] - x_t.y[index-1])/(x_t.x[index] - x_t.x[index-1])
    v_y = (y_t.y[index] - y_t.y[index-1])/(y_t.x[index] - y_t.x[index-1])
    v_z = (z_t.y[index] - z_t.y[index-1])/(z_t.x[index] - z_t.x[index-1])

    return np.array([x_t.y[index],y_t.y[index],z_t.y[index]]), np.array([v_x,v_y,v_z])

# print(v_x)
# print(v_y)
# print(v_z)
#
# print(x_t.y[index])
# print(y_t.y[index])
# print(z_t.y[index])
# plt.figure(0)
# # plt.plot(y_t.x,y_t.y)
# # plt.plot(time,y_points)
# plt.plot(y_t.x[index:],y_t.y[index:])
# plt.scatter(time, y_points)
# plt.show()
