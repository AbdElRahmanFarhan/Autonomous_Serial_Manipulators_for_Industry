#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
# import plotly.plotly as py
# import plotly.graph_objs as go
from scipy.interpolate import interp1d
from scipy import signal
from scipy.optimize import curve_fit


def fit_func_y(x, a, b, c):
    return a * x ** 2 + b * x + c


def fit_func_xz(x, a, b):
    return a * x + b


def inter_and_smooth(horizontal_axis, vertical_axis, res, inter_kind, final_point):
    inter_y_t = interp1d(horizontal_axis, vertical_axis, kind=inter_kind, copy=True)
    t_new = np.linspace(0, final_point, num=res, endpoint=False)
    if inter_kind == 'linear':
        n = 1
    elif inter_kind == 'quadratic':
        n = 2
    elif inter_kind == 'cubic':
        n = 3
    # smoothed_result = go.Scatter(
    #     x=t_new,
    #     y=signal.savgol_filter(inter_y_t(t_new), res, n, mode='interp'),
    #     mode='markers',
    #     marker=dict(
    #         size=6,
    #         color='#C190F0',
    #         symbol='triangle-up'
    #     ),
    #     name='Savitzky-Golay'
    # )
    return inter_y_t, t_new


def inter_and_smooth_coord(horizontal_axis, vertical_axis, res, inter_kind, final_point):
    inter_y_t = interp1d(horizontal_axis, vertical_axis, kind=inter_kind, copy=True)
    t_new = np.linspace(0, final_point, num=res, endpoint=False)
    if inter_kind == 'linear':
        n = 1
    elif inter_kind == 'quadratic':
        n = 2
    elif inter_kind == 'cubic':
        n = 3
    # smoothed_result = go.Scatter(
    #     x=t_new,
    #     y=signal.savgol_filter(inter_y_t(t_new), res, n, mode='interp'),
    #     mode='markers',
    #     marker=dict(
    #         size=6,
    #         color='#C190F0',
    #         symbol='triangle-up'
    #     ),
    #     name='Savitzky-Golay'
    # )
    y = signal.savgol_filter(inter_y_t(t_new), 7, n, mode='nearest')
    return y, t_new


def myfit_coords_base(horizontal_axis, vertical_axis, res, final_point, n, acc=-np.inf, vel = np.inf):
    t_new = np.linspace(0, final_point, num=res, endpoint=False)
    # trend=np.polyfit(horizontal_axis,vertical_axis, n)
    if n==2 : # y_axis(cam) or z_axis(base)
        trend, _ = curve_fit(fit_func_y, horizontal_axis, vertical_axis, bounds=([-9.8101/2, -np.inf, -np.inf], [-9.81/2, np.inf, np.inf]))
    elif n == 1: # x_axis or y_axis(base)
        trend, _ = curve_fit(fit_func_xz, horizontal_axis, vertical_axis, bounds=([-np.inf, -np.inf], [vel, np.inf]))
    elif n == 3: # z_axis(cam) or x_axis(base)
        trend, _ = curve_fit(fit_func_xz, horizontal_axis, vertical_axis, bounds=([-np.inf, -np.inf], [vel, np.inf]))

    trendpoly = np.poly1d(trend)
    return trendpoly, trend, t_new


def myfit_coords_cam(horizontal_axis, vertical_axis, res, final_point, n, acc=-np.inf, vel = np.inf):
    t_new = np.linspace(0, final_point, num=res, endpoint=False)
    # trend=np.polyfit(horizontal_axis,vertical_axis, n)
    if n==2 : # y_axis(cam)
        trend, _ = curve_fit(fit_func_y, horizontal_axis, vertical_axis, bounds=([9.81/2, -np.inf, -np.inf], [9.8101/2, np.inf, np.inf]))
    elif n == 1: # x_axis(cam)
        trend, _ = curve_fit(fit_func_xz, horizontal_axis, vertical_axis, bounds=([-np.inf, -np.inf], [vel, np.inf]))
    elif n == 3: # z_axis(cam)
        trend, _ = curve_fit(fit_func_xz, horizontal_axis, vertical_axis, bounds=([-np.inf, -np.inf], [vel, np.inf]))

    trendpoly = np.poly1d(trend)
    return trendpoly, trend, t_new


def myfit_pred(horizontal_axis, vertical_axis, res, final_point, n, acc=-np.inf, vel = np.inf):
    t_new = np.linspace(0, final_point, num=res, endpoint=False)
    # trend=np.polyfit(horizontal_axis,vertical_axis, n)
    if n==2 : # y_axis(cam)
        trend, _ = curve_fit(fit_func_y, horizontal_axis, vertical_axis, bounds=([0, -np.inf, -np.inf], [np.inf, np.inf, np.inf]))
    elif n == 1: # x_axis(cam)
        trend, _ = curve_fit(fit_func_xz, horizontal_axis, vertical_axis, bounds=([-np.inf, -np.inf], [vel, np.inf]))
    elif n == 3: # z_axis(cam)
        trend, _ = curve_fit(fit_func_xz, horizontal_axis, vertical_axis, bounds=([-np.inf, -np.inf], [vel, np.inf]))

    trendpoly = np.poly1d(trend)
    return trendpoly, trend, t_new





def index_query(time_axis, value):

    for point in range(len(time_axis)):

        if time_axis[point] >= value:
            return point


# def post_process(x_points, y_points, time):
#
#     time = (time - time[0])/1000.0
#
#     resolution_interp = 55
#     final_time = time[-1]
#     print("final time= ", final_time)
#     inter_type_y = 'quadratic'
#     inter_type_x = 'linear'
#
#     y_t, t_new = inter_and_smooth(time, y_points, resolution_interp, inter_type_y, final_time)
#     x_t, t_new = inter_and_smooth(time, x_points, resolution_interp, inter_type_x, final_time)
#
#     fig1 = plt.figure(1)
#     x_pp=x_t(t_new)
#     y_pp = y_t(t_new)
#     print(x_pp, y_pp)
#     plt.plot(x_points, -1*y_points, '-')
#     # plt.plot( x_pp, y_pp, '-', x_points, y_points, 'o')
#     plt.title('Pixels')
#     plt.legend(loc='upper left')
#     plt.show()
#
#     final_time = t_new[-1]/4
#     index = index_query(time, final_time)
#     print("this is index found", index)
#     v_x = (x_pp[index] - x_pp[index-1])/(x_pp[index] - x_pp[index-1])
#     v_y = (y_pp[index] - y_pp[index-1])/(y_pp[index] - y_pp[index-1])
#
#     return np.array([x_pp[index], y_pp[index], 0]), np.array([v_x, v_y, 0]), x_pp,y_pp


def post_process_coords(x_points, y_points, z_points, time, base):

    time = (time - time[0])/1000.0

    print("time = ", time)
    resolution_interp =255
    final_time = time[-1]
    prediction_time_step = final_time/resolution_interp

    # i = z_points.shape[0] - 2
    # i = 1
    # v_x = (x_points[i] - x_points[i-1])/(time[i] - time[i-1])
    # v_y = (y_points[i] - y_points[i-1])/(time[i] - time[i-1])
    # v_z = (z_points[i] - z_points[i-1])/(time[i] - time[i-1])
    #
    # v_x_2 = (x_points[i+1] - x_points[i])/(time[i+1] - time[i])
    # v_y_2 = (y_points[i+1] - y_points[i])/(time[i+1] - time[i])
    # v_z_2 = (z_points[i+1] - z_points[i])/(time[i+1] - time[i])
    #
    # print("v_x_1 = {0}, v_x_2 = {1}".format(v_x,v_x_2))
    # print("v_y_1 = {0}, v_y_2 = {1}".format(v_y,v_y_2))
    # a_t_1 = (time[i] + time[i-1]) / 2
    # a_t_2 = (time[i+1] + time[i]) / 2
    # a_x = (v_x_2-v_x) / (a_t_2 - a_t_1)
    # a_y = (v_y_2-v_y) / (a_t_2 - a_t_1)
    # a_z = (v_z_2-v_z) / (a_t_2 - a_t_1)
    #
    # print("a_z = ", a_z)

    # inter_type_y = 'linear'
    # inter_type_x = 'linear'
    # inter_type_z = 'linear'

    # y_t, t_new_inter = inter_and_smooth(time, y_points, resolution_interp, inter_type_y, final_time)
    # x_t, t_new_inter = inter_and_smooth(time, x_points, resolution_interp, inter_type_x, final_time)
    # z_t, t_new_inter = inter_and_smooth(time, z_points, resolution_interp, inter_type_z, final_time)
    #
    # fig1 = plt.figure(222)
    # plt.plot(x_t(t_new_inter), y_t(t_new_inter), '-')
    # plt.title('Distance_inter')
    # plt.legend(loc='upper left')
    # plt.show()

    if base :
        trendpoly_y, trend_y, t_new = myfit_coords_base(time, y_points, resolution_interp, final_time, 1)
        trendpoly_x, trend_x, t_new = myfit_coords_base(time, x_points, resolution_interp, final_time, 3)
        trendpoly_z, trend_z, t_new = myfit_coords_base(time, z_points, resolution_interp, final_time, 2)
    else :
        trendpoly_y, trend_y, t_new = myfit_coords_cam(time, y_points, resolution_interp, final_time, 2)
        trendpoly_x, trend_x, t_new = myfit_coords_cam(time, x_points, resolution_interp, final_time, 3)
        trendpoly_z, trend_z, t_new = myfit_coords_cam(time, z_points, resolution_interp, final_time, 1)

    # if from base :


    # plt.plot(t_new, trendpoly_y(t_new))
    # print("trend_y = ",trend_y)


    # fig2 = plt.figure(1)
    # plt.plot(trendpoly_x(t_new), -1*trendpoly_y(t_new), '-', x_points, -1*y_points, 'o')
    # plt.title('Distance')
    # plt.legend(loc='upper left')
    # plt.show()

    x_t=trendpoly_x(t_new)
    y_t=trendpoly_y(t_new)
    z_t=trendpoly_z(t_new)

    sz = len(time)
    index = index_query(t_new, time[sz/3])
    # index=index/2
    index = 150
    print("the found index is", index)


    v_x = (x_t[index] - x_t[index-1])/(t_new[index] - t_new[index-1])
    v_y = (y_t[index] - y_t[index-1])/(t_new[index] - t_new[index-1])
    v_z = (z_t[index] - z_t[index-1])/(t_new[index] - t_new[index-1])

    v_x_2 = (x_t[index+1] - x_t[index])/(t_new[index+1] - t_new[index])
    v_y_2 = (y_t[index+1] - y_t[index])/(t_new[index+1] - t_new[index])
    v_z_2 = (z_t[index+1] - z_t[index])/(t_new[index+1] - t_new[index])

    a_x = (v_x_2-v_x)/(t_new[index+1]-t_new[index])
    a_y = (v_y_2-v_y)/(t_new[index+1]-t_new[index])
    a_z = (v_z_2-v_z)/(t_new[index+1]-t_new[index])

    print("[{}, {}, {}]".format(a_x, a_y, a_z))

    # return np.array([x_pp[index],y_pp[index],z_pp[index]]), np.array([v_x, v_y, v_z])
    return np.array([x_t[index], y_t[index], z_t[index]]), np.array([v_x_2, v_y_2, v_z_2]), np.array([a_x, a_y, a_z]), x_t, y_t, z_t, prediction_time_step
    # return np.array([x_points[i+1], y_points[i+1], z_points[i+1]]), np.array([v_x_2, v_y_2, v_z_2]), np.array([a_x, a_y, a_z]), x_t, y_t, z_t, prediction_time_step
