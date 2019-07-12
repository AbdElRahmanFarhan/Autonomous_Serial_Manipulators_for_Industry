#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft
import rospy
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from attrobot.msg import control_params, ball_trajectory
from math import pi, floor, fabs
from std_msgs.msg import Bool
import pandas as pd
from scipy.optimize import minimize_scalar, differential_evolution, minimize
import os
import datetime


def Read_Loggings():

    directory = "/home/abd-alrahman/catkin_ws/src/attrobot/plotting/port_6510_log.csv"
    # theta columns
    thetas_dtype = {"R_FB_POS_RJ1": float, "R_FB_POS_RJ2": float, "R_FB_POS_RJ3": float, "R_FB_POS_RJ4": float,
                    "R_FB_POS_RJ5": float, "R_FB_POS_RJ6": float}
    thetas_usecols = ['R_FB_POS_RJ1', 'R_FB_POS_RJ2', 'R_FB_POS_RJ3', 'R_FB_POS_RJ4', 'R_FB_POS_RJ5', 'R_FB_POS_RJ6']

    # dtheta columns
    dthetas_dtype = {"R_FB_VEL_RJ1": float, "R_FB_VEL_RJ2": float, "R_FB_VEL_RJ3": float, "R_FB_VEL_RJ4": float,
                     "R_FB_VEL_RJ5": float, "R_FB_VEL_RJ6": float}
    dthetas_usecols = ['R_FB_VEL_RJ1', 'R_FB_VEL_RJ2', 'R_FB_VEL_RJ3', 'R_FB_VEL_RJ4', 'R_FB_VEL_RJ5', 'R_FB_VEL_RJ6']

    # time columns
    time_dtype = {"TIMESTAMP": float}
    time_usecols = ['TIMESTAMP']

    thetas = pd.read_csv(directory, dtype=thetas_dtype, usecols=thetas_usecols)
    dthetas = pd.read_csv(directory, dtype=dthetas_dtype, usecols=dthetas_usecols)
    time = pd.read_csv(directory, dtype=time_dtype, usecols=time_usecols)
    return time.values, thetas.values, dthetas.values

def Log_Data(time_to_log, thetas_to_log, dthetas_to_log, thetas_set_points_log, dthetas_set_points_log, k, lp, max_speed_dev, ISE, directory):

    print("======================RECORDING======================")
    dataset = pd.DataFrame({('R_FB_POS_RJ1_'): thetas_to_log[:, 0],
                            ('R_FB_POS_RJ2_'): thetas_to_log[:, 1],
                            ('R_FB_POS_RJ3_'): thetas_to_log[:, 2],
                            ('R_FB_POS_RJ4_'): thetas_to_log[:, 3],
                            ('R_FB_POS_RJ5_'): thetas_to_log[:, 4],
                            ('R_FB_POS_RJ6_'): thetas_to_log[:, 5],
                            ('R_FB_VEL_RJ1_'): dthetas_to_log[:, 0],
                            ('R_FB_VEL_RJ2_'): dthetas_to_log[:, 1],
                            ('R_FB_VEL_RJ3_'): dthetas_to_log[:, 2],
                            ('R_FB_VEL_RJ4_'): dthetas_to_log[:, 3],
                            ('R_FB_VEL_RJ5_'): dthetas_to_log[:, 4],
                            ('R_FB_VEL_RJ6_'): dthetas_to_log[:, 5],
                            ('R_SP_POS_RJ1_'): thetas_set_points_log[:, 0],
                            ('R_SP_POS_RJ2_'): thetas_set_points_log[:, 1],
                            ('R_SP_POS_RJ3_'): thetas_set_points_log[:, 2],
                            ('R_SP_POS_RJ4_'): thetas_set_points_log[:, 3],
                            ('R_SP_POS_RJ5_'): thetas_set_points_log[:, 4],
                            ('R_SP_POS_RJ6_'): thetas_set_points_log[:, 5],
                            ('R_SP_VEL_RJ1_'): dthetas_set_points_log[:, 0],
                            ('R_SP_VEL_RJ2_'): dthetas_set_points_log[:, 1],
                            ('R_SP_VEL_RJ3_'): dthetas_set_points_log[:, 2],
                            ('R_SP_VEL_RJ4_'): dthetas_set_points_log[:, 3],
                            ('R_SP_VEL_RJ5_'): dthetas_set_points_log[:, 4],
                            ('R_SP_VEL_RJ6_'): dthetas_set_points_log[:, 5],
                            ('TIMESTAMP_'): time_to_log,
                            ('K'): k,
                            ('LP'): lp,
                            ('max_speed_dev'): max_speed_dev,
                            ('ISE'): ISE
                            })
    print("======================WRITING======================")
    dataset.to_csv(directory)

    return

def Optimize (df):

    global  thetas_set_points_raw, dthetas_set_points_raw, sz_setpoints_raw, folder_str, counter,array_size, sampling_time
    # optimization step
    lp = 0
    k = 0.0
    damping_factor = df
    max_speed_dev = damping_factor * (180 / pi) * 0.5
    # max_speed_dev = 400
    control_parameters = control_params()
    control_parameters.k = k
    control_parameters.lp = lp
    control_parameters.max_speed_dev = max_speed_dev
    while (not rospy.is_shutdown()):
        # wait for egm node to send this msg
        print("=============Starting optimization======================")
        start_optimization = rospy.wait_for_message("/start_optimization", Bool)
        rospy.sleep(1)
        # publish control parameters
        print("=============Publishing the parameters======================")
        control_pub.publish(control_parameters)

        print("=============Waiting for egm motion to stop======================")
        read_logginigs = rospy.wait_for_message("/read_loggings", Bool)
        # read loggings from the file
        _, thetas, dthetas = Read_Loggings()
        # clip logging data

        if dthetas.shape[0] == 0 or thetas.shape[0] == 0:
            print ("file is empty")
            continue

        end_indices = np.where(np.fabs(dthetas) <= 0.2)[0]
        start_indices = np.where(np.fabs(dthetas) >= 0.2)[0]
        if (end_indices.size == 0 or start_indices.size == 0):
            print ("file is empty")
            continue
        end_index = np.max(end_indices)
        start_index = np.min(start_indices)
        if fabs(start_index - end_index) <= 10:
            print ("file is empty")
            continue

        main_time = np.arange(array_size) * sampling_time
        thetas_loggings_raw = thetas[start_index:end_index]
        dthetas_loggings_raw = dthetas[start_index:end_index]
        sz_loggings_raw = thetas_loggings_raw.shape[0]
        thetas_set_points = np.zeros((array_size, 6))
        dthetas_set_points = np.zeros((array_size, 6))
        thetas_loggings = np.zeros((array_size, 6))
        dthetas_loggings = np.zeros((array_size, 6))

        thetas_set_points[0:sz_setpoints_raw, :] = thetas_set_points_raw
        thetas_set_points[sz_setpoints_raw:, :] = thetas_set_points_raw[-1, :] * np.ones(
            (array_size - sz_setpoints_raw, 6))

        dthetas_set_points[0:sz_setpoints_raw, :] = dthetas_set_points_raw
        dthetas_set_points[sz_setpoints_raw:, :] = dthetas_set_points_raw[-1, :] * np.ones(
            (array_size - sz_setpoints_raw, 6))

        if (sz_loggings_raw <= array_size):
            thetas_loggings[0:sz_loggings_raw, :] = thetas_loggings_raw
            thetas_loggings[sz_loggings_raw:, :] = thetas_loggings_raw[-1, :] * np.ones(
                (array_size - sz_loggings_raw, 6))

            dthetas_loggings[0:sz_loggings_raw, :] = dthetas_loggings_raw
            dthetas_loggings[sz_loggings_raw:, :] = dthetas_loggings_raw[-1, :] * np.ones(
                (array_size - sz_loggings_raw, 6))
        else:
            thetas_loggings = thetas_loggings_raw[:array_size, :]
            dthetas_loggings = dthetas_loggings_raw[:array_size, :]

        control_str = str(lp)+'_'+str(k)+'_'+str(max_speed_dev)
        counter_str = "itr_" + str(counter)
        ISE = np.sum(np.trapz(((thetas_set_points - thetas_loggings)**2), dx = sampling_time, axis= 0)) / 6
        Log_Data(main_time, thetas_loggings, dthetas_loggings, thetas_set_points,
                 dthetas_set_points, k, lp, max_speed_dev, ISE,"/home/abd-alrahman/catkin_ws/src/attrobot/plotting/"+folder_str+"/"+counter_str+".csv")
        print ("----------------------------")
        print ("ISE = ", ISE)
        print ("----------------------------")
        print ("k = ", k)
        print ("damping_factor = ", damping_factor)
        counter = counter + 1
        break
    return ISE

# ros initialization
rospy.init_node("control_optimization")
set_points = rospy.wait_for_message("/robot_trajectory_planning_fixed", JointTrajectory)
control_pub = rospy.Publisher("/control_parameters", control_params, queue_size=2)

traj_point = JointTrajectoryPoint()
pos = []
vel = []
acc = []
time_algo = []
for p in range(len(set_points.points)):
    traj_point = set_points.points[p]
    pos.append(traj_point.positions)
    vel.append(traj_point.velocities)
    acc.append(traj_point.accelerations)
    sec = traj_point.time_from_start.secs
    nsec = traj_point.time_from_start.nsecs
    time_algo.append((sec + nsec * 10 ** -9))

thetas_set_points_raw = np.row_stack(pos) * (180 / pi)
dthetas_set_points_raw = np.row_stack(vel) * (180 / pi)
ddthetas_set_points_raw = np.row_stack(acc) * (180 / pi)
time_set_points_raw = np.array(time_algo)
sz_setpoints_raw = thetas_set_points_raw.shape[0]
final_time = time_set_points_raw[-1]
opt_duration = final_time + 3
sampling_time = 0.004
array_size = int(opt_duration / sampling_time)
counter = 0




# logging
time_now = datetime.datetime.now()
time_str = str(time_now.hour) + '_' + str(time_now.minute) + '_' + str(time_now.second)
theta_str = str(floor(thetas_set_points_raw[-1][0])) + '_'+ str(floor(thetas_set_points_raw[-1][1])) + '_'+ str(floor(thetas_set_points_raw[-1][2]))
folder_str = theta_str + '_' + time_str
os.mkdir("/home/abd-alrahman/catkin_ws/src/attrobot/plotting/"+folder_str+"/")

damping_factor = 1
ISE = Optimize(damping_factor)
# result = minimize(fun= Optimize, x0= [0.1, 0.98] ,bounds=[[0, 1], [0.96, 1]], options= {"maxiter" : 200, "disp" : True})
# print "-----------optimization result---------------"
# print result.x

# while(not rospy.is_shutdown()):
# k = 0.0033977325701527194
#     print ("-------------------------------------------")
#     print ("ISE = ", ISE)
#     if lp == 20:
#         break
#     lp = lp + 1






