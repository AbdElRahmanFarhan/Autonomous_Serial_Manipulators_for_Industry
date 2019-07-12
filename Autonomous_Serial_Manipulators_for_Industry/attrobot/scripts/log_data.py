#! /usr/bin/env python

import numpy as np

import rospy
import sys

from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initializer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import pi, floor, fabs
from std_msgs.msg import Bool
import pandas as pd
import os
import datetime
from attrobot.msg import ball_trajectory


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

    cartesian_dtype = {"R_FB_POS_X": float, "R_FB_POS_Y": float, "R_FB_POS_Z": float}
    cartesian_usecols = ['R_FB_POS_X', 'R_FB_POS_Y', 'R_FB_POS_Z']

    # time columns
    time_dtype = {"TIMESTAMP": float}
    time_usecols = ['TIMESTAMP']

    thetas = pd.read_csv(directory, dtype=thetas_dtype, usecols=thetas_usecols)
    dthetas = pd.read_csv(directory, dtype=dthetas_dtype, usecols=dthetas_usecols)
    time = pd.read_csv(directory, dtype=time_dtype, usecols=time_usecols)
    cartesian = pd.read_csv(directory, dtype=cartesian_dtype, usecols=cartesian_usecols)
    return time.values, thetas.values, dthetas.values, cartesian.values

def Log_Data(time_algo, time_to_log, thetas_to_log, dthetas_to_log, thetas_set_points_log, dthetas_set_points_log, ISE, cartesian_loggings, directory):

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
                            ('R_FB_POS_X_'): cartesian_loggings[:, 0],
                            ('R_FB_POS_Y_'): cartesian_loggings[:, 1],
                            ('R_FB_POS_Z_'): cartesian_loggings[:, 2],
                            ('TIMESTAMP_'): time_to_log,
                            ('TIME_ALGO'): time_algo[:,0],
                            ('ISE'): ISE
                            })
    print("======================WRITING======================")
    dataset.to_csv(directory)

    return

# ros initialization
roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('log_data', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("manipulator")
counter = 0
# logging
time_now = datetime.datetime.now()
folder_str = str(time_now.hour) + '_' + str(time_now.minute) + '_' + str(time_now.second)
os.mkdir("/home/abd-alrahman/catkin_ws/src/attrobot/plotting/" + folder_str + "/")

while not rospy.is_shutdown():
    # ball_traj_msg = rospy.wait_for_message("/ball_predicted_trajectory", ball_trajectory)
    set_points = rospy.wait_for_message("/robot_trajectory_planning", JointTrajectory)
    # x_ball = np.array([ball_traj_msg.x])
    # y_ball = np.array([ball_traj_msg.y])
    # z_ball = np.array([ball_traj_msg.z])
    # ball_traj = np.concatenate([x_ball.T, y_ball.T, z_ball.T], axis=1)
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
        time_algo.append((sec + nsec * (10 ** -9)))

    thetas_set_points_raw = np.row_stack(pos) * (180 / pi)
    dthetas_set_points_raw = np.row_stack(vel) * (180 / pi)
    ddthetas_set_points_raw = np.row_stack(acc) * (180 / pi)
    time_set_points_raw = np.array(time_algo)
    sz_setpoints_raw = thetas_set_points_raw.shape[0]
    final_time = time_set_points_raw[-1]
    opt_duration = final_time + 3
    sampling_time = time_set_points_raw[1]-time_set_points_raw[0]
    array_size = int(opt_duration / sampling_time)

    print("=============Waiting for egm motion to stop======================")
    read_logginigs = rospy.wait_for_message("/read_loggings", Bool)
    # read loggings from the file
    _, thetas, dthetas, cartesian = Read_Loggings()
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
    cartesian_loggings_raw = cartesian[start_index:end_index]
    sz_loggings_raw = thetas_loggings_raw.shape[0]

    thetas_set_points = np.zeros((array_size, 6))
    dthetas_set_points = np.zeros((array_size, 6))
    thetas_loggings = np.zeros((array_size, 6))
    dthetas_loggings = np.zeros((array_size, 6))
    cartesian_loggings = np.zeros((array_size, 3))

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

        cartesian_loggings[0:sz_loggings_raw, :] = cartesian_loggings_raw
        cartesian_loggings[sz_loggings_raw:, :] = cartesian_loggings_raw[-1, :] * np.ones(
            (array_size - sz_loggings_raw, 3))
    else:
        thetas_loggings = thetas_loggings_raw[:array_size, :]
        dthetas_loggings = dthetas_loggings_raw[:array_size, :]
        cartesian_loggings = cartesian_loggings[:array_size, :]


    counter_str = "itr_" + str(counter)
    ISE = np.sum(np.trapz(((thetas_set_points - thetas_loggings) ** 2), dx=sampling_time, axis=0)) / 6
    sz = len(time_algo)
    time_algo_to_log = np.zeros((array_size, 1))
    time_algo_to_log[:sz,0] = time_algo
    time_algo_to_log[sz:, 0] = main_time[sz:]
    Log_Data(time_algo_to_log, main_time, thetas_loggings, dthetas_loggings, thetas_set_points,
             dthetas_set_points, ISE, cartesian_loggings,
             "/home/abd-alrahman/catkin_ws/src/attrobot/plotting/" + folder_str + "/" + counter_str + ".csv")
    print ("ISE = ", ISE)
    counter = counter + 1
