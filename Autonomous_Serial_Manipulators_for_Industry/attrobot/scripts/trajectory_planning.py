#! /usr/bin/env python

import numpy as np
import time
from attrobot.STOPP import STOPP
import matplotlib.pyplot as plt
from scipy import interpolate
from math import ceil,floor,pi,e
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool


# j_max = 1000
# a_max = 100
# v_max = 4.4
j_max = 600
a_max = 20
v_max = 4.4

# j_max = 100
# a_max = 10
# v_max = 4
time_step = 0.008
size = 0
start_time = time.time()
interpolate = True
use_path = True
use_custom_path = True
log_data = False
joint_number = 0
plot_joint_number = 0

rospy.init_node("trajectory_planning")
# traj_pub = rospy.Publisher("/robot_trajectory_planning_fixed", JointTrajectory, queue_size=5)

# start_egm_pub = rospy.Publisher("/EGM_started", Bool, queue_size=1)
traj_pub = rospy.Publisher("/robot_trajectory_planning", JointTrajectory, queue_size=1)

while not rospy.is_shutdown():
    path_msg = rospy.wait_for_message("/robot_path_planning", JointTrajectory)
    rospy.loginfo("Path Recieved Time")

    time_now = time.time()
    sz = len(path_msg.points)
    path_global = np.zeros((6, sz))
    for p_num in range(sz):
        temp_point = path_msg.points[p_num]
        for j_num in range(6):
            path_global[j_num][p_num] = temp_point.positions[j_num]

    joint_number = np.argmax(abs(path_global[0:6,-1]-path_global[0:6,0]))
    all_joints_data = []
    all_joints_time = []

    print ("path = ", path_global)
    a = STOPP(0, 0, j_max, a_max, v_max, time_step, 0,
              0, 0,
              interpolation=interpolate)
    if (path_global[0].size%2) == 0:
        n = (path_global[0].size/2) - 1
        middle_point = (path_global[:,n]+path_global[:,n+1])/2.0
        path_global = np.column_stack([path_global[:,0:(n+1)],middle_point,path_global[:,(n+1):]])


    all_data, all_time = a.Make_path_profile(path_global[joint_number])
    pos_lift_up = 2 * ((all_data[0, -1] - all_data[0, 0]) + all_data[0, 0])
    all_data = np.column_stack([all_data, np.array([-np.flip(all_data[0, :-1]) + pos_lift_up, np.flip(all_data[1, :-1]),
                                                    -np.flip(all_data[2, :-1])])])
    reflected_time = np.cumsum(np.flip(np.diff(all_time)))
    all_time = np.concatenate([all_time, reflected_time + all_time[-1]], axis=0)
    for j_num in range(6):
        ratio = (path_global[j_num, -1]-path_global[j_num, 0]) / (all_data[0, -1] - all_data[0, 0])
        all_joints_data.append((((all_data-(all_data[:,0].reshape(3,1))) * ratio) + np.array([[path_global[j_num, 0]], [0], [0]])))
    # publish
    joint_traj = JointTrajectory()
    for p_num in range(all_data[0].size):
        point_traj = JointTrajectoryPoint()
        for j_num in range(6):
            point_traj.positions.append(all_joints_data[j_num][0, p_num])
            point_traj.velocities.append(all_joints_data[j_num][1, p_num])
            point_traj.accelerations.append(all_joints_data[j_num][2, p_num])
            secs = int(all_time[p_num])
            nsecs = int((all_time[p_num]-int(all_time[p_num]))*(10**9))
            point_traj.time_from_start.secs = secs
            point_traj.time_from_start.nsecs = nsecs
            # print("secs = {}, n_secs = {}".format(secs,nsecs))
        joint_traj.points.append(point_traj)
    rospy.loginfo("Publish trajectory to egm Time")
    traj_pub.publish(joint_traj)

    fig = plt.figure(1)
    for i in range(6):
        plt.plot(all_time, all_joints_data[i][1] * 180/pi)
    plt.legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
    plt.title("Velocity")
    fig = plt.figure(2)
    for i in range(6):
        plt.plot(all_time, all_joints_data[i][2] * 180 / pi)
    plt.legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
    plt.title("Acceleration")
    fig = plt.figure(3)
    for i in range(6):
        plt.plot(all_time, all_joints_data[i][0] * 180 / pi)
    plt.legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
    plt.title("Position")
    # plt.show()
