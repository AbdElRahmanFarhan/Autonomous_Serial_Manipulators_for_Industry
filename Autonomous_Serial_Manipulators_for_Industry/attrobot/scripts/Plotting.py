#! /usr/bin/env python

import matplotlib.pyplot as plt
import  numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initializer
import sys
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, PositionIKRequest
from std_msgs.msg import Header
from math import pi

from moveit_msgs.srv import GetPositionFK, GetPositionIK

def inter_and_smooth(horizontal_axis, vertical_axis, res, inter_kind, final_point):
    inter_y_t = interp1d(horizontal_axis, vertical_axis, kind=inter_kind)
    t_new = np.linspace(0, final_point, num=res, endpoint=True)
    return inter_y_t,t_new

def Read_Loggings(directory):

    # theta columns
    thetas_dtype = {"R_FB_POS_RJ1_": float, "R_FB_POS_RJ2_": float, "R_FB_POS_RJ3_": float, "R_FB_POS_RJ4_": float,
                    "R_FB_POS_RJ5_": float, "R_FB_POS_RJ6_": float}
    thetas_usecols = ['R_FB_POS_RJ1_', 'R_FB_POS_RJ2_', 'R_FB_POS_RJ3_', 'R_FB_POS_RJ4_', 'R_FB_POS_RJ5_', 'R_FB_POS_RJ6_']

    # dtheta columns
    dthetas_dtype = {"R_FB_VEL_RJ1_": float, "R_FB_VEL_RJ2_": float, "R_FB_VEL_RJ3_": float, "R_FB_VEL_RJ4_": float,
                     "R_FB_VEL_RJ5_": float, "R_FB_VEL_RJ6_": float}
    dthetas_usecols = ['R_FB_VEL_RJ1_', 'R_FB_VEL_RJ2_', 'R_FB_VEL_RJ3_', 'R_FB_VEL_RJ4_', 'R_FB_VEL_RJ5_', 'R_FB_VEL_RJ6_']

    thetas_sp_dtype =  {('R_SP_POS_RJ1_'): float, ('R_SP_POS_RJ2_'): float, ('R_SP_POS_RJ3_'): float, ('R_SP_POS_RJ4_'): float, ('R_SP_POS_RJ5_'): float, ('R_SP_POS_RJ6_'): float}

    dthetas_sp_dtype = {('R_SP_VEL_RJ1_'): float, ('R_SP_VEL_RJ2_'): float, ('R_SP_VEL_RJ3_'): float, ('R_SP_VEL_RJ4_'): float, ('R_SP_VEL_RJ5_'): float, ('R_SP_VEL_RJ6_'): float}

    # cartesian_dtype = {"R_FB_POS_X_": float, "R_FB_POS_Y_": float, "R_FB_POS_Z_": float}


    thetas_sp_usecols = [('R_SP_POS_RJ1_'), ('R_SP_POS_RJ2_'), ('R_SP_POS_RJ3_'), ('R_SP_POS_RJ4_'),('R_SP_POS_RJ5_'), ('R_SP_POS_RJ6_')]

    dthetas_sp_usecols = [('R_SP_VEL_RJ1_'), ('R_SP_VEL_RJ2_'), ('R_SP_VEL_RJ3_'), ('R_SP_VEL_RJ4_'),('R_SP_VEL_RJ5_'), ('R_SP_VEL_RJ6_')]

    # cartesian_usecols = ['R_FB_POS_X_', 'R_FB_POS_Y_', 'R_FB_POS_Z_']

    # cam_dtype = {"CAM_FB_POS_X_": float, "CAM_FB_POS_Y_": float, 'CAM_FB_POS_Z_': float}
    # cam_usecols = ["CAM_FB_POS_X_", "CAM_FB_POS_Y_", 'CAM_FB_POS_Z_']

    # time columns
    time_dtype = {"TIMESTAMP_": float}
    time_usecols = ['TIMESTAMP_']
    time_algo_dtype = {"TIME_ALGO": float}
    time_algo_usecols = ['TIME_ALGO']

    thetas = pd.read_csv(directory, dtype=thetas_dtype, usecols=thetas_usecols)
    dthetas = pd.read_csv(directory, dtype=dthetas_dtype, usecols=dthetas_usecols)
    thetas_sp = pd.read_csv(directory, dtype=thetas_sp_dtype, usecols=thetas_sp_usecols)
    dthetas_sp = pd.read_csv(directory, dtype=dthetas_sp_dtype, usecols=dthetas_sp_usecols)
    # cartesian = pd.read_csv(directory, dtype=cartesian_dtype, usecols=cartesian_usecols)
    # cam = pd.read_csv(directory, dtype=cam_dtype, usecols=cam_usecols)
    time = pd.read_csv(directory, dtype=time_dtype, usecols=time_usecols)
    time_algo = pd.read_csv(directory, dtype=time_algo_dtype, usecols=time_algo_usecols)


    return time_algo.values, time.values, thetas.values, dthetas.values, thetas_sp.values, dthetas_sp.values


read_flag = 0
x_coord = []
y_coord = []
z_coord = []
t_points = []
while read_flag == 1:

    f = open("/home/abd-alrahman/loggings/ball_trajectory_ref_t.txt", "r")
    if f.mode == 'r':
        f3 = f.readlines()
        for x in f3:
            t_points.append(float(x))

        # print(t_points)
        f = open("/home/abd-alrahman/loggings/ball_trajectory_ref_x.txt", "r")
        if f.mode == 'r':
            f1 = f.readlines()
            for x in f1:
                x_coord.append(float(x))
            # print(x_coord)

        f = open("/home/abd-alrahman/loggings/ball_trajectory_ref_y.txt", "r")
        if f.mode == 'r':
            f2 = f.readlines()
            for x in f2:
                y_coord.append(float(x))

            # print(y_coord)
        f = open("/home/abd-alrahman/loggings/ball_trajectory_ref_z.txt", "r")
        if f.mode == 'r':
            f3 = f.readlines()
            for x in f3:
                z_coord.append(float(x))

    read_flag = 0

t = np.array(t_points)
x_cam = np.array(x_coord)
y_cam = np.array(y_coord)
z_cam = np.array(z_coord)

roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('plotting', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("manipulator")

directory = "/home/abd-alrahman/catkin_ws/src/attrobot/plotting/1_55_23_pos/itr_0.csv"
time_algo, time, thetas, dthetas, thetas_sp, dthetas_sp = Read_Loggings(directory)

rospy.wait_for_service('/compute_fk')
fk_solution = rospy.ServiceProxy('/compute_fk', GetPositionFK)
ref_frame = Header()
ref_frame.frame_id = "base_link"
ref_frame.stamp = rospy.Time.now()
# fk_link_names
fk_link_names = ["racket"]
joint_state = JointState()
joint_state.header = Header()
joint_state.header.frame_id = "racket"
joint_state.header.stamp = rospy.Time.now()
joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
x_sp_list = []
y_sp_list = []
z_sp_list = []
x_list = []
y_list = []
z_list = []

for i in range(len(thetas_sp)):
    joint_state.position = [thetas_sp[i,0] * (pi / 180), thetas_sp[i,1] * (pi / 180), thetas_sp[i,2] * (pi / 180), thetas_sp[i,3] * (pi / 180), thetas_sp[i,4] * (pi / 180), thetas_sp[i,5] * (pi / 180)]

    robot_joint_state = RobotState()
    robot_joint_state.joint_state = joint_state
    response = fk_solution(ref_frame, fk_link_names, robot_joint_state)
    robot_cartesian_state = response.pose_stamped[0]
    x_sp_list.append(robot_cartesian_state.pose.position.x)
    y_sp_list.append(robot_cartesian_state.pose.position.y)
    z_sp_list.append(robot_cartesian_state.pose.position.z)

for i in range(len(thetas)):
    joint_state.position = [thetas[i,0] * (pi / 180), thetas[i,1] * (pi / 180), thetas[i,2] * (pi / 180), thetas[i,3] * (pi / 180), thetas[i,4] * (pi / 180), thetas[i,5] * (pi / 180)]

    robot_joint_state = RobotState()
    robot_joint_state.joint_state = joint_state
    response = fk_solution(ref_frame, fk_link_names, robot_joint_state)
    robot_cartesian_state = response.pose_stamped[0]
    x_list.append(robot_cartesian_state.pose.position.x)
    y_list.append(robot_cartesian_state.pose.position.y)
    z_list.append(robot_cartesian_state.pose.position.z)

time_algo_sz = time_algo.size - 350
joint = 0
# fig = plt.figure(0)
# plt.plot(time[:-300], thetas[:-300,joint])
# plt.plot(time_algo, thetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("position(deg)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 1 position performance", fontsize=14)
#
# fig = plt.figure(1)
# plt.plot(time[:-300], dthetas[:-300,joint])
# plt.plot(time[:-300], dthetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("velocity(deg/s)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 1 velocity performance", fontsize=16)

joint = 1,
fig = plt.figure(2)
plt.plot(time[:time_algo_sz], thetas[:time_algo_sz,joint])
plt.plot(time_algo[:time_algo_sz], thetas_sp[:time_algo_sz,joint])
plt.legend(['actual', 'set_points'], loc=4)
plt.xlabel("time(s)", fontsize=15, position=(1,0))
plt.ylabel("position(deg)", fontsize=15, position=(0,1), rotation=0)
plt.title("joint 2 position performance", fontsize=14)

fig = plt.figure(3)
plt.plot(time[:time_algo_sz], dthetas[:time_algo_sz,joint])
plt.plot(time_algo[:time_algo_sz], dthetas_sp[:time_algo_sz,joint])
plt.legend(['actual', 'set_points'], loc=4)
plt.xlabel("time(s)", fontsize=15, position=(1,0))
plt.ylabel("velocity(deg/s)", fontsize=15, position=(0,1), rotation=0)
plt.title("joint 2 velocity performance", fontsize=16)

# joint = 2
# fig = plt.figure(4)
# plt.plot(time[:-300], thetas[:-300,joint])
# plt.plot(time[:-300], thetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("position(deg)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 3 position performance", fontsize=14)
#
# fig = plt.figure(5)
# plt.plot(time[:-300], dthetas[:-300,joint])
# plt.plot(time[:-300], dthetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("velocity(deg/s)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 3 velocity performance", fontsize=16)
#
#
# joint = 3
# fig = plt.figure(6)
# plt.plot(time[:-300], thetas[:-300,joint])
# plt.plot(time[:-300], thetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("position(deg)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 4 position performance", fontsize=14)
#
# fig = plt.figure(7)
# plt.plot(time[:-300], dthetas[:-300,joint])
# plt.plot(time[:-300], dthetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("velocity(deg/s)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 4 velocity performance", fontsize=16)
#
# joint = 4
# fig = plt.figure(8)
# plt.plot(time[:-300], thetas[:-300,joint])
# plt.plot(time[:-300], thetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("position(deg)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 5 position performance", fontsize=14)
#
# fig = plt.figure(9)
# plt.plot(time[:-300], dthetas[:-300,joint])
# plt.plot(time[:-300], dthetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("velocity(deg/s)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 5 velocity performance", fontsize=16)
#
# joint = 5
# fig = plt.figure(10)
# plt.plot(time[:-300], thetas[:-300,joint])
# plt.plot(time[:-300], thetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("position(deg)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 6 position performance", fontsize=14)
#
# fig = plt.figure(11)
# plt.plot(time[:-300], dthetas[:-300,joint])
# plt.plot(time[:-300], dthetas_sp[:-300,joint])
# plt.legend(['actual', 'set_points'], loc=4)
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("velocity(deg/s)", fontsize=15, position=(0,1), rotation=0)
# plt.title("joint 6 velocity performance", fontsize=16)

fig = plt.figure(12)
plt.plot(time[:time_algo_sz], x_list[:time_algo_sz])
plt.plot(time_algo[:time_algo_sz], x_sp_list[:time_algo_sz])
plt.legend(['actual', 'set_points'], loc=4)
plt.xlabel("time(s)", fontsize=15, position=(1,0))
plt.ylabel("x(m)", fontsize=15, position=(0,1), rotation=0)
plt.title("position controller performance in cartesian space", fontsize=16)

fig = plt.figure(13)
plt.plot(time[:time_algo_sz], y_list[:time_algo_sz])
plt.plot(time_algo[:time_algo_sz], y_sp_list[:time_algo_sz])
plt.legend(['actual', 'set_points'], loc=4)
plt.xlabel("time(s)", fontsize=15, position=(1,0))
plt.ylabel("y(m)", fontsize=15, position=(0,1), rotation=0)
plt.title("position controller performance in cartesian space", fontsize=16)

fig = plt.figure(14)
plt.plot(time[:time_algo_sz], z_list[:time_algo_sz])
plt.plot(time_algo[:time_algo_sz], z_sp_list[:time_algo_sz])
plt.legend(['actual', 'set_points'], loc=4)
plt.xlabel("time(s)", fontsize=15, position=(1,0))
plt.ylabel("z(m)", fontsize=15, position=(0,1), rotation=0)
plt.title("position controller performance in cartesian space", fontsize=16)

fig = plt.figure(15)
ax = Axes3D(fig)
ax.plot3D(x_list[:time_algo_sz], y_list[:time_algo_sz], z_list[:time_algo_sz],'-')
ax.plot3D(x_sp_list[:time_algo_sz], y_sp_list[:time_algo_sz], z_sp_list[:time_algo_sz], '-')
plt.show()



# x_cam_inter,t_new = inter_and_smooth(t-t[0],x_cam,cartesian.shape[0],'linear',t[-1]-t[0])
# y_cam_inter,_ = inter_and_smooth(t-t[0],y_cam,cartesian.shape[0],'linear',t[-1]-t[0])
# z_cam_inter,_ = inter_and_smooth(t-t[0],z_cam,cartesian.shape[0],'linear',t[-1]-t[0])
# fig = plt.figure(2)
# ax = Axes3D(fig)
# ax.plot3D(cartesian[:,0]/1000.0+0.195 , cartesian[:,1]/1000.0+0.009, cartesian[:,2]/1000.0+0.067,'-')
# ax.plot3D(x_cam[1:], y_cam[1:], z_cam[1:], 'o')

# fourier transform
# fig = plt.figure(i)
# plt.title("Lp = " + str(lp))
# T = 0.004
# N = thetas_list[i].shape[0]
# yf = np.fft.fft(dthetas_list[i][:, 0])
# xf = np.linspace(0.0, 1.0 / (2.0 * T), N / 2)  # start stop num of samples
# # yf_max = np.max(2.0/N *np.abs(yf[0:N//2]))
# # dominant_freq = int(np.where(2.0/N * np.abs(yf[1:N//2]) == yf_max)[0])
# # print (dominant_freq)
# plt.plot(xf[(i+1):N / 2], (2.0 / N) * np.abs(yf[(i+1): N // 2]))
# fig = plt.figure(2)
# sample_time = time[1]-time[0]
# plt.plot(time, np.diff(dthetas[:,joint])/sample_time)
# plt.plot(time, np.diff(dthetas_sp[:,joint])/sample_time)
# plt.legend(['actual', 'set_points'])
# plt.xlabel("time(s)", fontsize=15, position=(1,0))
# plt.ylabel("acceleration(deg/s2)", fontsize=15, position=(0,1), rotation=0)
# plt.title("K = 0.4, LP = 0, MAX_SPEED_DEV = 315.28", fontsize=16)
