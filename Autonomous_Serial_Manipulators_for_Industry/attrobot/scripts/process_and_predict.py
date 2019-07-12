#! /usr/bin/env python

import numpy as np
from attrobot.traj_pred import mains as tp
import rospy
from attrobot.msg import ball_trajectory
import os
import time


# Initialize Node
rospy.init_node('process_and_predict', anonymous=True)
ball_pub = rospy.Publisher('/ball_predicted_trajectory', ball_trajectory, queue_size=1)
# rospy.sleep(1)


# def Callback(traj):
#     rospy.loginfo("received")
#     ball_pub.publish(ball_msg)
#
#
# raw_input()
# rospy.loginfo("start processing")

# ball_msg = ball_trajectory()
# rospy.loginfo('waiting fo message')
# while True:
#     ball_msg = rospy.wait_for_message('/ball_predicted',ball_trajectory)
#     rospy.loginfo('trajectory message received')


# # Initialize projectile start conditions
w_0 = np.zeros(shape=[3], dtype=float)
s_0 = np.array([0.6, -4, 1])
v_0 = np.array([0.2, 7.5, 2.7])
# Predict the remaining part of the trajectory(~100ms)
xyz, p, vel = tp('free', s_0, v_0, w_0)
ball_msg = ball_trajectory()
ball_msg.x = xyz[:, 0]
ball_msg.y = xyz[:, 1]
ball_msg.z = xyz[:, 2]
ball_msg.t = np.array([1.0 / 20.0])
raw_input()
rospy.loginfo("publish prediction msg")
ball_pub.publish(ball_msg)
