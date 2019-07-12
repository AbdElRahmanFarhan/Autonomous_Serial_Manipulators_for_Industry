#! /usr/bin/env python

import numpy as np
from attrobot.traj_pred import mains as tp
import rospy
from attrobot.msg import ball_trajectory
import os
import time


# Initialize Node
rospy.init_node('print_time', anonymous=True)
# ball_pub = rospy.Publisher('/ball_predicted_trajectory', ball_trajectory, queue_size=1)
rospy.sleep(1)
while True:
    rospy.loginfo('time')
