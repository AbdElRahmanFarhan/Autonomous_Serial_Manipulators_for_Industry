#! /usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initializer
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi



roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('user_interface', anonymous=True)
pose_pub = rospy.Publisher("/robot_target_pose", Pose, queue_size=5)
joint_pub = rospy.Publisher("/robot_target_joint", JointState, queue_size=5)
pose_1_pub = rospy.Publisher("/robot_target_pose_1", Pose, queue_size=5)
pose_2_pub = rospy.Publisher("/robot_target_pose_2", Pose, queue_size=5)

# rospy.sleep(1)

target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.1
target_pose.position.z = 0.5
target_pose.orientation.x = 0
target_pose.orientation.y = 0
target_pose.orientation.z = 0
target_pose.orientation.w = 1
raw_input()
pose_1_pub.publish(target_pose)
# target_pose = Pose()
# target_pose.position.x = 0.4
# target_pose.position.y = 0.1
# target_pose.position.z = 0.6
# target_pose.orientation.x = 0
# target_pose.orientation.y = 0
# target_pose.orientation.z = 0
# target_pose.orientation.w = 1
# raw_input()
# pose_2_pub.publish(target_pose)
# joint_goal = JointState()
# joint_goal.header = Header()
# joint_goal.header.frame_id = "base_link"
# joint_goal.header.stamp = rospy.Time.now()
# joint_goal.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
# joint_goal.position = [-90*(pi/180), -40*(pi/180), 0*(pi/180), 0*(pi/180), 0*(pi/180), 0*(pi/180)]
# joint_pub.publish(joint_goal)



rospy.loginfo("processing start time")
