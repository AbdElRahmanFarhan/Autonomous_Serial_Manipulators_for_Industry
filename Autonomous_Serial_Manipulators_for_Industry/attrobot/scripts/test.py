#! /usr/bin/env python

import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionFK,GetPositionIKResponse
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initializer
import sys
import numpy as np

rospy.init_node("test", anonymous=True)
target_pose_pub = rospy.Publisher("/robot_target_pose", Pose, queue_size=1 )
roscpp_initializer.roscpp_initialize(sys.argv)
robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("manipulator")

rospy.wait_for_service('/compute_ik')
ik_solution = rospy.ServiceProxy('/compute_ik', GetPositionIK)
#
ref_frame = Header()
ref_frame.frame_id = "base_link"
ref_frame.stamp = rospy.Time.now()
#
# fk_link_names
fk_link_names = ["racket"]
joint_state = JointState()
joint_state.header = Header()
joint_state.header.frame_id = "racket"
joint_state.header.stamp = rospy.Time.now()
joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
joint_state.position = [0, 0, 0, 0, 0, 0]
robot_joint_state = RobotState()
robot_joint_state.joint_state = joint_state

position_1 = PoseStamped()
position_1.header = ref_frame
position_1.pose.position.x = 0.68
position_1.pose.position.y = 0.1
position_1.pose.position.z = 0.5
position_1.pose.orientation.x = 0
position_1.pose.orientation.y = 0
position_1.pose.orientation.z = 0
position_1.pose.orientation.w = 1

position_2 = PoseStamped()
position_2.header = ref_frame
position_2.pose.position.x = 0.7
position_2.pose.position.y = 0
position_2.pose.position.z = 0.6
position_2.pose.orientation.x = 0
position_2.pose.orientation.y = 0
position_2.pose.orientation.z = 0
position_2.pose.orientation.w = 1

way_points = [position_1.pose, position_2.pose]
rospy.loginfo("Start IK Request")
ik_request = PositionIKRequest()
ik_request.group_name = "manipulator"
ik_request.avoid_collisions = True
ik_request.pose_stamped = position_1
response = ik_solution(ik_request)
rospy.loginfo("End IK Request")
# group.set_pose_reference_frame("base_link")
# group.set_pose_targets(way_points, end_effector_link="racket")
# plan = group.plan()
# path = plan.joint_trajectory
# sz = len(path.points)
# plan_array = np.zeros((sz, 3))
#
# solution = response.solution.joint_state.position
# ik_array = solution[0:3]
# for p_num in range(sz):
#     temp_point = path.points[p_num]
#     for j_num in range(3):
#         plan_array[p_num, j_num] = temp_point.positions[j_num]
# hitting_index = np.sum(np.abs(plan_array - ik_array), axis=1).argmin()
# print plan_array[hitting_index, :]
# print ik_array
print response
# print plan
# c = np.zeros((64, 6))
# for i in range(64):
#     b = np.unpackbits(np.array([i], dtype=np.uint8))
#     c[i] = ((b[2:] * -2) + 1)*30
#
# print "{"
# for i in range(64):
#     if i != 63:
#         print "{",c[i,0],",",c[i,1],",",c[i,2],",",c[i,3],",",c[i,4],",",c[i,5],"},"
#     else:
#         print "{",c[i,0],",",c[i,1],",",c[i,2],",",c[i,3],",",c[i,4],",",c[i,5],"}}"
#
