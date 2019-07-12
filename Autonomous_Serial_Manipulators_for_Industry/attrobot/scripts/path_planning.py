#! /usr/bin/env python
import sys
import time
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initializer
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Bool
from math import pi

# initializations
roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('path_planning', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("manipulator")

start_egm_pub = rospy.Publisher("/EGM_started", Bool, queue_size=1)
motion_pub = rospy.Publisher("/robot_path_planning", JointTrajectory, queue_size=1)
skip_flag = 0

while not rospy.is_shutdown():

    if (skip_flag == 0):
        # joint_state_msg = rospy.wait_for_message("/robot_joint_state", JointState)
        initial_state = JointState()
        initial_state.header = Header()
        initial_state.header.frame_id = "base_link"
        initial_state.header.stamp = rospy.Time.now()
        initial_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        # initial_state.position = joint_state_msg.position

        initial_state.position = [28 * (pi/180), 34.72 * (pi/180), 1.52 * (pi/180), 34.86 * (pi/180), -42.73 * (pi/180),  -27.7 * (pi/180)]

        robot_state = RobotState()
        robot_state.joint_state = initial_state
        group.set_start_state(robot_state)
        group.set_pose_reference_frame("base_link")
        # group.set_goal_orientation_tolerance(0.5)



    print ("waiting for hitting point !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    target_pose = rospy.wait_for_message("/robot_target_pose", Pose)

    rospy.loginfo("Goal Point Recieved Time")
    # target_joint = rospy.wait_for_message("/robot_target_joint", JointState)
    # #print ("hitting point recieved")
    # group.set_trajectory_constraints()
    plan = group.plan(target_pose)


    # plan = group.plan(target_joint)

    if len(plan.joint_trajectory.points) > 1:
        motion_pub.publish(plan.joint_trajectory)
        #print ("path_planning time = ", time.time() - time_now)
        skip_flag = 0
    else :
        skip_flag = 1




