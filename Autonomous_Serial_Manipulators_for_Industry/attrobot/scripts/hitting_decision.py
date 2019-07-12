#! /usr/bin/env python

from attrobot.msg import ball_trajectory
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, PositionIKRequest
from std_msgs.msg import Header
from std_msgs.msg import Bool
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initializer
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import numpy as np
import rospy
import math
import sys
import time
# import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import pi


def hitting_callback(ball_traj_msg):

    global counter, fk_solution, ik_solution, start_egm_pub, target_pose_pub, robot, scene, target_pose_msg,     egm_start_msg

    rospy.loginfo("prediction msg recieved Time = {}".format(counter))
    counter +=1
    # get current robot state
    # header
    ref_frame = Header()
    ref_frame.frame_id = "base_link"
    ref_frame.stamp = rospy.Time.now()
    # fk_link_names
    fk_link_names = ["racket"]
    joint_state = JointState()
    # joint_state_msg = rospy.wait_for_message("/robot_joint_state", JointState)
    joint_state.header = Header()
    joint_state.header.frame_id = "racket"
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    joint_state.position = [28 * (pi/180), 34.72 * (pi/180), 1.52 * (pi/180), 34.86 * (pi/180), -42.73 * (pi/180),  -27.7 * (pi/180)]
    robot_joint_state = RobotState()
    robot_joint_state.joint_state = joint_state
    response = fk_solution(ref_frame, fk_link_names, robot_joint_state)
    robot_cartesian_state = response.pose_stamped[0]

    # print ("wait for ball trajectory msg")
    # ball_traj_msg = rospy.wait_for_message("/ball_predicted_trajectory", ball_trajectory)

    # continue

    start_egm_pub.publish(egm_start_msg)
    # print ball_traj_msg
    time_now = time.time()
    # #print ("ball trajectory msg recieved ")
    x_ball = np.array([ball_traj_msg.x])
    y_ball = np.array([ball_traj_msg.y])
    z_ball = np.array([ball_traj_msg.z])
    t_ball = np.array([ball_traj_msg.t])

    # delta_t_global = np.array([ball_traj_msg.t])
    ball_traj = np.concatenate([x_ball.T,y_ball.T,z_ball.T], axis = 1)
    x_robot = robot_cartesian_state.pose.position.x
    y_robot = robot_cartesian_state.pose.position.y
    z_robot = robot_cartesian_state.pose.position.z
    current_robot_state = np.array([x_robot, y_robot, z_robot])
    print ("current robot state = ", current_robot_state)

    target_pose, t, start, end = hitting_point(ball_traj, current_robot_state, t_ball)
    if t == 0:
        # print("Hereeeeeeeeeeeeeeee")
        # print(target_pose)
        # # figure = plt.figure(11)
        # plt.ylim((-8,1))
        # plt.xlim((3, 3))
        # ax = Axes3D(figure)
        # ax.plot3D(ball_traj_msg.x, ball_traj_msg.y, ball_traj_msg.z, 'o', c='y')
        # ax.scatter3D(ball_traj_msg.x[points_list[0]], ball_traj_msg.y[points_list[0]], ball_traj_msg.z[points_list[0]], c='r')
        # ax.scatter3D(current_robot_state[0], current_robot_state[1], current_robot_state[2],
        #              c='b')

        u = np.linspace(0, np.pi, 15)
        v = np.linspace(0, 2 * np.pi, 15)
        R = R_global
        x = R * np.outer(np.sin(u), np.sin(v))
        y = R * np.outer(np.sin(u), np.cos(v))
        z = R * np.outer(np.cos(u), np.ones_like(v))
        # ax.plot_wireframe(x, y, z)

        u = np.linspace(0, np.pi, 10)
        v = np.linspace(0, 2 * np.pi, 10)
        r = r_global
        x = r * np.outer(np.sin(u), np.sin(v))
        y = r * np.outer(np.sin(u), np.cos(v))
        z = r * np.outer(np.cos(u), np.ones_like(v))
        # ax.plot_wireframe(x, y, z)

        # ax.legend(["ball trajectory", "current robot state", "outer radius", "inner radius"])
#         # plt.show()
    else:
        # publish hitting point
        orientation_ranges = np.array([[0, 0, 0, 1]])

        Valid_Pose_found = False
        p_idx = 0 # position(point) index
        o_idx = 0 # orientation index
        target_pose_msg.position.x = target_pose[0, 0]
        target_pose_msg.position.y = target_pose[0, 1]
        target_pose_msg.position.z = target_pose[0, 2]
        target_pose_msg.orientation.x = orientation_ranges[0, 0]
        target_pose_msg.orientation.y = orientation_ranges[0, 1]
        target_pose_msg.orientation.z = orientation_ranges[0, 2]
        target_pose_msg.orientation.w = orientation_ranges[0, 3]
        print("!!!!!!!!!!!!1number of positions = ", end-start+1)
        rospy.loginfo("start search")

        for j in range(target_pose.shape[0]):
            for i in range(orientation_ranges.shape[0]):

                position_1 = PoseStamped()
                position_1.header = ref_frame
                position_1.pose.position.x = target_pose[j, 0]
                position_1.pose.position.y = target_pose[j, 1]
                position_1.pose.position.z = target_pose[j, 2]
                position_1.pose.orientation.x = orientation_ranges[i, 0]
                position_1.pose.orientation.y = orientation_ranges[i, 1]
                position_1.pose.orientation.z = orientation_ranges[i, 2]
                position_1.pose.orientation.w = orientation_ranges[i, 3]

                ik_request = PositionIKRequest()
                ik_request.group_name = "manipulator"
                ik_request.avoid_collisions = True
                ik_request.pose_stamped = position_1
                rospy.loginfo("test {} start".format(j*orientation_ranges.shape[0] + i))
                response = ik_solution(ik_request)
                rospy.loginfo("test {} end".format(j * orientation_ranges.shape[0] + i))
                if response.error_code.val == 1:
                    Valid_Pose_found = True
                    target_pose_msg.position.x = target_pose[j, 0]
                    target_pose_msg.position.y = target_pose[j, 1]
                    target_pose_msg.position.z = target_pose[j, 2]
                    target_pose_msg.orientation.x = orientation_ranges[i, 0]
                    target_pose_msg.orientation.y = orientation_ranges[i, 1]
                    target_pose_msg.orientation.z = orientation_ranges[i, 2]
                    target_pose_msg.orientation.w = orientation_ranges[i, 3]
                    p_idx = j
                    o_idx = i
                    # print("!!!!!!!!!!!!!orientation index = ", i)
                    # print("!!!!!!!!!!!!!position index = ", j)
                    break
            if Valid_Pose_found:
                break
            elif j == target_pose.shape[0]-1:
                rospy.loginfo("No valid Pose Found !!!!!!")
                Valid_Pose_found = False

        rospy.loginfo("end search")
        # #print ("hitting point = ", target_pose_msg)
        # #print ("time to hit = ", t)
        # points_list.append(target_pose)

        if Valid_Pose_found:
            rospy.loginfo("Publishing hitting point!!!!!!!!")
            robot_joint_state_1 = RobotState()
            robot_joint_state_1.joint_state  = response.solution.joint_state
            # print("joints = ", np.array(response.solution.joint_state.position)*180/pi)
            response = fk_solution(ref_frame, fk_link_names, robot_joint_state_1)
            target_pose_msg = response.pose_stamped[0].pose
            # print("goal_x = {}, goal_y = {}, goal_z = {}".format(target_pose[p_idx, 0], target_pose[p_idx, 1],
            #                                                      target_pose[p_idx, 2]))
            # print("pose_x = {}, pose_y = {}, pose_z = {}".format(target_pose_msg.position.x, target_pose_msg.position.y,
            #                                                      target_pose_msg.position.z))
            # print("orient_x = {}, orient_y = {}, orient_z = {}, orient_w = {}".format(target_pose_msg.orientation.x, target_pose_msg.orientation.y,
            #                                                      target_pose_msg.orientation.z,target_pose_msg.orientation.w ))
            response = fk_solution(ref_frame, ["link_6"], robot_joint_state_1)
            # print("link_pose_x = {}, link_pose_y = {}, link_pose_z = {}".format(response.pose_stamped[0].pose.position.x, response.pose_stamped[0].pose.position.y,
            #                                                      response.pose_stamped[0].pose.position.z))
            rospy.loginfo("Start publishing goal point to planner Time")
            target_pose_pub.publish(target_pose_msg)
            print ("target pose = ", target_pose_msg)

        # figure = plt.figure(12)
        # ax = Axes3D(figure)
        # ax.plot3D(ball_traj_msg.x[start-5:(end+1)], ball_traj_msg.y[start-5:(end+1)], ball_traj_msg.z[start-5:(end+1)], 'x', c='r')
        # ax.scatter3D(target_pose[j, 0], target_pose[j, 1], target_pose[j, 2],c='g')
        # ax.scatter3D(ball_traj_msg.x[points_list[0]], ball_traj_msg.y[points_list[0]], ball_traj_msg.z[points_list[0]], c='r')
        # ax.scatter3D(current_robot_state[0], current_robot_state[1], current_robot_state[2],
        #              c='b')

        u = np.linspace(0, np.pi, 15)
        v = np.linspace(0, 2 * np.pi, 15)
        R = R_global
        x = R * np.outer(np.sin(u), np.sin(v))
        y = R * np.outer(np.sin(u), np.cos(v))
        z = R * np.outer(np.cos(u), np.ones_like(v))
        # ax.plot_wireframe(x, y, z)

        u = np.linspace(0, np.pi, 10)
        v = np.linspace(0, 2 * np.pi, 10)
        r = r_global
        x = r * np.outer(np.sin(u), np.sin(v))
        y = r * np.outer(np.sin(u), np.cos(v))
        z = r * np.outer(np.cos(u), np.ones_like(v))
        # ax.plot_wireframe(x, y, z)

        # ax.legend(["ball trajectory", "target pose", "current robot state", "outer radius", "inner radius"])
#         # plt.show()

        #print ("hitting time = ", time.time() - time_now)
        # # visualize ball trajectory in Rviz
        # prediction_dt = 0.02
        # simulation_dt = 0.02
        # loop_step = int(simulation_dt / prediction_dt)
        # ball_pose = PoseStamped()
        # ball_pose.header.frame_id = "base_link"
        # point = 0
        # #print ("size = ",  ball_traj.shape[0])
        # while point < ball_traj.shape[0]:
        #     if ball_traj[point, 0] == target_pose[0]:
        #          break
        #     ball_pose.pose.position.x = ball_traj[point, 0]
        #     ball_pose.pose.position.y = ball_traj[point, 1]
        #     ball_pose.pose.position.z = ball_traj[point, 2]
        #     ball_radius = 0.02
        #     scene.add_sphere(str(point), ball_pose, ball_radius)
        #     point = point + loop_step
        #     # rospy.sleep(simulation_dt)




def hitting_point(discretized_ball_traj, robot_state, t_ball):
    global points_list, R_global, r_global
    T_global_to_robot = np.eye(4, 4, dtype=float)
    T_global_to_robot[0:3, 3] = np.array([0, 0, 0])
    dt_min = 0.1
    plan_threshold = 0.2
    R = R_global
    r = r_global
    R_2 = math.pow(R, 2)
    r_2 = math.pow(r, 2)
    delta_t = t_ball[0]
    index_1 = int(math.ceil(dt_min / delta_t))
    index_1 = 0
    points_list.append(index_1)
    xyz_after_time_filter = discretized_ball_traj[index_1:, :]
    # print ("after filter time", xyz_after_time_filter)
    if xyz_after_time_filter[0].size <= 1:
        print ("no point due to time")
        return 'no point due to time', 0,0 ,0
    xyz_after_time_filter_inc = np.ones((xyz_after_time_filter.shape[0], 4))
    xyz_after_time_filter_inc[:, 0:3] = xyz_after_time_filter
    points_robot = xyz_after_time_filter_inc
    points_robot = np.dot(T_global_to_robot, xyz_after_time_filter_inc.T)
    # x^2 + y^2 + z^2 = R^2 --- radius_points = radius of sphere of each poiny
    radius_points = np.sum(np.power(points_robot.T[:, 0:3], 2), axis=1)
    indices_inside_ws = np.array(np.where(np.logical_and(radius_points >= r_2, radius_points <= R_2)))
    if indices_inside_ws[0].size <=1  :
        print ("no point for hitting")
        return 'no point for hitting', 0, 0, 0
    start = indices_inside_ws[0, 0]
    end = indices_inside_ws[0, -1]
    #print ("indices = ", indices_inside_ws)
    #print("xs = ", points_robot.T[start:end, 0])
    # print ("indices inside ws", indices_inside_ws)
    indices_above_table = np.where(points_robot.T[start:(end+1), 2] > 0.15)
    if indices_above_table[0].size < 1:
        print ("no point for hitting")
        return 'no point for hitting', 0, 0, 0
    indices_above_table = indices_above_table + start
    start = indices_above_table[0, 0]
    end = indices_above_table[0, -1]
    dist = np.sqrt(np.power(np.subtract(robot_state[0], points_robot.T[start:(end+1), 0]), 2) + np.power(
        np.subtract(robot_state[1], points_robot.T[start:(end+1), 1]), 2) + np.power(
        np.subtract(robot_state[2], points_robot.T[start:(end+1), 2]), 2))
    ind = np.argmin(dist)
    # points_robot = points_robot.T[start:(end+1), :].T
    # points_in_plane = np.array(np.where(np.logical_and(points_robot.T[:,1]>= (-1 * plan_threshold), points_robot.T[:,1]<=plan_threshold)))
    # print("points_in_plane = {}".format(points_in_plane))
    # if points_in_plane[0].size < 1:
    #     print ("no point for hitting")
    #     return 'no point for hitting', 0, 0, 0
    # start = points_in_plane[0, 0]
    # end = points_in_plane[0, -1]
    # points_robot = points_robot.T[start:(end+1),:].T
    sorted_indices = np.argsort(dist)
    points_robot = points_robot.T[sorted_indices+start,:].T
    # ind = np.argmax(points_robot.T[start:(end+1), 2])
    print("Chosen index = ",ind)
    # ind = 0
    optimum_point_index_in_points_robot = ind + start
    index_in_discretized_ball_traj = index_1 + optimum_point_index_in_points_robot
    time = delta_t * index_in_discretized_ball_traj
    points_list.append(optimum_point_index_in_points_robot)
    return points_robot.T[:, 0:3], time, start, end


rospy.init_node("hitting_decision", anonymous=True)
start_egm_pub = rospy.Publisher("/EGM_started", Bool, queue_size=1)
target_pose_pub = rospy.Publisher("/robot_target_pose", Pose, queue_size=1 )
rospy.Subscriber("/ball_predicted_trajectory", ball_trajectory, hitting_callback)
roscpp_initializer.roscpp_initialize(sys.argv)
robot = RobotCommander()
scene = PlanningSceneInterface()
# group = MoveGroupCommander("manipulator")
rospy.wait_for_service('/compute_fk')
fk_solution = rospy.ServiceProxy('/compute_fk', GetPositionFK)
rospy.wait_for_service('/compute_ik')
ik_solution = rospy.ServiceProxy('/compute_ik', GetPositionIK)
points_list = []
counter = 1
target_pose_msg = Pose()
egm_start_msg = Bool()
egm_start_msg.data = True
R_global = 1
r_global = 0.4
rospy.spin()
