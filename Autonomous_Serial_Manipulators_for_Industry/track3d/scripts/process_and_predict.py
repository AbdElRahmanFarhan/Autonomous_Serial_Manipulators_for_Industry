#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
# from Generalized_Impact_Model import impactModel as gim
from track3d.traj_pred import mains as tp
from mpl_toolkits.mplot3d import Axes3D
from track3d.msg import  ball_trajectory
# import mathq
from Post_Processing_Block import myfit_coords_base, myfit_coords_cam
from Post_Processing_Block import post_process_coords
import rospy

if __name__ == '__main__':


	# Initialize Node
	rospy.init_node('process_and_predict', anonymous=True)
	prediction_pub = rospy.Publisher('/ball_predicted_trajectory', ball_trajectory, queue_size=10)
	loop_num=0

	while(not rospy.is_shutdown()):

		x_points = []
		y_points = []
		t_points = []
		t_pixels = []

		x_coord = []
		y_coord = []
		z_coord = []

		w_0 = np.zeros(shape=[3], dtype=float)

		print("Process_and_Predict is waiting for ball trajectory messege")
		msg = rospy.wait_for_message('/ball_trajectory_topic', ball_trajectory)
		rospy.loginfo("Ball trajectory messege recieved Time")

		origin_point = np.array([0, 0, 0])
		transform = np.array([[ 0.73608089,  0.01867336,  0.67663596, -0.63030813],
       [-0.67680551,  0.0364277 ,  0.73526003, -3.45084299],
       [-0.01091851, -0.99916181,  0.03945197,  0.72712321],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

		# translation_hc = np.array([-0.374,    -2.305065,  0.634908+0.123])
		# translation_hc = np.array([-0.36938121+0.15, -2.40334327-0.7, 0.63575143+0.125+0.78]) arkam doctor shady
		# translation_hc = np.array([-0.36938121 + 0.105, -2.40334327 - 0.71, 0.63575143 + 0.125 + 0.76])
		# ##Hard Codedd:
		# transform = np.array([[0 , 0 , 1, translation_hc[0]],
		# 						   [-1, 0 , 0, translation_hc[1]],
		# 						   [0 , -1, 0, translation_hc[2]],
		# 						   [0 , 0 , 0,         1       ]])

		base = True  # from Base or from Cam
		demo = False

		raw_t = np.array(msg.t)
		raw_x_cam = np.array(msg.x)
		raw_y_cam = np.array(msg.y)
		raw_z_cam = np.array(msg.z)

		zero_index = np.array(np.where(np.logical_or((raw_z_cam >= 100.0) , (raw_z_cam <= 0.1))))
		x_cam = np.delete(raw_x_cam, zero_index)
		y_cam = np.delete(raw_y_cam, zero_index)
		z_cam = np.delete(raw_z_cam, zero_index)
		t = np.delete(raw_t, zero_index)

		outlier_index = np.array(np.where(np.diff(z_cam) < 0)) + 1
		x_cam = np.delete(x_cam, outlier_index)
		y_cam = np.delete(y_cam, outlier_index)
		z_cam = np.delete(z_cam, outlier_index)
		t = np.delete(t, outlier_index)

		outlier_index = np.array(np.where(np.abs(np.diff(z_cam)) >= 0.2)) + 1
		x_cam = np.delete(x_cam, outlier_index)
		y_cam = np.delete(y_cam, outlier_index)
		z_cam = np.delete(z_cam, outlier_index)
		t = np.delete(t, outlier_index)

		outlier_index = np.array(np.where(np.abs(np.diff(x_cam)) >= 0.5)) + 1
		x_cam = np.delete(x_cam, outlier_index)
		y_cam = np.delete(y_cam, outlier_index)
		z_cam = np.delete(z_cam, outlier_index)
		t = np.delete(t, outlier_index)

		outlier_index = np.array(np.where(np.abs(np.diff(y_cam)) >= 0.5)) + 1
		x_cam = np.delete(x_cam, outlier_index)
		y_cam = np.delete(y_cam, outlier_index)
		z_cam = np.delete(z_cam, outlier_index)
		t = np.delete(t, outlier_index)

		# x_cam = np.delete(x_cam, 0)
		# y_cam = np.delete(y_cam, 0)
		# z_cam = np.delete(z_cam, 0)
		# t = np.delete(t, 0)

		# print("outlier_index = {0}".format(outlier_index))

		# zero_index = np.where(raw_z_cam == 100.0)
		# x_cam_tracked = np.delete(raw_x_cam, zero_index)
		# y_cam_tracked = np.delete(raw_y_cam, zero_index)
		# z_cam_tracked = np.delete(raw_z_cam, zero_index)
		# t_tracked = np.delete(raw_t, zero_index)

		n_fit = 15
		prediction_time_step = 1.0 / 10.0
		x = np.array([])
		y = np.array([])
		z = np.array([])
		x_base = np.array([])
		y_base = np.array([])
		z_base = np.array([])

		print("total_filtered_points = ", x_cam.shape[0])
		# s_2, v_2, x_pp, y_pp = post_process(x_pixels, y_pixels, t_pix)

		if base:

			# s_1_cam, v_1_cam, a_1_cam, x_cam_pp, y_cam_pp, z_cam_pp, prediction_time_step = \
			# 	post_process_coords(x_cam[:n_fit], y_cam[:n_fit], z_cam[:n_fit], t[:n_fit], base)
			s_1_cam, v_1_cam, a_1_cam, x_cam_pp, y_cam_pp, z_cam_pp, _ = \
				post_process_coords(x_cam, y_cam, z_cam, t, False)
			print("VELOCITY_OF_BAL = {}".format(v_1_cam))

			if not demo:
				# points_raw_from_base = np.dot(transform, np.row_stack([x_cam, y_cam, z_cam, np.ones((1, x_cam.shape[0]))]))
				# x_base_raw = points_raw_from_base[0, :]
				# y_base_raw = points_raw_from_base[1, :]
				# z_base_raw = points_raw_from_base[2, :]
				#
				# s_1_base, v_1_base, a_1_base, x_base_pp, y_base_pp, z_base_pp, _ = \
				# 	post_process_coords(x_base_raw, y_base_raw, z_base_raw, t, True)
				#
				# xyz_base, p_base, vel_base = tp('free', s_1_base, v_1_base, a_1_base, prediction_time_step, True)
				#
				# x_base = xyz_base[:, 0]
				# y_base = xyz_base[:, 1]
				# z_base = xyz_base[:, 2]

				xyz, p, vel = tp('free', s_1_cam, v_1_cam, a_1_cam, prediction_time_step, False)

				# Plot the data
				x = xyz[:, 0] - origin_point[0]
				y = xyz[:, 1] - origin_point[1]
				z = xyz[:, 2] - origin_point[2]
				points_from_base = np.dot(transform, np.row_stack([x, y, z, np.ones((1, x.shape[0]))]))
				x_base = points_from_base[0, :]
				y_base = points_from_base[1, :]
				z_base = points_from_base[2, :]

				tpred_new = np.linspace(0, (x_base.shape[0] - 1) * prediction_time_step, num=x_base.shape[0])
				# resolution_interp = 2 * (x_base.shape[0])
				resolution_interp = 255

			else:
				s_1_base = np.array([0.6, -4, 1])
				v_1_base = np.array([-1.2, 8, 2.5])
				xyz, p, vel = tp('free', s_1_base, v_1_base, a_1_cam, prediction_time_step, True)

				# # Plot the data
				x_base = xyz[:, 0]
				y_base = xyz[:, 1]
				z_base = xyz[:, 2]

				tpred_new = np.linspace(0, (x_base.shape[0] - 1) * prediction_time_step, num=x_base.shape[0])
				resolution_interp = 2 * (x_base.shape[0])

			final_time = tpred_new[-1] + 0.2
			rospy.loginfo("Prediction Fitting Started")
			trendpoly_y, trend_y, _ = myfit_coords_base(tpred_new, y_base, resolution_interp, final_time, 1)
			trendpoly_x, trend_x, _ = myfit_coords_base(tpred_new, x_base, resolution_interp, final_time, 3)
			trendpoly_z, trend_z, t_new = myfit_coords_base(tpred_new, z_base, resolution_interp, final_time, 2)
			rospy.loginfo("Prediction Fitting Finished")

			prediction_time_step = t_new[1]
			# print("time_series = {}".format(t_new))
			x_base_hr = trendpoly_x(t_new)
			y_base_hr = trendpoly_y(t_new)
			z_base_hr = trendpoly_z(t_new)

			pred_msg = ball_trajectory()
			pred_msg.x = x_base_hr
			pred_msg.y = y_base_hr
			pred_msg.z = z_base_hr
			pred_msg.t = [prediction_time_step]

			now = rospy.get_rostime()
			print("time of publishing", now.secs + now.nsecs * (10 ** -9))
			prediction_pub.publish(pred_msg)

			# for i in range(1):
			# 	# rospy.loginfo("Start to send prediction to hitting Time")
			# 	now = rospy.get_rostime()
			# 	print("time of publishing", now.secs+now.nsecs*(10**-9))
			# 	prediction_pub.publish(pred_msg)
			# 	# rospy.sleep(3)


			points_from_base = np.dot(transform, np.row_stack([x_cam_pp - origin_point[0],
															   y_cam_pp - origin_point[1],
															   z_cam_pp - origin_point[2],
															   np.ones((1, x_cam_pp.shape[0]))]))
			x_cam_to_base_pp = points_from_base[0, :]
			y_cam_to_base_pp = points_from_base[1, :]
			z_cam_to_base_pp = points_from_base[2, :]

			# pred_msg = ball_trajectory()
			# pred_msg.x = x_cam_to_base_pp
			# pred_msg.y = y_cam_to_base_pp
			# pred_msg.z = z_cam_to_base_pp
			# pred_msg.t = [prediction_time_step]


			points_from_base = np.dot(transform, np.row_stack([x_cam - origin_point[0],
															   y_cam - origin_point[1],
															   z_cam - origin_point[2],
															   np.ones((1, x_cam.shape[0]))]))
			x_cam_to_base = points_from_base[0, :]
			y_cam_to_base = points_from_base[1, :]
			z_cam_to_base = points_from_base[2, :]


			# fig_1 = plt.figure(33)
			# ax = Axes3D(fig_1)
			# lower, upper = ax.get_ylim()
			# print("lower = {}, upper = {}".format(lower,upper))
			# ax.set_xlim(-1, 1)
			# ax.set_ylim(-1, 1)
			# ax.plot3D(x_cam_to_base, y_cam_to_base, z_cam_to_base, 'o')
			# # ax.plot3D(x_base_raw, y_base_raw, z_base_raw, 'o')
			# ax.plot3D(x_cam_to_base_pp, y_cam_to_base_pp, z_cam_to_base_pp, '-')
			# # ax.plot3D(x_base_pp, y_base_pp, z_base_pp, '-')
			# ax.plot3D(x_base, y_base, z_base, 'o')
			# ax.plot3D(x_base_hr, y_base_hr, z_base_hr, '-')
			# ax.legend(['Raw', 'fitted', 'Predicted', 'Fitted_pred'])
			# plt.show()


		else:
			# s_1_cam, v_1_cam, a_1_cam, x_cam_pp, y_cam_pp, z_cam_pp, prediction_time_step = \
			# 	post_process_coords(x_cam[:n_fit], y_cam[:n_fit], z_cam[:n_fit], t[:n_fit], base)
			s_1_cam, v_1_cam, a_1_cam, x_cam_pp, y_cam_pp, z_cam_pp, _ = \
				post_process_coords(x_cam, y_cam, z_cam, t, base)

			xyz, p, vel = tp('free', s_1_cam, v_1_cam, a_1_cam, prediction_time_step, base)

			# # Plot the data
			x = xyz[:, 0]
			y = xyz[:, 1]
			z = xyz[:, 2]

			tpred_new = np.linspace(0, (x.shape[0] - 1) * prediction_time_step, num=x.shape[0])

			resolution_interp = 255
			final_time = tpred_new[-1] + (0.7 - tpred_new[-1])
			rospy.loginfo("Prediction Fitting Started")
			trendpoly_y, trend_y, _ = myfit_coords_cam(tpred_new, y, resolution_interp, final_time, 2)
			trendpoly_x, trend_x, _ = myfit_coords_cam(tpred_new, x, resolution_interp, final_time, 3)
			trendpoly_z, trend_z, t_new = myfit_coords_cam(tpred_new, z, resolution_interp, final_time, 1)
			rospy.loginfo("Prediction Fitting Finished")

			x_cam_hr = trendpoly_x(t_new)
			y_cam_hr = trendpoly_y(t_new)
			z_cam_hr = trendpoly_z(t_new)

			pred_msg = ball_trajectory()
			pred_msg.x = x_cam_hr
			pred_msg.y = y_cam_hr
			pred_msg.z = z_cam_hr
			pred_msg.t = [prediction_time_step]

			fig_1 = plt.figure(22)
			ax = Axes3D(fig_1)
			ax.plot3D(x_cam,z_cam,-1*y_cam,'o')
			ax.plot3D(x_cam_pp, z_cam_pp, -1*y_cam_pp, '-')
			ax.plot3D(x, z, -1*y, 'o')
			ax.plot3D(x_cam_hr, z_cam_hr, -1*y_cam_hr, '-')
			ax.legend(['Raw', 'fitted', 'Predicted', 'Fitted_pred'])
			plt.show()


		rospy.loginfo("<<<<<<<<<<<Publish>>>>>>>>>>>")

		print("prediction_time_step = ", prediction_time_step)

