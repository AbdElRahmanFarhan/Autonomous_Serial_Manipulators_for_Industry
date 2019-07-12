#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
# from Generalized_Impact_Model import impactModel as gim
from track3d.traj_pred import mains as tp
from mpl_toolkits.mplot3d import Axes3D
from track3d.msg import  ball_trajectory
# import mathq
# from Post_Processing_Block import post_process
from Post_Processing_Block import post_process_coords,myfit_coords_base,myfit_coords_cam,myfit_pred
import rospy


if __name__ == '__main__':


	# Initialize Node
	rospy.init_node('process_and_predict', anonymous=True)
	prediction_pub = rospy.Publisher('/ball_predicted_trajectory', ball_trajectory, queue_size=10)

	accumulated_err = np.array([0.0,0.0,0.0])
	avg_err = np.array([0.0,0.0,0.0])
	err_list = []
	pred_test = False
	base = True  # from Base or from Cam
	demo = False

	for i in range(1,2):

		if i==13:
			continue

		if pred_test:
			# pred_num = "/Loggings/pred_test_sha2a/pred_test_" + str(i)
			pred_num = "/Loggings/pred_test_lab/pred_test_" + str(i)
		else:
			pred_num = "/Loggings"
		x_points = []
		y_points = []
		t_points = []
		t_pixels = []

		pred_valid_point = []

		x_coord = []
		y_coord = []
		z_coord = []

		w_0 = np.zeros(shape=[3], dtype=float)

		origin_point = np.array([0, 0, 0])

		transform = np.array([[ 0.73608089,  0.01867336,  0.67663596, -0.63030813],
       [-0.67680551,  0.0364277 ,  0.73526003, -3.45084299-0.025],
       [-0.01091851, -0.99916181,  0.03945197,  0.72712321],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

		# translation_hc = np.array([-0.36938121+0.15, -2.40334327-0.15, 0.63575143+0.125])
	   #
		# # translation_hc = np.array([-0.374,    -2.305065,  0.634908+0.123])
		# ##Hard Codedd:
		# transform = np.array([[0, 0, 1, translation_hc[0]],
		# 						   [-1, 0, 0, translation_hc[1]],
		# 						   [0, -1, 0, translation_hc[2]],
		# 						   [0, 0, 0, 1]])

		# # Poll on topic for ball_trajectory data
		f = open("/home/ahmedshehata"+ pred_num +"/ball_trajectory_pixels_x.txt", "r")
		if f.mode == 'r':
			f1 = f.readlines()
			for x in f1:
				x_points.append(float(x))
			# print(x_points)

		f = open("/home/ahmedshehata"+ pred_num +"/ball_trajectory_pixels_y.txt", "r")
		if f.mode == 'r':
			f2 = f.readlines()
			for x in f2:
				y_points.append(float(x))
			# print(y_points)
		f = open("/home/ahmedshehata"+ pred_num +"/ball_trajectory_pixels_t.txt", "r")
		if f.mode == 'r':
			f3 = f.readlines()
			for x in f3:
				t_pixels.append(float(x))
		f = open("/home/ahmedshehata"+ pred_num +"/ball_trajectory_ref_t.txt", "r")
		if f.mode == 'r':
			f3 = f.readlines()
			for x in f3:
				t_points.append(float(x))

			print(t_points)
			f = open("/home/ahmedshehata"+ pred_num +"/ball_trajectory_ref_x.txt", "r")
			if f.mode == 'r':
				f1 = f.readlines()
				for x in f1:
					x_coord.append(float(x))
				print("x_values = ", x_coord)

			f = open("/home/ahmedshehata"+ pred_num +"/ball_trajectory_ref_y.txt", "r")
			if f.mode == 'r':
				f2 = f.readlines()
				for x in f2:
					y_coord.append(float(x))

				# print(y_coord)
			f = open("/home/ahmedshehata"+ pred_num +"/ball_trajectory_ref_z.txt", "r")
			if f.mode == 'r':
				f3 = f.readlines()
				for x in f3:
					z_coord.append(float(x))
			if pred_test:
				f = open("/home/ahmedshehata"+ pred_num +"/pred_valid_point", "r")
				if f.mode == 'r':
					f3 = f.readlines()
					for x in f3:
						pred_valid_point.append(float(x))


		R_global = 1
		r_global = 0.4
		n_fit = 10

		# raw_input()

		rospy.loginfo("START_ALL")

		rospy.sleep(0.1)

		x_pixels = np.array(x_points)
		y_pixels = np.array(y_points)
		t_pix = np.array(t_pixels)

		raw_t = np.array(t_points[0:n_fit])
		raw_x_cam = np.array(x_coord[0:n_fit])
		raw_y_cam = np.array(y_coord[0:n_fit])
		raw_z_cam = np.array(z_coord[0:n_fit])

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

		prediction_time_step = 1.0 / 20.0
		print("total_filtered_points = ", x_cam.shape[0])
		# s_2, v_2, x_pp, y_pp = post_process(x_pixels, y_pixels, t_pix)

		if base:

			# s_1_cam, v_1_cam, a_1_cam, x_cam_pp, y_cam_pp, z_cam_pp, prediction_time_step = \
			# 	post_process_coords(x_cam[:n_fit], y_cam[:n_fit], z_cam[:n_fit], t[:n_fit], False)
			s_1_cam, v_1_cam, a_1_cam, x_cam_pp, y_cam_pp, z_cam_pp, _ = \
				post_process_coords(x_cam, y_cam, z_cam, t, False)

			# REMEMEBER TO UNCOMMENT THOSE IN ACTUAL LAB OR ACTUAL TRACKING
			if not demo:
				# points_raw_from_base = np.dot(transform, np.row_stack([x_cam, y_cam, z_cam, np.ones((1, x_cam.shape[0]))]))
				# x_base_raw = points_raw_from_base[0, :]
				# y_base_raw = points_raw_from_base[1, :]
				# z_base_raw = points_raw_from_base[2, :]

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
			# REMEMEBER TO COMMENT THOSE IN ACTUAL LAB OR ACTUAL TRACKING
			#######
			else:
				s_1_base = np.array([0.6, -4, 1])
				v_1_base = np.array([0, 8, 2.5])
				xyz, p, vel = tp('free', s_1_base, v_1_base, a_1_cam, prediction_time_step, True)

				# # Plot the data
				x_base = xyz[:, 0]
				y_base = xyz[:, 1]
				z_base = xyz[:, 2]

				tpred_new = np.linspace(0, (x_base.shape[0] - 1) * prediction_time_step, num=x_base.shape[0])
				resolution_interp = 2 * (x_base.shape[0])

			#######

			final_time = tpred_new[-1]+ 0.2
			rospy.loginfo("Prediction Fitting Started")
			trendpoly_y, trend_y, _ = myfit_coords_base(tpred_new, y_base, resolution_interp, final_time, 1)
			trendpoly_x, trend_x, _ = myfit_coords_base(tpred_new, x_base, resolution_interp, final_time, 3)
			trendpoly_z, trend_z, t_new = myfit_coords_base(tpred_new, z_base, resolution_interp, final_time, 2)
			rospy.loginfo("Prediction Fitting Finished")

			prediction_time_step = t_new[1]
			print("time_series = {}".format(t_new))

			x_base_hr = trendpoly_x(t_new)
			y_base_hr = trendpoly_y(t_new)
			z_base_hr = trendpoly_z(t_new)

			pred_msg = ball_trajectory()
			pred_msg.x = x_base_hr
			pred_msg.y = y_base_hr
			pred_msg.z = z_base_hr
			pred_msg.t = [prediction_time_step]

			now = rospy.get_rostime()
			print("time of publishing", now.secs+now.nsecs*(10**-9))
			prediction_pub.publish(pred_msg)

			counter = 0
			# while(True):
			# 	counter+=1
			# 	now = rospy.get_rostime()
			# 	print("time of publishing", now.secs+now.nsecs*(10**-9), counter)
			# 	prediction_pub.publish(pred_msg)
			# 	rospy.sleep(3)


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

			#
			fig_1 = plt.figure(33)
			ax = Axes3D(fig_1)
			ax.set_xlim(-1, 1)
			ax.set_ylim(-1, 1)

			ax.plot3D(x_cam_to_base, y_cam_to_base, z_cam_to_base, 'o')
			# ax.plot3D(x_base_raw, y_base_raw, z_base_raw, 'o')
			# ax.plot3D(x_cam_to_base_1, y_cam_to_base_1, z_cam_to_base_1, 'o')
			ax.plot3D(x_cam_to_base_pp, y_cam_to_base_pp, z_cam_to_base_pp, '-')
			# ax.plot3D(x_base_pp, y_base_pp, z_base_pp, '-')
			ax.plot3D(x_base, y_base, z_base, 'o')
			# ax.plot3D(x_base_1, y_base_1, z_base_1, 'o')
			ax.plot3D(x_base_hr, y_base_hr, z_base_hr, '-')

			# ax.scatter3D(ball_traj_msg.x[points_list[0]], ball_traj_msg.y[points_list[0]], ball_traj_msg.z[points_list[0]], c='r')
			ax.scatter3D(0.5623, -0.008, 0.6965,
						 c='b')

			u = np.linspace(0, np.pi, 15)
			v = np.linspace(0, 2 * np.pi, 15)
			R = R_global
			x = R * np.outer(np.sin(u), np.sin(v))
			y = R * np.outer(np.sin(u), np.cos(v))
			z = R * np.outer(np.cos(u), np.ones_like(v))
			ax.plot_wireframe(x, y, z)

			u = np.linspace(0, np.pi, 10)
			v = np.linspace(0, 2 * np.pi, 10)
			r = r_global

			x = r * np.outer(np.sin(u), np.sin(v))
			y = r * np.outer(np.sin(u), np.cos(v))
			z = r * np.outer(np.cos(u), np.ones_like(v))
			ax.plot_wireframe(x, y, z)

			ax.legend(['Raw', 'fitted',
					   'Predicted',
					   'Fitted_pred',
					   "current robot state", "outer radius", "inner radius"])
			# ax.legend(['Predicted_from_cam','predicted_from_base', "current robot state", "outer radius", "inner radius"])


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
			final_time = tpred_new[-1] + 0.2
			rospy.loginfo("Prediction Fitting Started")
			trendpoly_y, trend_y, _ = myfit_pred(tpred_new, y, resolution_interp, final_time, 2)
			trendpoly_x, trend_x, _ = myfit_pred(tpred_new, x, resolution_interp, final_time, 3)
			trendpoly_z, trend_z, t_new = myfit_pred(tpred_new, z, resolution_interp, final_time, 1)
			rospy.loginfo("Prediction Fitting Finished")

			x_cam_hr = trendpoly_x(t_new)
			y_cam_hr = trendpoly_y(t_new)
			z_cam_hr = trendpoly_z(t_new)

			pred_msg = ball_trajectory()
			pred_msg.x = x_cam_hr
			pred_msg.y = y_cam_hr
			pred_msg.z = z_cam_hr
			pred_msg.t = [prediction_time_step]

			if pred_test:
				point = np.array(pred_valid_point)

				pred_valid_index = 0
				min_err_idx = 0

				min_err_idx = np.argmin(np.sqrt((x_cam_hr - point[0])**2 + (y_cam_hr - point[1])**2 + (z_cam_hr - point[2])**2))

				err_list.append([x_cam_hr[min_err_idx]-point[0], y_cam_hr[min_err_idx]-point[1], z_cam_hr[min_err_idx]-point[2]])
				accumulated_err = accumulated_err + np.abs(np.array([x_cam_hr[min_err_idx]-point[0],
														  y_cam_hr[min_err_idx]-point[1],
														  z_cam_hr[min_err_idx]-point[2]]))

			fig_1 = plt.figure(22)
			ax = Axes3D(fig_1)
			lower, upper = ax.get_xlim()
			ax.set_ylim(0, 4)
			ax.set_xlim(-3, 1)
			ax.plot3D(x_cam, z_cam, -1 * y_cam, 'o')
			ax.plot3D(x_cam_pp, z_cam_pp, -1 * y_cam_pp, '-')
			ax.plot3D(x, z, -1 * y, 'o')
			ax.plot3D(x_cam_hr, z_cam_hr, -1 * y_cam_hr, '-')
			if pred_test:
				ax.scatter3D([point[0]], [point[2]], [-1 * point[1]], s=100, c='r')
			ax.set_xlabel("x_axis(meters)")
			ax.set_ylabel("y_axis(meters)")
			ax.set_zlabel("z_axis(meters)")
			if pred_test:
				ax.legend(['Raw', 'fitted', 'Prediction', 'Fitted_Prrediction', 'Actual_Ball_Pos'])
			else:
				ax.legend(['Raw', 'fitted', 'Prediction', 'Fitted_Prrediction'])

			# plt.show()


	plt.show()
	# sz = len(err_list)
	# avg_err = accumulated_err/sz
	# print(sz)
	# for i in range(sz):
	# 	print("err_{} = {}".format(i,err_list[i]))
	# print("avg_err = {}".format(avg_err))

	# rospy.loginfo("<<<<<<<<<<<Publish>>>>>>>>>>>")
	#
	# print("prediction_time_step = ", prediction_time_step)
	#