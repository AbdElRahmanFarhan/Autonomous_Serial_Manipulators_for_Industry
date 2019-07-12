from opensfm import transformations as tp
import numpy as np

v0_base = np.array([[0.443286,0.5623,0.632586,0.583116,0.340471,0.754448,0.6059,0.404021,0.337374,0.282957,0.774328,0.478929,0.317229],
                    [-0.183638,-0.00800076,-0.258584,-0.375281,-0.141528,-0.0425044,-0.399467,-0.529367,-0.404127,-0.474892,-0.217371,-0.332818,-0.420147],
                    [0.859916,0.6965,0.526628,0.589228,0.67779,0.403997,0.418483,0.429795,0.797568,0.58977,0.464919,0.197786,0.752833]])

v1_cam = np.array([[-1.38357,-1.40912,-1.18821,-1.13242,-1.46646,-1.27472,-1.10881,-1.16939,-1.32368,-1.28534,-1.1001,-1.25784,-1.344],
[0.00909144,0.178298,0.338233,0.27336,0.183219,0.481077,0.438966,0.416357,0.0618065,0.257493,0.402567,0.659425,0.106361],
[3.148,3.345,3.195,3.045,3.067,3.54,3.061,2.819,2.926,2.769,3.301,3.048,2.943]])

points_from_cam = np.array([[-1.07044,-1.11742,-0.863315,-0.843891,-1.16101,-0.956304,-0.819095,-0.863226,-1.00849,-0.980129,-0.810453,-0.930909],
[-0.0264473,0.141155,0.292899,0.242114,0.146446,0.430936,0.402089,0.377846,0.0282488,0.222147,0.362088,0.603154],
[2.649,2.88,2.659,2.634,2.591,3.005,2.627,2.369,2.463,2.309,2.844,2.525]
])

# points_from_cam =np.array([[0.588982,0.585597,0.588399,0.635026,0.633011,0.630657,0.674301,0.681154,0.672107,0.720505,0.718995,0.717622,0.761875,0.759941,0.75969],
# [0.375755,0.331842,0.292321,0.376464,0.334826,0.290834,0.373108,0.335261,0.288974,0.375249,0.331455,0.290456,0.373082,0.331666,0.288774],
# [1.566,1.557,1.56,1.576,1.571,1.561,1.569,1.581,1.56,1.578,1.571,1.568,1.576,1.572,1.568]
# ])
points_from_cam_test = np.array([[0.525515,0.525403,0.522962,0.568202,0.567985,0.564456,0.612571,0.607019,0.606341,0.654158,0.651666,0.648289,0.692383,0.693668,0.686361],
[0.456749,0.503442,0.54185,0.461768,0.502272,0.545899,0.460732,0.503018,0.545344,0.462095,0.502827,0.544537,0.462095,0.503816,0.543168],
[2.028,2.036,2.035,2.025,2.032,2.035,2.028,2.024,2.029,2.022,2.021,2.024,2.007,2.017,2.002]
])


# points_from_ref = np.array([[0,0,0,0.043,0.043,0.043,0.086,0.086,0.086,0.129,0.129,0.129,0.172,0.172,0.172],
# [0,0.043,0.086,0,0.043,0.086,0,0.043,0.086,0,0.043,0.086,0,0.043,0.086],
# [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])
points_from_ref = np.array([[0.443283,0.5623,0.632586,0.583116,0.340476,0.754451,0.605904,0.404023,0.337374,0.282958,0.774331,0.478938],
                            [-0.183639,-0.00800102,-0.258559,-0.375285,-0.141521,-0.0425079,-0.399436,-0.529336,-0.40413,-0.474876,-0.217383,-0.332801],
                            [0.859922,0.696499,0.526649,0.589226,0.677774,0.403999,0.418481,0.429798,0.797568,0.589751,0.464885,0.197798]]
)
# points_from_ref = np.flip(points_from_ref, axis=1)
# print("points_from_ref = {}".format(points_from_ref))
# points_from_ref_test = (points_from_ref.T + np.array([0, 0, 0.235])).T

# points_from_cam_test = np.array([[-1.38088,-1.3795,-1.38288,-1.40753,-1.41134,-1.41842,-1.43833,-1.44534,-1.45406,-1.4695,-1.47523,-1.48361,-1.50597,-1.51002,-1.51681],
# [-0.0700649,-0.0283379,0.0128551,-0.0773988,-0.0363429,0.00461825,-0.084997,-0.0442385,-0.00354864,-0.0923689,-0.0519004,-0.0115607,-0.102686,-0.0622027,-0.0194867],
# [2.581,2.563,2.554,2.549,2.546,2.544,2.531,2.529,2.53,2.51,2.506,2.502,2.499,2.488,2.486]
# ])

# Transformation from color to Depth
Translation_Vector = np.array([[-0.0148123], [-9.23669e-05], [-0.000335707]])
Rotation_Matrix = np.array([[0.999969, 0.000804914, -0.00783795],
                            [-0.000805273, 1, -4.26803e-05],
                            [0.00783791, 4.89906e-05, 0.999969]])
# point_from_color = np.array([[0.58652], [0.26804], [1.727]])
color_to_depth = np.row_stack([np.column_stack([Rotation_Matrix, Translation_Vector]), np.array([0, 0, 0, 1])])

# points_from_depth = np.dot(color_to_depth, np.column_stack([point_from_color.T,np.ones((point_from_color.shape[1],1))]).T)
# points_from_depth_1 = points_from_depth[0:-1, :]
# print("point_from_depth = {}".format(points_from_depth_1))


# Here we transform the points detected from depth frame aligned to RGB image to Depth frame aligned with IR 1.
points_from_cam_depth = np.dot(color_to_depth, np.column_stack([v1_cam.T,np.ones((v1_cam.shape[1],1))]).T)
v1_cam = points_from_cam_depth[0:-1, :]

# points_from_cam_depth = np.dot(color_to_depth, np.column_stack([points_from_cam.T,np.ones((points_from_cam.shape[1],1))]).T)
# points_from_cam = points_from_cam_depth[0:-1, :]

points_from_cam_depth_test = np.dot(color_to_depth, np.column_stack([points_from_cam_test.T,np.ones((points_from_cam_test.shape[1],1))]).T)
points_from_cam_test = points_from_cam_depth_test[0:-1, :]

translation_hc = np.array([-0.374,    -2.305065,  0.634908])
##Hard Codedd:
transformation = np.array([[0, 0, 1, translation_hc[0]],[-1, 0, 0, translation_hc[1]],[0, -1, 0, translation_hc[2]],[0, 0, 0, 1]])
pred_test = np.array([[-0.639065], [0.412908], [1.247], [1]]).reshape(4,)
pred_test_transformed = np.delete(np.dot(transformation, pred_test),-1)
pred_actual = np.array([0.873, -1.666, 0.222])
err = pred_test_transformed-pred_actual
translation_hc = translation_hc - err
print("new_translation = {}".format(translation_hc))
print("transformed_pred_test = {}".format(pred_test_transformed))
print("transformed_pred_err = {}".format(err))
# left_point = points_from_cam[:, 8]
# upper_point = points_from_cam[:, 0]
# origin_point = np.array([[-1.48885], [-0.00691594], [2.794]]).reshape(3,)
# left_point = np.array([[-1.55606], [-0.00978877], [2.759]]).reshape(3,)
# upper_point = np.array([[-1.49665], [-0.0916774], [2.82]]).reshape(3,)


origin_point = np.array([[-0.559052], [0.793845], [2.734]]).reshape(3,)
left_point = np.array([[-0.627862], [0.830621], [2.819]]).reshape(3,)
upper_point = np.array([[-0.555371], [0.701219], [2.716]]).reshape(3,)

# point_to_test = np.dot(color_to_depth, np.array([origin_point[0, 0], origin_point[1, 0], origin_point[2, 0], 1.0]).reshape(4,1))
# point_to_test = point_to_test.reshape((4,))
# origin_point = np.delete(point_to_test, -1)
# point_to_test = np.dot(color_to_depth, np.array([left_point[0, 0], left_point[1, 0], left_point[2, 0], 1.0]).reshape(4,1))
# point_to_test = point_to_test.reshape((4,))
# left_point = np.delete(point_to_test, -1)
# point_to_test = np.dot(color_to_depth, np.array([upper_point[0, 0], upper_point[1, 0], upper_point[2, 0], 1.0]).reshape(4,1))
# point_to_test = point_to_test.reshape((4,))
# upper_point = np.delete(point_to_test, -1)

x_base = (origin_point - left_point)/np.linalg.norm(origin_point - left_point)
z_base = (upper_point - origin_point)/np.linalg.norm(upper_point - origin_point)
y_base = np.cross(z_base, x_base)
# Transformation of reference origin to base frame is a translation mat.
translation = np.array([0.926, -0.4, -0.02]) + np.array([-0.4,0.3946,0])
# translation = np.array([0,0,0])
# origin_point_test = np.array([[-1.37307], [0.0569827], [2.341]])
# print("Origin point_test= {}".format(origin_point_test))
#
# origin_point = np.array([[-1.73768], [0.493835], [2.52]])
# right_point = np.array([[-0.401871], [0.465812], [2.377]])
# upper_point = np.array([[-1.74797], [-0.213531], [2.575]])
# # lower_point = np.array([[0.473433], [0.58833], [1.812]])
#
# # origin_point = points_from_cam[:, 0].reshape(3,1)
# # right_point = points_from_cam[:, 12].reshape(3,1)
# # # upper_point = np.array([[0.538866], [0.591994], [2.054]])
# # lower_point = points_from_cam[:, 2].reshape(3,1)
#
# point_to_test = np.dot(color_to_depth, np.array([origin_point[0, 0], origin_point[1, 0], origin_point[2, 0], 1.0]).reshape(4,1))
# point_to_test = point_to_test.reshape((4,))
# origin_point = np.delete(point_to_test, -1)
# point_to_test = np.dot(color_to_depth, np.array([right_point[0, 0], right_point[1, 0], right_point[2, 0], 1.0]).reshape(4,1))
# point_to_test = point_to_test.reshape((4,))
# right_point = np.delete(point_to_test, -1)
# point_to_test = np.dot(color_to_depth, np.array([upper_point[0, 0], upper_point[1, 0], upper_point[2, 0], 1.0]).reshape(4,1))
# point_to_test = point_to_test.reshape((4,))
# upper_point = np.delete(point_to_test, -1)
# # point_to_test = np.dot(color_to_depth, np.array([lower_point[0, 0], lower_point[1, 0], lower_point[2, 0], 1.0]).reshape(4,1))
# # point_to_test = point_to_test.reshape((4,))
# # lower_point = np.delete(point_to_test, -1)
#
# x_base = (right_point - origin_point)/np.linalg.norm(right_point - origin_point)
# z_base = (upper_point - origin_point)/np.linalg.norm(upper_point - origin_point)
# y_base = np.cross(z_base, x_base)
# # y_base = (lower_point - origin_point)/np.linalg.norm(lower_point - origin_point)
# # z_base = np.cross(x_base, y_base)
# # translation = np.array([0.61, -(0.02+0.043*4), 0.243])
# # translation = np.array([0.4773, -0.013, 0.743])
# translation = -np.array([0.937, -0.04, 0.277])

corner_from_cam = np.array([[1.0668], [0.280084], [1.78]]).reshape(3,)
madrab_centre = np.array([[0.556128], [-0.446898], [1.809]]).reshape(3,)

print("x_base = {}".format(x_base))
print("y_base = {}".format(y_base))
print("z_base = {}".format(z_base))
print("origin_point = ", origin_point)
# point_to_test = np.dot(color_to_depth, np.array([origin_point_test[0, 0], origin_point_test[1, 0], origin_point_test[2, 0], 1.0]).reshape(4,1))
# corner_from_base = np.dot(color_to_depth, np.array([corner_from_cam[0, 0], corner_from_cam[1, 0], corner_from_cam[2, 0], 1.0]).reshape(4,1))
# madrab_from_base = np.dot(color_to_depth, np.array([madrab_centre[0, 0], madrab_centre[1, 0], madrab_centre[2, 0], 1.0]).reshape(4,1))

# point_to_test = point_to_test.reshape((4,))
# point_to_test[0:3] = point_to_test[0:3] - origin_point
# point_to_test = np.delete(point_to_test, -1)
# corner_from_base = corner_from_cam.reshape((4,))
# corner_from_base[0:3] = corner_from_base[0:3] - origin_point
# madrab_centre_base = madrab_centre.reshape((4,))
# madrab_centre_base[0:3] = madrab_centre_base[0:3] - origin_point

# print("point_to_test = {}".format(point_to_test))

# transformed_points_from_cam = np.dot(point_to_test - origin_point, np.column_stack([x_base, y_base, z_base])) + translation
# transformed_points_from_cam_2 = np.dot(points_from_cam_test.T - origin_point, np.column_stack([x_base, y_base, z_base])) + translation
# transformed_points_from_cam_3 = np.dot((points_from_cam.T - origin_point), np.column_stack([x_base, y_base, z_base])) + translation
transformation = np.row_stack([np.column_stack([np.row_stack([x_base, y_base, z_base]), translation]), np.array([0, 0, 0, 1])])
transformed_points_from_cam = np.dot(corner_from_cam - origin_point, np.column_stack([x_base, y_base, z_base])) + translation
transformed_points_from_cam_2 = np.dot(madrab_centre - origin_point, np.column_stack([x_base, y_base, z_base])) + translation

print("transformation = ",transformation)
# transformed_points_from_cam = np.dot(transformation, point_to_test)

print("transformed_point_test = {}".format(transformed_points_from_cam))
print("from_color = {}".format(transformed_points_from_cam_2))



# all_err = transformed_points_from_cam-v0_base.T
# print("all_error = {}".format(all_err))
# print("all_error_avg = {}".format(np.mean(all_err, axis=0)))

# transformed_points_from_cam = np.dot(v1_cam.T - origin_point, np.column_stack([x_base, y_base, z_base])) + translation
# all_err = transformed_points_from_cam-v0_base.T
# print("all_error = {}".format(all_err))
# print("all_error_avg = {}".format(np.mean(all_err, axis=0)))


# x_base_test = np.dot((v1_cam[:, 0]-origin_point).reshape((1,3)),x_base.reshape((3,1)))
# y_base_test = np.dot((v1_cam[:, 0]-origin_point).reshape((1,3)),y_base.reshape((3,1)))
# z_base_test = np.dot((v1_cam[:, 0]-origin_point).reshape((1,3)),z_base.reshape((3,1)))

# x_base_test = np.dot((points_from_cam[:, 0]-origin_point).reshape((1, 3)), x_base.reshape((3, 1)))
# y_base_test = np.dot((points_from_cam[:, 0]-origin_point).reshape((1, 3)), y_base.reshape((3, 1)))
# z_base_test = np.dot((points_from_cam[:, 0]-origin_point).reshape((1, 3)), z_base.reshape((3, 1)))
#
# print("x_base_test = {}".format(x_base_test))
# print("y_base_test = {}".format(y_base_test))
# print("z_base_test = {}".format(z_base_test))
#
# point_transformed = np.array([x_base_test[0,0], y_base_test[0,0], z_base_test[0,0]]) + translation
#
# print("point_transformed_error = {}".format(point_transformed-points_from_ref[:, 0]))
transform_rot = np.column_stack([x_base, y_base, z_base])
# print("transform_rot = {}".format(transform_rot))
transformed_points_from_cam = np.dot((points_from_cam - origin_point.reshape(3,1)).T, transform_rot) + translation
all_err = transformed_points_from_cam-points_from_ref.T
print("transformed_points_from_cam = ", transformed_points_from_cam*1000)
print("all_error = {}".format(all_err))
print("all_error_avg = {}".format(np.mean(np.abs(all_err), axis=0)))
#
# transformed_points_from_cam_test = np.dot((points_from_cam_test - origin_point.reshape(3,1)).T, transform_rot) + translation
# all_err = transformed_points_from_cam_test-points_from_ref_test.T
# print("all_error = {}".format(all_err))
# print("all_error_avg = {}".format(np.mean(np.abs(all_err), axis=0)))

# transformed_points_from_cam = np.dot(points_from_cam_test.T - origin_point, transform_rot)
# all_err = transformed_points_from_cam - points_from_ref_test.T
# print("transformed_points_from_test_rotated = {}".format(transformed_points_from_cam))
# print("all_error = {}".format(all_err))
# print("all_error_avg = {}".format(np.mean(np.abs(all_err), axis=0)))


#Least squares method to find Transformation Matrix :
#========================================================

n_train = 0
n_p = v1_cam.shape[1]
n_test = 0

transform=tp.affine_matrix_from_points(v1_cam[:,0:n_p-n_train], v0_base[:,0:n_p-n_train],False,False,usesvd=False)
transformed_to_base = transform.dot(np.row_stack([v1_cam[0,n_test:], v1_cam[1,n_test:], v1_cam[2,n_test:], np.ones((1,n_p-n_test))]))
transformed_point = np.dot(transform, [[-1.61777], [-0.232737], [2.457], [1]])

print("point = ", transformed_point)
# for row in range(3):
#     print(transform[row, 0], ",", transform[row, 1], ",", transform[row, 2], ",")
# print(transform[0, 3], ",", transform[1, 3], ",", transform[2, 3])

print("The transform = ", transform)
print("points validation = ", transformed_to_base)

error = np.subtract(transformed_to_base[:-1,:], v0_base[:,n_test:]).T
print("error = ", error)
print("avg_error = {}".format(np.mean(np.abs(error), axis=0)))

