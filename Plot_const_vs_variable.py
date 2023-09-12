import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import transforms3d.quaternions as tf_quaternions
# from matplotlib import font_manager as fm
from tf.transformations import euler_from_quaternion
import math

# Set font properties
# font_properties = fm.FontProperties(family='Times New Roman')
# plt.rcParams['font.family'] = 'Times New Roman'


plt.rcParams["font.family"] = "serif"


# import matplotlib.font_manager
# csfont = {'fontname':'Times New Roman'}

# Set the default font
# plt.rcParams['font.family'] = 'Times New Roman'
# plt.rcParams['font.serif'] = ['Times New Roman']

# Sample data: XYZ points and quaternions
# xyz_data = np.array([
#     [1, 2, 3, 0.707, 0, 0, 0.707],  # X, Y, Z, q0, q1, q2, q3
#     [4, 5, 6, 0.5, 0.5, 0.5, 0.5],
#     [7, 8, 9, 0, 0.707, 0, 0.707]
# ])
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/ramp_0_v6/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size

print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)

xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
################################################## New

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))


##################################################
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]
##################################################




all_data = []
for i in range(6):
    filename = folder_path+'artag'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
artag_data = np.array(all_data).T

size = artag_data.shape
num_rows, num_columns = size
print("artag Number of rows:", num_rows)
print("artag Number of columns:", num_columns)


# Stack the data from all files
artag_estimate_data = np.array(all_data).T   # xyz xyz

artag_xyz_real = artag_estimate_data[:, :3]  # xyz  glboal
artag_xyz_estimate = artag_estimate_data[:, 3:] # xyz global
#######################################



# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
#
# ax.scatter(xyz_coordinates[0,0],xyz_coordinates[0,1],xyz_coordinates[0,2], color='b', label='drone')  # label='drone'
# ax.scatter(wamv_xyz_coordinates[0,0],wamv_xyz_coordinates[0,1],wamv_xyz_coordinates[0,2], color='g',label='boat') # label='boat'
# ax.scatter(artag_xyz_real[0,0],artag_xyz_real[0,1],artag_xyz_real[0,2], color='r',label='target estimation') # label='boat'
# ax.scatter(artag_xyz_estimate[0,0],artag_xyz_estimate[0,1],artag_xyz_estimate[0,2], color='k',label='target calibrated by imu') # label='boat'
#
#
# for i in range(len(xyz_coordinates)):
#     if i % 10 == 0:
#         ax.scatter(*xyz_coordinates[i], color='b' )  # label='drone'
#     if i % 20 == 0:
#         ax.scatter(*wamv_xyz_coordinates[i], color='g' )  # label='wamv'
#         ax.scatter(*artag_xyz_real[i], color='r')  # label='artag'
#         ax.scatter(*artag_xyz_estimate[i], color='k')  # label='artag_after_imu_transform'
#     rotation_matrix = tf_quaternions.quat2mat(quaternions[i])
#     orientation_vector = rotation_matrix[:, 0]  # Use the first column of rotation matrix as orientation
#     ax.quiver(
#         xyz_coordinates[i, 0], xyz_coordinates[i, 1], xyz_coordinates[i, 2],
#         orientation_vector[0], orientation_vector[1], orientation_vector[2],
#         color='r', length=0.5, arrow_length_ratio=0.2
#     )
#
# ax.set_xlabel('x-Position [meter]')
# ax.set_ylabel('y-Position [meter]')
# ax.set_zlabel('z-Position [meter]')
# plt.title('3D Position')
# plt.legend()


# plt.show()

#######################################3

# tracking error
all_data = []
for i in range(2):
    filename = folder_path+'tracking_error'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
tracking_error_xy_data = np.array(all_data).T

size = tracking_error_xy_data.shape
num_rows, num_columns = size
print("tracking_error_xy_data Number of rows:", num_rows)
print("tracking_error_xy_data Number of columns:", num_columns)

tracking_error_x = tracking_error_xy_data[:, 0]
tracking_error_y = tracking_error_xy_data[:, 1]


#######################################3
# fig2 = plt.figure()
# Extract X coordinates from the data
x_coords = xyz_data[:, 0]
y_coords = xyz_data[:, 1]
z_coords = xyz_data[:, 2]
wamv_x_coords = wamv_xyz_data[:, 0]
wamv_y_coords = wamv_xyz_data[:, 1]
wamv_z_coords = wamv_xyz_data[:, 2]


# # Create a 2D plot
# plt.plot(x_coords, marker='', linestyle='dashed', color='b', label='drone x')
# plt.plot(y_coords, marker='', linestyle='dashed', color='r', label='drone y')
# plt.plot(z_coords, marker='', linestyle='dashed', color='k', label='drone z')
# plt.plot(wamv_x_coords, marker='', linestyle='-', color='b', label='wamv x')
# plt.plot(wamv_y_coords, marker='', linestyle='-', color='r', label='wamv y')
# plt.plot(wamv_z_coords, marker='', linestyle='-', color='k', label='wamv z')
#
# plt.plot(tracking_error_x, marker='', linestyle='dashed', color='m', label='tracking error x')
# plt.plot(tracking_error_y, marker='', linestyle='-', color='m', label='tracking error y')
#
#
# # Set labels for the axis
# plt.xlabel('time')
# plt.ylabel('meter')
#
# # Set plot title
# plt.title('drone and boat distance')
# plt.legend()

##################################################


# Create a figure and axis
# fig3, ax1 = plt.subplots()

crop_data = 2831
x_coords = x_coords[crop_data:]
y_coords = y_coords[crop_data:]
z_coords = z_coords[crop_data:]
wamv_x_coords = wamv_x_coords[crop_data:]
wamv_y_coords = wamv_y_coords[crop_data:]
wamv_z_coords = wamv_z_coords[crop_data:]


# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)

# # Plot 'a' data on the left y-axis
# ax1.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
# ax1.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
# # ax1.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
# # ax1.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
#
# ax1.set_xlabel('Time [second]')
# ax1.set_ylabel('y-Position [meter]', color='k')
#
# # Create a second y-axis on the right side
# ax2 = ax1.twinx()
#
# # Plot 'b' data on the right y-axis
# ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
# ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
# ax2.set_ylabel('x-Position [meter]', color='b')
#
# # Customize the color of ax2
# ax2.spines['right'].set_color('b')  # Set the color of the right spine
# ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
#
# # Add legends
# ax1.legend(loc='upper left')
# ax2.legend(loc='upper right')
#
# plt.title('Position')

##################################################
# fig3 = plt.figure()
#
# # Create a 2D plot
# plt.plot(x_coords, marker='', linestyle='-', color='b', label='drone x')
# # plt.plot(y_coords, marker='', linestyle='dashed', color='r', label='drone y')
# # plt.plot(z_coords, marker='', linestyle='dashed', color='k', label='drone z')
# plt.plot(wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
# # plt.plot(wamv_y_coords, marker='', linestyle='-', color='r', label='wamv y')
# # plt.plot(wamv_z_coords, marker='', linestyle='-', color='k', label='wamv z')
#
# plt.plot(tracking_error_x, marker='', linestyle='-', color='m', label='tracking error x')
# # plt.plot(tracking_error_y, marker='', linestyle='-', color='m', label='tracking error y')
#
# # Set labels for the axis
# plt.xlabel('time')
# plt.ylabel('meter')
#
# # Set plot title
# plt.title('drone and boat x position')
# plt.legend()


##################################################
# fig4 = plt.figure()
#
# # Create a 2D plot
# # plt.plot(x_coords, marker='', linestyle='dashed', color='b', label='drone x')
# plt.plot(y_coords, marker='', linestyle='-', color='r', label='drone y')
# plt.plot(z_coords, marker='', linestyle='-', color='k', label='drone z')
# # plt.plot(wamv_x_coords, marker='', linestyle='-', color='b', label='wamv x')
# plt.plot(wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
# plt.plot(wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
#
# # plt.plot(tracking_error_x, marker='', linestyle='dashed', color='m', label='tracking error x')
# plt.plot(tracking_error_y, marker='', linestyle='-', color='m', label='tracking error y')
#
# # Set labels for the axis
# plt.xlabel('time')
# plt.ylabel('meter')
#
# # Set plot title
# plt.title('drone and boat y position')
# plt.legend()



##################################################



# all_data = []
# for i in range(14):
#     filename = folder_path+'speed'+str(i)+'.txt'
#     data_list = np.loadtxt(filename)
#     data_array = np.array(data_list)
#     all_data.append(data_array)
#
# # Stack the data from all files
# speed_data = np.array(all_data).T
#
# size = speed_data.shape
# num_rows, num_columns = size
# print("speed Number of rows:", num_rows)
# print("speed Number of columns:", num_columns)
#
#
# fig5 = plt.figure()
#
# # Extract X coordinates from the data
# iris_speed_x = speed_data[crop_data:, 0]
# iris_speed_y = speed_data[crop_data:, 1]
# wamv_speed_x = speed_data[crop_data:, 2]
# wamv_speed_y = speed_data[crop_data:, 3]
# estimate_speed_x = speed_data[crop_data:, 4]
# estimate_speed_y = speed_data[crop_data:, 5]
# iris_imu_measured_speed_x = speed_data[crop_data:, 6]
# iris_imu_measured_speed_y = speed_data[crop_data:, 7]
# filted_boat_speed_x = speed_data[crop_data:, 8]  # filter_wamv_speed
# filted_boat_speed_y = speed_data[crop_data:, 9]
# estimate_speed_x_trans = speed_data[crop_data:, 10]
# estimate_speed_y_trans = speed_data[crop_data:, 11]
# filted_boat_speed_x_trans = speed_data[crop_data:, 12]
# filted_boat_speed_y_trans = speed_data[crop_data:, 13]
# # Create a 2D plot
# plt.plot(x, wamv_speed_x, marker='', linestyle='dashed', color='r', label='real x-velocity boat')
# plt.plot(x, filted_boat_speed_x, marker='', linestyle='-', color='r', label='estimated x-velocity boat')
# plt.plot(x, wamv_speed_y, marker='', linestyle='dashed', color='b', label='real y-velocity boat')
# plt.plot(x, filted_boat_speed_y, marker='', linestyle='-', color='b', label='estimated y-velocity boat')
#
# # plt.plot(estimate_speed_x, marker='', linestyle='-', color='b', label='estimate boat speed x')
# # plt.plot(estimate_speed_y, marker='', linestyle='-', color='r', label='estimate boat speed y')
#
# plt.plot(x, iris_speed_x, marker='', linestyle='dashed', color='k', label='real x-velocity drone')
# # plt.plot(iris_speed_y, marker='', linestyle='dotted', color='k', label='drone speed y')
#
# plt.plot(x, iris_imu_measured_speed_x, marker='', linestyle='dashed', color='g', label='imu measured x-velocity drone')
# # plt.plot(iris_imu_measured_speed_y, marker='', linestyle='dotted', color='g', label='imu measured drone speed y')
#
#
#
#
# # plt.plot(estimate_speed_x_trans, marker='', linestyle='-', color='m', label='estimate boat speed x')
# # plt.plot(filted_boat_speed_x_trans, marker='', linestyle='dashed', color='m', label='filtered estimate boat speed x')
#
# # # Parameters for the moving average filter
# # window_size = 10  # Number of samples to average
# # # Apply moving average filter
# # smoothed_data_average = np.convolve(iris_imu_measured_speed_x, np.ones(window_size)/window_size, mode='valid')
# # plt.plot(smoothed_data_average, marker='', linestyle='-', color='r', label='average filtered estimate boat speed x')
#
# # Set labels for the axis
# plt.xlabel('Time [second]')
# plt.ylabel('Velocity [meter/second]')
#
# # Set plot title
# plt.title('Velocity')
# plt.legend()



##################################################



# fig5 = plt.figure()
#
#
# # Create a 2D plot
# # plt.plot(x, wamv_speed_x, marker='', linestyle='dashed', color='r', label='real x-velocity boat')
# # plt.plot(x, wamv_speed_y, marker='', linestyle='dashed', color='b', label='real y-velocity boat')
# # plt.plot(x, iris_speed_x, marker='', linestyle='dashed', color='k', label='real x-velocity drone')
#
# # plt.plot(iris_speed_y, marker='', linestyle='dotted', color='k', label='drone speed y')
#
# # plt.plot(x, iris_imu_measured_speed_x, marker='', linestyle='dashed', color='g', label='imu measured x-velocity drone')
# # plt.plot(iris_imu_measured_speed_y, marker='', linestyle='dotted', color='g', label='imu measured drone speed y')
#
# plt.plot(x, iris_row[crop_data:], marker='', linestyle='-', color='g', label='row')
# plt.plot(x, iris_pitch[crop_data:], marker='', linestyle='-', color='m', label='pitch')
# plt.plot(x, iris_yaw[crop_data:], marker='', linestyle='-', color='y', label='yaw')
#
#
#
#
# # Set labels for the axis
# plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree]')
#
# # Set plot title
# plt.title('Orientation')
# plt.legend()


####################

# plt.figure(figsize=(8, 4))
# plt.subplot(1, 2, 1)
# plt.plot(x, iris_row[crop_data:], marker='', linestyle='-', color='g', label='row')
# plt.plot(x, iris_pitch[crop_data:], marker='', linestyle='-', color='m', label='pitch')
# plt.plot(x, iris_yaw[crop_data:], marker='', linestyle='-', color='y', label='yaw')
#
# # Set labels for the axis
# plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree] - No Ramp')
#
# # Set plot title
# plt.title('Orientation')
# plt.legend()
#
#
#
# plt.subplot(1, 2, 2)
#
#
# plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
# plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
# # ax1.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
# # ax1.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
# plt.xlabel('Time [second]')
# plt.ylabel('y-Position [meter]', color='k')
# plt.legend(loc='upper left')
#
#
# # Create a second y-axis on the right side
# ax2 = plt.twinx()
#
# # Plot 'b' data on the right y-axis
# ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
# ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
# ax2.set_ylabel('x-Position [meter]', color='b')
#
# # Customize the color of ax2
# ax2.spines['right'].set_color('b')  # Set the color of the right spine
# ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
#
# # Add legends
# # plt.legend(loc='upper left')
# ax2.legend(loc='upper right')
# plt.tight_layout()  # Adjust subplot spacing
#
# plt.title('Position')

#==========================================================
print("============================= Read Ramp 0_v2 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageI_ PID high cannot follow/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 800
crop_end = 1600
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############


plt.figure(figsize=(8, 12))
plt.subplot(3, 2, 1)


all_data = []
for i in range(14):
    filename = folder_path+'speed'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
speed_data = np.array(all_data).T

size = speed_data.shape
num_rows, num_columns = size
print("speed Number of rows:", num_rows)
print("speed Number of columns:", num_columns)



# Extract X coordinates from the data
iris_speed_x = speed_data[crop_data:crop_end, 0]
iris_speed_y = speed_data[crop_data:crop_end, 1]
wamv_speed_x = speed_data[crop_data:crop_end, 2]
wamv_speed_y = speed_data[crop_data:crop_end, 3]
estimate_speed_x = speed_data[crop_data:crop_end, 4]
estimate_speed_y = speed_data[crop_data:crop_end, 5]
iris_imu_measured_speed_x = speed_data[crop_data:crop_end, 6]
iris_imu_measured_speed_y = speed_data[crop_data:crop_end, 7]
filted_boat_speed_x = speed_data[crop_data:crop_end, 8]  # filter_wamv_speed
filted_boat_speed_y = speed_data[crop_data:crop_end, 9]
estimate_speed_x_trans = speed_data[crop_data:crop_end, 10]
estimate_speed_y_trans = speed_data[crop_data:crop_end, 11]
filted_boat_speed_x_trans = speed_data[crop_data:crop_end, 12]
filted_boat_speed_y_trans = speed_data[crop_data:crop_end, 13]
# Create a 2D plot
plt.plot(x, wamv_speed_x, marker='', linestyle='dashed', color='r', label='real x-velocity boat')
plt.plot(x, filted_boat_speed_x, marker='', linestyle='-', color='y') # , label='estimated x-velocity boat'
# plt.plot(x, wamv_speed_y, marker='', linestyle='dashed', color='b', label='real y-velocity boat')
# plt.plot(x, filted_boat_speed_y, marker='', linestyle='-', color='b', label='estimated y-velocity boat')

# plt.plot(estimate_speed_x, marker='', linestyle='-', color='b', label='estimate boat speed x')
# plt.plot(estimate_speed_y, marker='', linestyle='-', color='r', label='estimate boat speed y')

# plt.plot(x, iris_speed_x, marker='', linestyle='dashed', color='k', label='real x-velocity drone')
# plt.plot(iris_speed_y, marker='', linestyle='dotted', color='k', label='drone speed y')

# plt.plot(x, iris_imu_measured_speed_x, marker='', linestyle='dashed', color='g', label='imu measured x-velocity drone')
# plt.plot(iris_imu_measured_speed_y, marker='', linestyle='dotted', color='g', label='imu measured drone speed y')

# plt.plot(estimate_speed_x_trans, marker='', linestyle='-', color='m', label='estimate boat speed x')
# plt.plot(filted_boat_speed_x_trans, marker='', linestyle='dashed', color='m', label='filtered estimate boat speed x')

# # Parameters for the moving average filter
# window_size = 10  # Number of samples to average
# # Apply moving average filter
# smoothed_data_average = np.convolve(iris_imu_measured_speed_x, np.ones(window_size)/window_size, mode='valid')
# plt.plot(smoothed_data_average, marker='', linestyle='-', color='r', label='average filtered estimate boat speed x')

# Set labels for the axis
# plt.xlabel('Time [second]')
plt.ylabel('[m/s]- Const PID for High altitude')




# Set labels for the axis
# plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree] - const PID for high attitude')
# plt.ylim(-25, 30)  # Adjust the range as needed
# Set plot title
plt.title('Velocity')
plt.legend()


# plt.plot(x, iris_row[crop_data:], marker='', linestyle='-', color='g', label='row')
# plt.plot(x, iris_pitch[crop_data:], marker='', linestyle='-', color='m', label='pitch')
# plt.plot(x, iris_yaw[crop_data:], marker='', linestyle='-', color='y', label='yaw')
#
# # Set labels for the axis
# # plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree] - const PID for high attitude')
# plt.ylim(-25, 30)  # Adjust the range as needed
# # Set plot title
# plt.title('Orientation')
# # plt.legend()


plt.subplot(3, 2, 2)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
# plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
plt.legend(loc='upper center')
plt.legend(loc=(0.2,0.65))

# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper center')
# ax2.legend(loc='upper right')
plt.tight_layout()  # Adjust subplot spacing

plt.title('Position')

##################################################


#==========================================================
print("============================= Read Ramp 1 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageII_PID_low attitude/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 0
crop_end = 700 # len(xyz_data[:,0])
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############


# plt.figure(figsize=(8, 4))
plt.subplot(3, 2, 3)
# plt.plot(x, iris_row[crop_data:crop_end], marker='', linestyle='-', color='g', label='row')
# plt.plot(x, iris_pitch[crop_data:crop_end], marker='', linestyle='-', color='m', label='pitch')
# plt.plot(x, iris_yaw[crop_data:crop_end], marker='', linestyle='-', color='y', label='yaw')

all_data = []
for i in range(14):
    filename = folder_path+'speed'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
speed_data = np.array(all_data).T

size = speed_data.shape
num_rows, num_columns = size
print("speed Number of rows:", num_rows)
print("speed Number of columns:", num_columns)


# Extract X coordinates from the data
iris_speed_x = speed_data[crop_data:crop_end, 0]
iris_speed_y = speed_data[crop_data:crop_end, 1]
wamv_speed_x = speed_data[crop_data:crop_end, 2]
wamv_speed_y = speed_data[crop_data:crop_end, 3]
estimate_speed_x = speed_data[crop_data:crop_end, 4]
estimate_speed_y = speed_data[crop_data:crop_end, 5]
iris_imu_measured_speed_x = speed_data[crop_data:crop_end, 6]
iris_imu_measured_speed_y = speed_data[crop_data:crop_end, 7]
filted_boat_speed_x = speed_data[crop_data:crop_end, 8]  # filter_wamv_speed
filted_boat_speed_y = speed_data[crop_data:crop_end, 9]
estimate_speed_x_trans = speed_data[crop_data:crop_end, 10]
estimate_speed_y_trans = speed_data[crop_data:crop_end, 11]
filted_boat_speed_x_trans = speed_data[crop_data:crop_end, 12]
filted_boat_speed_y_trans = speed_data[crop_data:crop_end, 13]
# Create a 2D plot
plt.plot(x, wamv_speed_x, marker='', linestyle='dashed', color='r') # , label='real x-velocity boat'
plt.plot(x, filted_boat_speed_x, marker='', linestyle='-', color='y', label='estimated x-velocity boat')
# plt.plot(x, wamv_speed_y, marker='', linestyle='dashed', color='b', label='real y-velocity boat')
# plt.plot(x, filted_boat_speed_y, marker='', linestyle='-', color='b', label='estimated y-velocity boat')

# plt.plot(estimate_speed_x, marker='', linestyle='-', color='b', label='estimate boat speed x')
# plt.plot(estimate_speed_y, marker='', linestyle='-', color='r', label='estimate boat speed y')

# plt.plot(x, iris_speed_x, marker='', linestyle='dashed', color='k', label='real x-velocity drone')
# plt.plot(iris_speed_y, marker='', linestyle='dotted', color='k', label='drone speed y')

# plt.plot(x, iris_imu_measured_speed_x, marker='', linestyle='dashed', color='g', label='imu measured x-velocity drone')
# plt.plot(iris_imu_measured_speed_y, marker='', linestyle='dotted', color='g', label='imu measured drone speed y')

# plt.plot(estimate_speed_x_trans, marker='', linestyle='-', color='m', label='estimate boat speed x')
# plt.plot(filted_boat_speed_x_trans, marker='', linestyle='dashed', color='m', label='filtered estimate boat speed x')

# # Parameters for the moving average filter
# window_size = 10  # Number of samples to average
# # Apply moving average filter
# smoothed_data_average = np.convolve(iris_imu_measured_speed_x, np.ones(window_size)/window_size, mode='valid')
# plt.plot(smoothed_data_average, marker='', linestyle='-', color='r', label='average filtered estimate boat speed x')

# Set labels for the axis
# plt.xlabel('Time [second]')
plt.ylabel('[m/s]- const PID for Low altitude')


# # Set labels for the axis
# # plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree] - const PID for low attitude')
# plt.ylim(-25, 30)  # Adjust the range as needed
# # Set plot title
# # plt.title('Orientation')
plt.legend(loc='lower left')


plt.subplot(3, 2, 4)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
# plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
# plt.legend(loc='upper left')


# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper left')
ax2.legend(loc='upper center')
plt.tight_layout()  # Adjust subplot spacing

# plt.title('Position')



#==========================================================
print("============================= Read Ramp 2 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageIII_variablePID/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 0
crop_end = 707# len(xyz_data[:,0])
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############


# plt.figure(figsize=(8, 4))
plt.subplot(3, 2, 5)
# plt.plot(x, iris_row[crop_data:crop_end], marker='', linestyle='-', color='g', label='row')
# plt.plot(x, iris_pitch[crop_data:crop_end], marker='', linestyle='-', color='m', label='pitch')
# plt.plot(x, iris_yaw[crop_data:crop_end], marker='', linestyle='-', color='y', label='yaw')



all_data = []
for i in range(14):
    filename = folder_path+'speed'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
speed_data = np.array(all_data).T

size = speed_data.shape
num_rows, num_columns = size
print("speed Number of rows:", num_rows)
print("speed Number of columns:", num_columns)


# Extract X coordinates from the data
iris_speed_x = speed_data[crop_data:crop_end, 0]
iris_speed_y = speed_data[crop_data:crop_end, 1]
wamv_speed_x = speed_data[crop_data:crop_end, 2]
wamv_speed_y = speed_data[crop_data:crop_end, 3]
estimate_speed_x = speed_data[crop_data:crop_end, 4]
estimate_speed_y = speed_data[crop_data:crop_end, 5]
iris_imu_measured_speed_x = speed_data[crop_data:crop_end, 6]
iris_imu_measured_speed_y = speed_data[crop_data:crop_end, 7]
filted_boat_speed_x = speed_data[crop_data:crop_end, 8]  # filter_wamv_speed
filted_boat_speed_y = speed_data[crop_data:crop_end, 9]
estimate_speed_x_trans = speed_data[crop_data:crop_end, 10]
estimate_speed_y_trans = speed_data[crop_data:crop_end, 11]
filted_boat_speed_x_trans = speed_data[crop_data:crop_end, 12]
filted_boat_speed_y_trans = speed_data[crop_data:crop_end, 13]
# Create a 2D plot
plt.plot(x, wamv_speed_x, marker='', linestyle='dashed', color='r', label='real x-velocity boat')
plt.plot(x, filted_boat_speed_x, marker='', linestyle='-', color='y', label='estimated x-velocity boat')


# Set labels for the axis
plt.xlabel('Time [second]')
plt.ylabel('[m/s]- Adaptive PID')



# Set labels for the axis
# plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree] - Adaptive PID')
# plt.ylim(-25, 30)  # Adjust the range as needed

# Set plot title
# plt.title('Orientation')
# plt.legend()




plt.subplot(3, 2, 6)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
# plt.legend(loc='upper left')


# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper left')
# ax2.legend(loc='upper right')
plt.tight_layout()  # Adjust subplot spacing
# plt.title('Position')


#==========================================================

fig = plt.figure(figsize=(18, 4.8))

# fig = plt.figure(figsize=(16, 4))




print("============================= Read Ramp 0_v2 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageI_ PID high cannot follow/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 800
crop_end = 1600
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############

plt.subplot(1, 4, 1)

all_data = []
for i in range(14):
    filename = folder_path+'speed'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
speed_data = np.array(all_data).T

size = speed_data.shape
num_rows, num_columns = size
print("speed Number of rows:", num_rows)
print("speed Number of columns:", num_columns)


# Extract X coordinates from the data
iris_speed_x = speed_data[crop_data:crop_end, 0]
iris_speed_y = speed_data[crop_data:crop_end, 1]
wamv_speed_x = speed_data[crop_data:crop_end, 2]
wamv_speed_y = speed_data[crop_data:crop_end, 3]
estimate_speed_x = speed_data[crop_data:crop_end, 4]
estimate_speed_y = speed_data[crop_data:crop_end, 5]
iris_imu_measured_speed_x = speed_data[crop_data:crop_end, 6]
iris_imu_measured_speed_y = speed_data[crop_data:crop_end, 7]
filted_boat_speed_x = speed_data[crop_data:crop_end, 8]  # filter_wamv_speed
filted_boat_speed_y = speed_data[crop_data:crop_end, 9]
estimate_speed_x_trans = speed_data[crop_data:crop_end, 10]
estimate_speed_y_trans = speed_data[crop_data:crop_end, 11]
filted_boat_speed_x_trans = speed_data[crop_data:crop_end, 12]
filted_boat_speed_y_trans = speed_data[crop_data:crop_end, 13]
# Create a 2D plot
plt.plot(x, wamv_speed_x, marker='', linestyle='dashed', color='y', label='real x-velocity boat')
# plt.plot(x, filted_boat_speed_x, marker='', linestyle='-', color='y') # , label='estimated x-velocity boat'
# plt.plot(x, wamv_speed_y, marker='', linestyle='dashed', color='b', label='real y-velocity boat')
# Set labels for the axis
plt.xlabel('Time [second]')
plt.ylabel('Velocity [m/s]')


# Set labels for the axis
# plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree] - const PID for high attitude')
# plt.ylim(-25, 30)  # Adjust the range as needed
# Set plot title
plt.title('Velocity of the Target')
plt.legend()


plt.subplot(1, 4, 2)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='boat y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='boat z')
plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
plt.legend(loc='upper center')
plt.legend(loc=(0.2,0.75))

# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='boat x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper center')
# ax2.legend(loc='upper right')
plt.tight_layout()  # Adjust subplot spacing

plt.title('Const PID for High altitude')


point_to_highlight_x = 22.2 # x-coordinate of the point to highlight
point_to_highlight_y = 8.5 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)


#==========================================================
print("============================= Read Ramp 1 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageII_PID_low attitude/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 0
crop_end = 700 # len(xyz_data[:,0])
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############


plt.subplot(1, 4, 3)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='boat y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='boat z')
plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
# plt.legend(loc='upper left')


# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='boat x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper left')
ax2.legend(loc='upper center')
plt.tight_layout()  # Adjust subplot spacing

plt.title('Const PID for Low altitude')




point_to_highlight_x = 25.3  # x-coordinate of the point to highlight
point_to_highlight_y = 27.5 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)

#==========================================================
print("============================= Read Ramp 2 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageIII_variablePID/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 0
crop_end = 707# len(xyz_data[:,0])
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############






##############################



plt.subplot(1, 4, 4)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
# plt.legend(loc='upper left')


# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper left')
# ax2.legend(loc='upper right')
plt.tight_layout()  # Adjust subplot spacing
plt.title('Adaptive PID')


point_to_highlight_x = 35  # x-coordinate of the point to highlight
point_to_highlight_y = 43 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)

point_to_highlight_x = 35  # x-coordinate of the point to highlight
point_to_highlight_y = 8.4 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)


point_to_highlight_x = 35  # x-coordinate of the point to highlight
point_to_highlight_y = 1.4 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)
#==========================================================


















#==========================================================

# fig = plt.figure(figsize=(18, 4.8))
fig = plt.figure(figsize=(16, 4))




print("============================= Read Ramp 0_v2 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageI_ PID high cannot follow/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 800
crop_end = 1600
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############

plt.subplot(1, 4, 1)

all_data = []
for i in range(14):
    filename = folder_path+'speed'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
speed_data = np.array(all_data).T

size = speed_data.shape
num_rows, num_columns = size
print("speed Number of rows:", num_rows)
print("speed Number of columns:", num_columns)


# Extract X coordinates from the data
iris_speed_x = speed_data[crop_data:crop_end, 0]
iris_speed_y = speed_data[crop_data:crop_end, 1]
wamv_speed_x = speed_data[crop_data:crop_end, 2]
wamv_speed_y = speed_data[crop_data:crop_end, 3]
estimate_speed_x = speed_data[crop_data:crop_end, 4]
estimate_speed_y = speed_data[crop_data:crop_end, 5]
iris_imu_measured_speed_x = speed_data[crop_data:crop_end, 6]
iris_imu_measured_speed_y = speed_data[crop_data:crop_end, 7]
filted_boat_speed_x = speed_data[crop_data:crop_end, 8]  # filter_wamv_speed
filted_boat_speed_y = speed_data[crop_data:crop_end, 9]
estimate_speed_x_trans = speed_data[crop_data:crop_end, 10]
estimate_speed_y_trans = speed_data[crop_data:crop_end, 11]
filted_boat_speed_x_trans = speed_data[crop_data:crop_end, 12]
filted_boat_speed_y_trans = speed_data[crop_data:crop_end, 13]
# Create a 2D plot
plt.plot(x, wamv_speed_x, marker='', linestyle='dashed', color='y', label='real x-velocity boat')
# plt.plot(x, filted_boat_speed_x, marker='', linestyle='-', color='y') # , label='estimated x-velocity boat'
# plt.plot(x, wamv_speed_y, marker='', linestyle='dashed', color='b', label='real y-velocity boat')
# Set labels for the axis
plt.xlabel('Time [second]')
plt.ylabel('Velocity [m/s]')


# Set labels for the axis
# plt.xlabel('Time [second]')
# plt.ylabel('Angle [degree] - const PID for high attitude')
# plt.ylim(-25, 30)  # Adjust the range as needed
# Set plot title
plt.title('Velocity of the Target')
plt.legend()


plt.subplot(1, 4, 2)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='boat y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='boat z')
plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
plt.legend(loc='upper center')
plt.legend(loc=(0.2,0.69))

# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='boat x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper center')
# ax2.legend(loc='upper right')
plt.tight_layout()  # Adjust subplot spacing

plt.title('Const PID for High altitude')


point_to_highlight_x = 22.2 # x-coordinate of the point to highlight
point_to_highlight_y = 8.5 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)


#==========================================================
print("============================= Read Ramp 1 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageII_PID_low attitude/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 0
crop_end = 700 # len(xyz_data[:,0])
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############


plt.subplot(1, 4, 3)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='boat y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='boat z')
plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
# plt.legend(loc='upper left')


# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='boat x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper left')
ax2.legend(loc='upper center')
plt.tight_layout()  # Adjust subplot spacing

plt.title('Const PID for Low altitude')




point_to_highlight_x = 25.3  # x-coordinate of the point to highlight
point_to_highlight_y = 27.5 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)

#==========================================================
print("============================= Read Ramp 2 ======================")
folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/Aug27_stageIII_variablePID/'
all_data = []
for i in range(7):
    filename = folder_path+'iris'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
xyz_data = np.array(all_data).T
size = xyz_data.shape
num_rows, num_columns = size
print("iris Number of rows:", num_rows)
print("iris Number of columns:", num_columns)
xyz_coordinates = xyz_data[:, :3]
quaternions = xyz_data[:, 3:]
#############

iris_qx = quaternions[:,0]
iris_qy = quaternions[:,1]
iris_qz = quaternions[:,2]
iris_qw = quaternions[:,3]
iris_row = []
iris_pitch = []
iris_yaw = []
for i in range(len(iris_qx)):
    iris_quaternion = (iris_qx[i], iris_qy[i], iris_qz[i], iris_qw[i])
    row, pitch, yaw = euler_from_quaternion(iris_quaternion)
    iris_row.append(math.degrees(row))
    iris_pitch.append(math.degrees(pitch))
    iris_yaw.append(math.degrees(yaw))
############
all_data = []
for i in range(7):
    filename = folder_path+'wamv'+str(i)+'.txt'
    data_list = np.loadtxt(filename)
    data_array = np.array(data_list)
    all_data.append(data_array)

# Stack the data from all files
wamv_xyz_data = np.array(all_data).T

size = wamv_xyz_data.shape
num_rows, num_columns = size
print("wamv Number of rows:", num_rows)
print("wamv Number of columns:", num_columns)

wamv_xyz_coordinates = wamv_xyz_data[:, :3]
wamv_quaternions = wamv_xyz_data[:, 3:]

############
crop_data = 0
crop_end = 707# len(xyz_data[:,0])
# Extract X coordinates from the data
x_coords = xyz_data[crop_data:crop_end, 0]
y_coords = xyz_data[crop_data:crop_end, 1]
z_coords = xyz_data[crop_data:crop_end, 2]
wamv_x_coords = wamv_xyz_data[crop_data:crop_end, 0]
wamv_y_coords = wamv_xyz_data[crop_data:crop_end, 1]
wamv_z_coords = wamv_xyz_data[crop_data:crop_end, 2]
############

# 20 Hz = 0.05 second
Total_len = len(x_coords)
print("Total_len:", Total_len)
Total_time = 0.05*Total_len
# Total_time = Total_len
x = np.linspace(0, Total_time, Total_len)
############






##############################



plt.subplot(1, 4, 4)

plt.plot(x, y_coords, marker='', linestyle='-', color='r', label='drone y')
plt.plot(x, wamv_y_coords, marker='', linestyle='dashed', color='r', label='wamv y')
plt.plot(x, z_coords, marker='', linestyle='-', color='k', label='drone z')
plt.plot(x, wamv_z_coords, marker='', linestyle='dashed', color='k', label='wamv z')
plt.xlabel('Time [second]')
plt.ylabel('y-z-Position [meter]', color='k')
# plt.legend(loc='upper left')


# Create a second y-axis on the right side
ax2 = plt.twinx()

# Plot 'b' data on the right y-axis
ax2.plot(x, x_coords, marker='', linestyle='-', color='b', label='drone x')
ax2.plot(x, wamv_x_coords, marker='', linestyle='dashed', color='b', label='wamv x')
ax2.set_ylabel('x-Position [meter]', color='b')

# Customize the color of ax2
ax2.spines['right'].set_color('b')  # Set the color of the right spine
ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis

# Add legends
# plt.legend(loc='upper left')
# ax2.legend(loc='upper right')
plt.tight_layout()  # Adjust subplot spacing
plt.title('Adaptive PID')


point_to_highlight_x = 35  # x-coordinate of the point to highlight
point_to_highlight_y = 43 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)

point_to_highlight_x = 35  # x-coordinate of the point to highlight
point_to_highlight_y = 8.4 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)


point_to_highlight_x = 35  # x-coordinate of the point to highlight
point_to_highlight_y = 1.4 # y-coordinate of the point to highlight
# plt.scatter(point_to_highlight_x, point_to_highlight_y, color='y', marker='o')

# Highlight the specific point with a big circle and dashed line
plt.scatter(
    point_to_highlight_x,
    point_to_highlight_y,
    color='gold',
    label='highlighted point',
    marker='o',
    facecolor='none',  # Make the marker transparent
    s=500,  # Size of the marker
    linestyle='dashed',  # Dashed line around the marker
    linewidth=2  # Line width of the dashed line
)

plt.tight_layout()  # Adjust subplot spacing

#==========================================================
# Show the plot
plt.show()






