import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import transforms3d.quaternions as tf_quaternions

# Sample data: XYZ points and quaternions
# xyz_data = np.array([
#     [1, 2, 3, 0.707, 0, 0, 0.707],  # X, Y, Z, q0, q1, q2, q3
#     [4, 5, 6, 0.5, 0.5, 0.5, 0.5],
#     [7, 8, 9, 0, 0.707, 0, 0.707]
# ])

all_data = []
for i in range(7):
    filename = 'iris'+str(i)+'.txt'
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
##################################################
all_data = []
for i in range(7):
    filename = 'wamv'+str(i)+'.txt'
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


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# for i in range(len(xyz_coordinates)):
#     if i % 10 == 0:
#         ax.scatter(*xyz_coordinates[i], color='b')
#     if i % 20 == 0:
#         ax.scatter(*wamv_xyz_coordinates[i], color='g')
#     rotation_matrix = tf_quaternions.quat2mat(quaternions[i])
#     orientation_vector = rotation_matrix[:, 0]  # Use the first column of rotation matrix as orientation
#     ax.quiver(
#         xyz_coordinates[i, 0], xyz_coordinates[i, 1], xyz_coordinates[i, 2],
#         orientation_vector[0], orientation_vector[1], orientation_vector[2],
#         color='r', length=0.5, arrow_length_ratio=0.2
#     )
#
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
#
# plt.show()

fig2 = plt.figure()
# Extract X coordinates from the data
x_coords = xyz_data[:, 0]
y_coords = xyz_data[:, 1]
z_coords = xyz_data[:, 2]
wamv_x_coords = wamv_xyz_data[:, 0]
wamv_y_coords = wamv_xyz_data[:, 1]
wamv_z_coords = wamv_xyz_data[:, 2]


# Create a 2D plot
plt.plot(x_coords, marker='', linestyle='dashed', color='b', label='drone x')
plt.plot(y_coords, marker='', linestyle='dashed', color='r', label='drone y')
plt.plot(z_coords, marker='', linestyle='dashed', color='k', label='drone z')
plt.plot(wamv_x_coords, marker='', linestyle='-', color='b', label='wamv x')
plt.plot(wamv_y_coords, marker='', linestyle='-', color='r', label='wamv y')
plt.plot(wamv_z_coords, marker='', linestyle='-', color='k', label='wamv z')

# Set labels for the axis
plt.xlabel('time')
plt.ylabel('meter')

# Set plot title
plt.title('drone and boat distance')
plt.legend()

# Show the plot
plt.show()