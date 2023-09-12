# importing the required module
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "serif"

Kp = []
Ki = []
Kd = []
gz = []
err = []
with open("z_training_Feb_28_v2/best_particle_PID_result.txt", 'r') as file_data:
    count = 0
    for line in file_data:
        data = line.split()
        print(data)
        Kp.append(float(data[0]))
        Ki.append(float(data[1]))
        Kd.append(float(data[2]))
        gz.append(float(data[3]))
        err.append(float(data[4]))
        count +=1
    print(count)
# x axis values
# x = [1, 2, 3]
x = list(range(0,count))
# corresponding y axis values
#y = [2, 4, 1]

# Kp = round(Kp, 4)
# Ki = round(Ki, 4)
# Kd = round(Kd, 4)

# plotting the points
# #plt.plot(x, y)
# plt.figure(0)
# plt.plot(x, Kp, label = "Kp")
# plt.plot(x, Ki, label = "Ki")
# plt.plot(x, Kd, label = "Kd")
# plt.plot(x, gz, label = "gz")
# plt.legend()
# plt.xlabel('x - axis - iteration') # naming the x axis
# plt.ylabel('y - axis - value') # naming the y axis
# #plt.ylim([0, 5])
# # giving a title to my graph
# plt.title('Best particle')
# # function to show the plot
#
# plt.figure(1)
# plt.xlabel('x - axis - iteration')
# plt.ylabel('y - axis - value')
# plt.title('Cost function')
# plt.plot(x, err, label = "err")

########################################

# plt.figure(2)

# Create a figure and axis
plt.figure(figsize=(8, 8))
plt.subplot(2, 2, 1)


# fig, ax1 = plt.subplots(figsize=(5, 7))


# Plot 'b' data on the right y-axis
plt.plot(x, err, marker='', linestyle='-', color='b', label='Cost Function')
# plt.xlabel('Iteration') # naming the x axis
plt.ylabel('Value', color='b') # naming the y axis
plt.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# ax1.spines['left'].set_color('b')  # Set the color of the right spine
# ax1.set_xticklabels(x, fontsize=12)
plt.tick_params(axis='both', which='both')  # Adjust tick label font size
# Add legends
plt.legend(bbox_to_anchor=(1, 0.3)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2


# Create a second y-axis on the right side
ax2 = plt.twinx()
ax2.plot(x, Kp, label = "Kp", color='steelblue')
ax2.plot(x, Ki, label = "Ki", color='orange')
ax2.plot(x, Kd, label = "Kd", color='limegreen')
ax2.plot(x, gz, label = r"$\alpha$", color='salmon')
# ax2.legend(fontsize=13)
ax2.tick_params(axis='both', which='both')  # Adjust tick label font size


# ax2.set_ylabel('x-Position [meter]', color='b')
# Customize the color of ax2
ax2.spines['left'].set_color('b')  # Set the color of the right spine
# ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis


plt.title('Best Particle and Cost Function')


##################################################
plt.subplot(2, 2, 3)


# fig, ax1 = plt.subplots(figsize=(5, 7))

# Plot 'b' data on the right y-axis
plt.plot(x, err, marker='', linestyle='-', color='b', label='Cost Function')
plt.xlabel('Iteration') # naming the x axis
plt.ylabel('Value', color='b') # naming the y axis
plt.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# ax1.spines['left'].set_color('b')  # Set the color of the right spine
# ax1.set_xticklabels(x, fontsize=12)
plt.tick_params(axis='both', which='both')  # Adjust tick label font size
# Add legends
# plt.legend(bbox_to_anchor=(1, 0.3)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2

# Create a second y-axis on the right side
ax2 = plt.twinx()
ax2.plot(x, Kp, label = "Kp", color='steelblue')
ax2.plot(x, Ki, label = "Ki", color='orange')
ax2.plot(x, Kd, label = "Kd", color='limegreen')
ax2.plot(x, gz, label = r"$\alpha$", color='salmon')
# ax2.legend(fontsize=13)
ax2.tick_params(axis='both', which='both')  # Adjust tick label font size

# ax2.set_ylabel('x-Position [meter]', color='b')
# Customize the color of ax2
ax2.spines['left'].set_color('b')  # Set the color of the right spine
# ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis




##################################################




# save_particle(filename, kp, Ki, Kd, gz, vz, az, dist, result)


def read_global(filename, plot_num, plot_name):
    Kp = []
    Ki = []
    Kd = []
    err = []
    with open(filename, 'r') as file_data:
        count = 0
        for line in file_data:
            data = line.split()
            #print(data)
            Kp.append(float(data[0]))
            Ki.append(float(data[1]))
            Kd.append(float(data[2]))
            err.append(float(data[3]))
            count +=1
        print(count)
    x = list(range(0,count))

    # plotting the points
    plt.figure(plot_num)
    plt.plot(x, Kp, label="Kp")
    plt.plot(x, Ki, label="Ki")
    plt.plot(x, Kd, label="Kd")
    plt.legend()
    # naming the x axis
    plt.xlabel('x - axis - iteration')
    # naming the y axis
    plt.ylabel('y - axis - value')
    # plt.ylim([0, 5])
    # giving a title to my graph
    plt.title(plot_name)

    return x, Kp, Ki, Kd, err

def read_particle(filename, plot_num, plot_name):
    Kp = []
    Ki = []
    Kd = []
    gz = []
    vz = []
    az = []
    dist = []
    err = []
    with open(filename, 'r') as file_data:
        count = 0
        for line in file_data:
            data = line.split()
            #print(data)
            Kp.append(float(data[0]))
            Ki.append(float(data[1]))
            Kd.append(float(data[2]))
            gz.append(float(data[3]))
            vz.append(float(data[4]))
            az.append(float(data[5]))
            dist.append(float(data[6]))
            err.append(float(data[7]))
            count +=1
        print(count)
    x = list(range(0,count))

    # # plotting the points
    # plt.figure(plot_num)
    # plt.plot(x, Kp, label="Kp")
    # plt.plot(x, Ki, label="Ki")
    # plt.plot(x, Kd, label="Kd")
    # plt.plot(x, gz, label=r"$\alpha$")
    # plt.legend()
    # # naming the x axis
    # plt.xlabel('x - axis - iteration')
    # # naming the y axis
    # plt.ylabel('y - axis - value')
    # # plt.ylim([0, 5])
    # # giving a title to my graph
    # plt.title(plot_name)

    return x, Kp, Ki, Kd, gz, vz, az, dist, err

#directory_name = "PID_training/"
#directory_name = "PID_training_Feb_7_with 505 iter/"
#directory_name = "z_training_Feb_28_v2/"
directory_name = "z_training_Feb_28_v2/"
# save_particle(filename, kp, Ki, Kd, gz, vz, az, dist, result)

x, Kp_1, Ki_1, Kd_1, gz_1, vz, az, dist, err_0 = read_particle(directory_name+"0th_particle_PID_result.txt",2,'1st particle')
x, Kp_2, Ki_2, Kd_2, gz_2, vz, az, dist, err_1 = read_particle(directory_name+"1th_particle_PID_result.txt",3,'2nd particle')
x, Kp_3, Ki_3, Kd_3, gz_3, vz, az, dist, err_2 = read_particle(directory_name+"2th_particle_PID_result.txt",4,'3rd particle')
x, Kp_4, Ki_4, Kd_4, gz_4, vz, az, dist, err_3 = read_particle(directory_name+"3th_particle_PID_result.txt",5,'4th particle')
x, Kp_5, Ki_5, Kd_5, gz_5, vz, az, dist, err_4 = read_particle(directory_name+"4th_particle_PID_result.txt",6,'5th particle')


# x, Kp, Ki, Kd, err_best = read_global(directory_name+"best_particle_PID_result.txt",7,'Best_particle')
#
# plt.figure(8)
# # plt.plot(x[0:99], err_0[0:99], label = "0th_particle")
# # plt.plot(x[0:99], err_1[0:99], label = "1th_particle")
# # plt.plot(x[0:99], err_2[0:99], label = "2th_particle")
# # plt.plot(x[0:99], err_3[0:99], label = "3th_particle")
# # plt.plot(x[0:99], err_4[0:99], label = "4th_particle")
# plt.plot(x, err_best, label = "Best_particle")
# plt.title("Error between the drone and the landing spot")
#
# plt.legend()
# # naming the x axis
# plt.xlabel('x - axis - iteration')
# # naming the y axis
# plt.ylabel('y - axis - error')
# plt.title('Error between the drone and the landing spot')






########################################


# Create a figure and axis
# fig, ax1 = plt.subplots(figsize=(5, 7))
plt.subplot(2, 2, 2)

# Plot 'b' data on the right y-axis
plt.plot(x, Kp_1, marker='', linestyle='-', color='steelblue', label='Kp')
plt.plot(x, Kp_2, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_3, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_4, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_5, marker='', linestyle='-', color='steelblue')

# Plot 'b' data on the right y-axis
plt.plot(x, Ki_1, marker='', linestyle='-', color='orange', label='Ki')
plt.plot(x, Ki_2, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_3, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_4, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_5, marker='', linestyle='-', color='orange')

# Plot 'b' data on the right y-axis
plt.plot(x, Kd_1, marker='', linestyle='-', color='limegreen', label='Kd')
plt.plot(x, Kd_2, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_3, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_4, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_5, marker='', linestyle='-', color='limegreen')

# Plot 'b' data on the right y-axis
plt.plot(x, gz_1, marker='', linestyle='-', color='salmon', label=r"$\alpha$")
plt.plot(x, gz_2, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_3, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_4, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_5, marker='', linestyle='-', color='salmon')


# plt.xlabel('Iteration') # naming the x axis
# plt.ylabel('Value') # naming the y axis
# ax1.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# ax1.spines['left'].set_color('b')  # Set the color of the right spine
# ax1.set_xticklabels(x, fontsize=12)
plt.tick_params(axis='both', which='both')  # Adjust tick label font size
plt.legend()

# Add legends
plt.legend(bbox_to_anchor=(1, 0.5)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2
plt.tight_layout()  # Adjust subplot spacing
plt.title('All Particles Trajectories')





##################################################
# Create a figure and axis
# fig, ax1 = plt.subplots(figsize=(5, 7))
plt.subplot(2, 2, 4)

# Plot 'b' data on the right y-axis
plt.plot(x, Kp_1, marker='', linestyle='-', color='steelblue', label='Kp')
plt.plot(x, Kp_2, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_3, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_4, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_5, marker='', linestyle='-', color='steelblue')

# Plot 'b' data on the right y-axis
plt.plot(x, Ki_1, marker='', linestyle='-', color='orange', label='Ki')
plt.plot(x, Ki_2, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_3, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_4, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_5, marker='', linestyle='-', color='orange')

# Plot 'b' data on the right y-axis
plt.plot(x, Kd_1, marker='', linestyle='-', color='limegreen', label='Kd')
plt.plot(x, Kd_2, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_3, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_4, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_5, marker='', linestyle='-', color='limegreen')

# Plot 'b' data on the right y-axis
plt.plot(x, gz_1, marker='', linestyle='-', color='salmon', label=r"$\alpha$")
plt.plot(x, gz_2, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_3, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_4, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_5, marker='', linestyle='-', color='salmon')


plt.xlabel('Iteration') # naming the x axis
# plt.ylabel('Value') # naming the y axis
# ax1.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# ax1.spines['left'].set_color('b')  # Set the color of the right spine
# ax1.set_xticklabels(x, fontsize=12)
plt.tick_params(axis='both', which='both')  # Adjust tick label font size
# plt.legend()

# Add legends
# plt.legend(bbox_to_anchor=(1, 0.5)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2
plt.tight_layout()  # Adjust subplot spacing
# plt.title('All Particles Trajectories')


################==============================================


# Create a figure and axis
plt.figure(figsize=(8, 4))
# plt.subplot(2, 2, 1)
#
#
# # fig, ax1 = plt.subplots(figsize=(5, 7))
#
#
# # Plot 'b' data on the right y-axis
# plt.plot(x, err, marker='', linestyle='-', color='b', label='Cost Function')
# # plt.xlabel('Iteration') # naming the x axis
# plt.ylabel('Value', color='b') # naming the y axis
# plt.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# # ax1.spines['left'].set_color('b')  # Set the color of the right spine
# # ax1.set_xticklabels(x, fontsize=12)
# plt.tick_params(axis='both', which='both')  # Adjust tick label font size
# # Add legends
# plt.legend(bbox_to_anchor=(1, 0.3)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2
#
#
# # Create a second y-axis on the right side
# ax2 = plt.twinx()
# ax2.plot(x, Kp, label = "Kp", color='steelblue')
# ax2.plot(x, Ki, label = "Ki", color='orange')
# ax2.plot(x, Kd, label = "Kd", color='limegreen')
# ax2.plot(x, gz, label = r"$\alpha$", color='salmon')
# # ax2.legend(fontsize=13)
# ax2.tick_params(axis='both', which='both')  # Adjust tick label font size
#
#
# # ax2.set_ylabel('x-Position [meter]', color='b')
# # Customize the color of ax2
# ax2.spines['left'].set_color('b')  # Set the color of the right spine
# # ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
#
#
# plt.title('Best Particle and Cost Function')


##################################################
# plt.subplot(2, 2, 3)
#
#
# # fig, ax1 = plt.subplots(figsize=(5, 7))
#
# # Plot 'b' data on the right y-axis
# plt.plot(x, err, marker='', linestyle='-', color='b', label='Cost Function')
# plt.xlabel('Iteration') # naming the x axis
# plt.ylabel('Value', color='b') # naming the y axis
# plt.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# # ax1.spines['left'].set_color('b')  # Set the color of the right spine
# # ax1.set_xticklabels(x, fontsize=12)
# plt.tick_params(axis='both', which='both')  # Adjust tick label font size
# # Add legends
# # plt.legend(bbox_to_anchor=(1, 0.3)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2
#
# # Create a second y-axis on the right side
# ax2 = plt.twinx()
# ax2.plot(x, Kp, label = "Kp", color='steelblue')
# ax2.plot(x, Ki, label = "Ki", color='orange')
# ax2.plot(x, Kd, label = "Kd", color='limegreen')
# ax2.plot(x, gz, label = r"$\alpha$", color='salmon')
# # ax2.legend(fontsize=13)
# ax2.tick_params(axis='both', which='both')  # Adjust tick label font size
#
# # ax2.set_ylabel('x-Position [meter]', color='b')
# # Customize the color of ax2
# ax2.spines['left'].set_color('b')  # Set the color of the right spine
# # ax2.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis




##################################################




#directory_name = "PID_training/"
#directory_name = "PID_training_Feb_7_with 505 iter/"
#directory_name = "z_training_Feb_28_v2/"
directory_name = "z_training_Feb_28_v2/"
# save_particle(filename, kp, Ki, Kd, gz, vz, az, dist, result)

x, Kp_1, Ki_1, Kd_1, gz_1, vz, az, dist, err_0 = read_particle(directory_name+"0th_particle_PID_result.txt",2,'1st particle')
x, Kp_2, Ki_2, Kd_2, gz_2, vz, az, dist, err_1 = read_particle(directory_name+"1th_particle_PID_result.txt",3,'2nd particle')
x, Kp_3, Ki_3, Kd_3, gz_3, vz, az, dist, err_2 = read_particle(directory_name+"2th_particle_PID_result.txt",4,'3rd particle')
x, Kp_4, Ki_4, Kd_4, gz_4, vz, az, dist, err_3 = read_particle(directory_name+"3th_particle_PID_result.txt",5,'4th particle')
x, Kp_5, Ki_5, Kd_5, gz_5, vz, az, dist, err_4 = read_particle(directory_name+"4th_particle_PID_result.txt",6,'5th particle')





########################################


# Create a figure and axis
# fig, ax1 = plt.subplots(figsize=(5, 7))
plt.subplot(1, 2, 1)

# Plot 'b' data on the right y-axis
plt.plot(x, Kp_1, marker='', linestyle='-', color='steelblue', label='Kp')
plt.plot(x, Kp_2, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_3, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_4, marker='', linestyle='-', color='steelblue')
plt.plot(x, Kp_5, marker='', linestyle='-', color='steelblue')

# Plot 'b' data on the right y-axis
plt.plot(x, Ki_1, marker='', linestyle='-', color='orange', label='Ki')
plt.plot(x, Ki_2, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_3, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_4, marker='', linestyle='-', color='orange')
plt.plot(x, Ki_5, marker='', linestyle='-', color='orange')

# Plot 'b' data on the right y-axis
plt.plot(x, Kd_1, marker='', linestyle='-', color='limegreen', label='Kd')
plt.plot(x, Kd_2, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_3, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_4, marker='', linestyle='-', color='limegreen')
plt.plot(x, Kd_5, marker='', linestyle='-', color='limegreen')

# Plot 'b' data on the right y-axis
plt.plot(x, gz_1, marker='', linestyle='-', color='salmon', label=r"$\alpha$")
plt.plot(x, gz_2, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_3, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_4, marker='', linestyle='-', color='salmon')
plt.plot(x, gz_5, marker='', linestyle='-', color='salmon')


plt.xlabel('Iteration') # naming the x axis
# plt.ylabel('Value') # naming the y axis
# ax1.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# ax1.spines['left'].set_color('b')  # Set the color of the right spine
# ax1.set_xticklabels(x, fontsize=12)
plt.tick_params(axis='both', which='both')  # Adjust tick label font size
plt.legend()

# Add legends
plt.legend(bbox_to_anchor=(1, 0.5)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2
plt.tight_layout()  # Adjust subplot spacing
plt.title('All Particles Trajectories - Descending')




##################################################
# Create a figure and axis
# fig, ax1 = plt.subplots(figsize=(5, 7))


directory_name = "Sep_11_PSO_train/"
# save_particle(filename, kp, Ki, Kd, gz, vz, az, dist, result)

x_1, Kp_1, Ki_1, Kd_1, gz_1, vz, az, dist, err_0 = read_particle(directory_name+"0th_particle_PID_result.txt",2,'1st particle')
x_2, Kp_2, Ki_2, Kd_2, gz_2, vz, az, dist, err_1 = read_particle(directory_name+"1th_particle_PID_result.txt",3,'2nd particle')
x_3, Kp_3, Ki_3, Kd_3, gz_3, vz, az, dist, err_2 = read_particle(directory_name+"2th_particle_PID_result.txt",4,'3rd particle')
x_4, Kp_4, Ki_4, Kd_4, gz_4, vz, az, dist, err_3 = read_particle(directory_name+"3th_particle_PID_result.txt",5,'4th particle')
x_5, Kp_5, Ki_5, Kd_5, gz_5, vz, az, dist, err_4 = read_particle(directory_name+"4th_particle_PID_result.txt",6,'5th particle')


# print("len of x:", len(x))
# print("len of Kp_1:", len(Kp_1))
# print("len of Kp_2:", len(Kp_2))
# print("len of Kp_3:", len(Kp_3))
# print("len of Kp_4:", len(Kp_4))
# print("len of Kp_5:", len(Kp_5))

plt.subplot(1, 2, 2)

# Plot 'b' data on the right y-axis
plt.plot(x_1, Kp_1, marker='', linestyle='-', color='steelblue', label='Kp')
plt.plot(x_2, Kp_2, marker='', linestyle='-', color='steelblue')
plt.plot(x_3, Kp_3, marker='', linestyle='-', color='steelblue')
plt.plot(x_4, Kp_4, marker='', linestyle='-', color='steelblue')
plt.plot(x_5, Kp_5, marker='', linestyle='-', color='steelblue')

# Plot 'b' data on the right y-axis
plt.plot(x_1, Ki_1, marker='', linestyle='-', color='orange', label='Ki')
plt.plot(x_2, Ki_2, marker='', linestyle='-', color='orange')
plt.plot(x_3, Ki_3, marker='', linestyle='-', color='orange')
plt.plot(x_4, Ki_4, marker='', linestyle='-', color='orange')
plt.plot(x_5, Ki_5, marker='', linestyle='-', color='orange')

# Plot 'b' data on the right y-axis
plt.plot(x_1, Kd_1, marker='', linestyle='-', color='limegreen', label='Kd')
plt.plot(x_2, Kd_2, marker='', linestyle='-', color='limegreen')
plt.plot(x_3, Kd_3, marker='', linestyle='-', color='limegreen')
plt.plot(x_4, Kd_4, marker='', linestyle='-', color='limegreen')
plt.plot(x_5, Kd_5, marker='', linestyle='-', color='limegreen')

# Plot 'b' data on the right y-axis
plt.plot(x_1, gz_1, marker='', linestyle='-', color='fuchsia', label=r"$\beta$")
plt.plot(x_2, gz_2, marker='', linestyle='-', color='fuchsia')
plt.plot(x_3, gz_3, marker='', linestyle='-', color='fuchsia')
plt.plot(x_4, gz_4, marker='', linestyle='-', color='fuchsia')
plt.plot(x_5, gz_5, marker='', linestyle='-', color='fuchsia')


plt.xlabel('Iteration') # naming the x axis
# plt.ylabel('Value') # naming the y axis
# ax1.tick_params(axis='y', colors='b')  # Set the tick color for the right y-axis
# ax1.spines['left'].set_color('b')  # Set the color of the right spine
# ax1.set_xticklabels(x, fontsize=12)
plt.tick_params(axis='both', which='both')  # Adjust tick label font size
plt.legend(bbox_to_anchor=(1, 0.5)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2

# Add legends
# plt.legend(bbox_to_anchor=(1, 0.5)) # x right 0.95 left 0.5       # Y  high 0.3 lower 0.2
plt.tight_layout()  # Adjust subplot spacing
plt.title('All Particles Trajectories - Tracking')


################

plt.show()