# importing the required module
import matplotlib.pyplot as plt
Kp = []
Ki = []
Kd = []
err = []
with open("0_5m_per_second_boat_5m_drone/best_particle_PID_result.txt", 'r') as file_data:
    count = 0
    for line in file_data:
        data = line.split()
        print(data)
        Kp.append(float(data[0]))
        Ki.append(float(data[1]))
        Kd.append(float(data[2]))
        err.append(float(data[3]))
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
#plt.plot(x, y)
plt.figure(0)
plt.plot(x, Kp, label = "Kp")
plt.plot(x, Ki, label = "Ki")
plt.plot(x, Kd, label = "Kd")
plt.legend()
plt.xlabel('x - axis - iteration') # naming the x axis
plt.ylabel('y - axis - value')  # naming the y axis
#plt.ylim([0, 5])
# giving a title to my graph
plt.title('best particle PID')
# function to show the plot

# plt.figure(1)
# plt.plot(x, err, label = "err")
# plt.title('cost function')
# plt.xlabel('x - axis - iteration') # naming the x axis
# plt.ylabel('average distance between UAV and USV')  # naming the y axis

###############


def read_particle(filename, plot_num, plot_name):
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



directory_name = "0_5m_per_second_boat_5m_drone/"
#directory_name = "PID_training_Feb_7_with 505 iter/"
x, Kp, Ki, Kd, err_0 = read_particle(directory_name+"0th_particle_PID_result.txt",2,'1st particle')
x, Kp, Ki, Kd, err_1 = read_particle(directory_name+"1th_particle_PID_result.txt",3,'2nd particle')
x, Kp, Ki, Kd, err_2 = read_particle(directory_name+"2th_particle_PID_result.txt",4,'3rd particle')
x, Kp, Ki, Kd, err_3 = read_particle(directory_name+"3th_particle_PID_result.txt",5,'4th particle')
x, Kp, Ki, Kd, err_4 = read_particle(directory_name+"4th_particle_PID_result.txt",6,'5th particle')


x, Kp, Ki, Kd, err_best = read_particle(directory_name+"best_particle_PID_result.txt",7,'Best particle')

plt.figure(8)
# plt.plot(x[0:99], err_0[0:99], label = "0th_particle")
# plt.plot(x[0:99], err_1[0:99], label = "1th_particle")
# plt.plot(x[0:99], err_2[0:99], label = "2th_particle")
# plt.plot(x[0:99], err_3[0:99], label = "3th_particle")
# plt.plot(x[0:99], err_4[0:99], label = "4th_particle")
plt.plot(x, err_best, label = "Best particle")
plt.title("Error between the drone and the landing spot")

plt.legend()
# naming the x axis
plt.xlabel('x - axis - iteration')
# naming the y axis
# plt.ylabel('y - axis - error')
plt.ylabel('average distance between UAV and USV')  # naming the y axis
plt.title('Cost function')





plt.show()