import os
import numpy as np
import time
# https://stackoverflow.com/questions/13034496/using-global-variables-between-files
# import glob
# import shutil
# https://stackoverflow.com/questions/185936/how-to-delete-the-contents-of-a-folder


def write_data(filename, value):
    f = open(filename, "w")
    f.write(str(value))
    f.close()
def read_data(filename):
    f = open(filename, 'r')
    cur_iter = f.read()
    f.close()
    return float(cur_iter)
def save_particle(filename, kp, Ki, Kd, gz, vz, az, dist, time, result):
    f = open(filename, "a")
    f.write(str(kp)+'   '+str(Ki)+'   '+str(Kd)+'   '+str(gz)+'   '+str(vz)+'   '+str(az)+'   '+str(dist)+'   '+str(time)+'   '+str(result)+"\n")
    f.close()

def save_global(filename, kp, Ki, Kd, gz, result):
    f = open(filename, "a")
    f.write(str(kp)+'   '+str(Ki)+'   '+str(Kd)+'   '+str(gz)+'   '+str(result)+"\n")
    f.close()

def save_beta(filename, kp, Ki, Kd, beta, dist, angular_vel_x, angular_vel_y, result):
    f = open(filename, "a")
    f.write(str(kp)+'   '+str(Ki)+'   '+str(Kd)+'   '+str(beta)+'   '+str(dist)+'   '+str(angular_vel_x)+'   '+str(angular_vel_y)+'   '+str(result)+"\n")
    f.close()


def clean_ros():
    files = glob.glob('/home/lifan/.ros/*')
    for f in files:
        os.remove(f)
    print("=================== Clean ROS files =============== ")

# PSO global parameters

# Hyper-parameter of the algorithm
c1 = c2 = 0.1
w = 0.8

# Create particles
n_particles = 5
np.random.seed(100)

# X = np.random.rand(2, n_particles) * 5
x0 = np.random.rand(1, n_particles) * 2   # KP
x1 = np.random.rand(1, n_particles) * 1   # KI
x2 = np.random.rand(1, n_particles) * 1   # KD
x3 = np.random.rand(1, n_particles) * 3   # beta  2

X = np.vstack([x0, x1, x2, x3])
V = np.random.randn(4, n_particles) * 0.1    # KDI


# Initialize data
pbest = X
#pbest_obj = f(X[0], X[1])                 # array size:  1*n_particles
pbest_obj = np.zeros(n_particles)
for i in range(n_particles):
    print(str(i)+"th particle PID:"+str(X[0][i])+"   "+ str(X[1][i])+"   "+str(X[2][i])+"    beta:"+str(X[3][i]))
    write_data('Kp.txt',X[0][i])
    write_data('Ki.txt',X[1][i])
    write_data('Kd.txt',X[2][i])
    write_data('beta.txt',X[3][i])
    #os.system("python Jan_31_PSO_PID_h_sim_move_land_height_Aruco_Barrier_GPS_initial_velocity.py")
    #os.system("python Feb_27_PSO_gZ_time_PID_h_sim_move_land_height_Aruco_Barrier_GPS_initial_velocity.py")
    #time.sleep(3)

    curr_iter = read_data('iter.txt')
    end_iter = curr_iter + 1
    count_repeat = 0
    while curr_iter < end_iter:
        os.system("python Sep_9_constboatspeed_ramp_for_PSO_train.py")
        time.sleep(3)
        curr_iter = read_data('iter.txt')
        print("curr_iter: " +str(curr_iter) + "iter repeat: "+ str(count_repeat))
        count_repeat = count_repeat + 1

    f_result = read_data('f_result.txt')
    # az_result = read_data('az_before_landing.txt')
    # vz_result = read_data('vz_before_landing.txt')
    # dist_result = read_data('dist_after_landing.txt')
    # time_result = read_data('time_for_landing.txt')
    dist = read_data('dist.txt')
    angular_vel_x = read_data('angular_vel_x.txt')
    angular_vel_y = read_data('angular_vel_y.txt')

    pbest_obj[i] = f_result

    filename = str(i) + "th_particle_PID_result.txt"
    # save_global(filename, X[0][i], X[1][i], X[2][i], X[3][i], f_result) # save every particle
    #save_particle(filename, X[0][i], X[1][i], X[2][i], X[3][i], vz_result, az_result, dist_result, time_result, f_result)  # save every particle
    save_beta(filename, X[0][i], X[1][i], X[2][i], X[3][i], dist, angular_vel_x, angular_vel_y, f_result)
    #clean_ros()
    os.system("rm -rf /home/lifan/.ros/* ")


# global saving
print("pbest_obj:" + str(pbest_obj))
print("Kp:"+ str(x0))
print("Ki:"+ str(x1))
print("Kd:"+ str(x2))
print("beta:"+ str(x3))
gbest = pbest[:, pbest_obj.argmin()]
gbest_obj = pbest_obj.min()

print("=================== Initialize data Completed =============== ")

print("gbest: "+str(gbest))
print("gbest_obj: "+str(gbest_obj))
# raise SystemExit(0)



def update():
    "Function to do one iteration of particle swarm optimization"
    global V, X, pbest, pbest_obj, gbest, gbest_obj
    # Update params
    r1, r2 = np.random.rand(2)
    V = w * V + c1 * r1 * (pbest - X) + c2 * r2 * (gbest.reshape(-1, 1) - X)
    X = X + V
    #obj = f(X[0], X[1])    # array size:  1*n_particles
    obj = np.zeros(n_particles)
    for i in range(n_particles):
        #print("particle:" + str(X[0][i]) + " : " + str(X[1][i]))
        print(str(i) + "th particle PID:" + str(X[0][i]) + "   " + str(X[1][i]) + "   " + str(X[2][i])+ "   beta:" + str(X[2][i]))
        write_data('Kp.txt', X[0][i])
        write_data('Ki.txt', X[1][i])
        write_data('Kd.txt', X[2][i])
        write_data('beta.txt', X[3][i])
        #os.system("python Jan_31_PSO_PID_h_sim_move_land_height_Aruco_Barrier_GPS_initial_velocity.py")
        #os.system("python Feb_27_PSO_gZ_time_PID_h_sim_move_land_height_Aruco_Barrier_GPS_initial_velocity.py")
        #time.sleep(3)

        curr_iter = read_data('iter.txt')
        end_iter = curr_iter + 1
        count_repeat = 0
        one_particle_repeat_time = 0

        while curr_iter < end_iter:
            os.system("python Sep_9_constboatspeed_ramp_for_PSO_train.py")
            time.sleep(3)
            curr_iter = read_data('iter.txt')
            print("curr_iter: " + str(curr_iter) + "iter repeat: " + str(count_repeat))
            count_repeat = count_repeat + 1

            if one_particle_repeat_time > 0:  # count one particle repeat time, use 20 second to restart environment
                for k in range(20):
                    print("get stuck in one particle, repeat "+ str(one_particle_repeat_time) + " times and wait for"+ str(k)+"/20 seconds")
                    print("please restart gazebo and environment")
                    time.sleep(1)
            one_particle_repeat_time = one_particle_repeat_time + 1
            print("leave from iter ")


        f_result = read_data('f_result.txt')
        # az_result = read_data('az_before_landing.txt')
        # vz_result = read_data('vz_before_landing.txt')
        # dist_result = read_data('dist_after_landing.txt')
        # time_result = read_data('time_for_landing.txt')
        dist = read_data('dist.txt')
        angular_vel_x = read_data('angular_vel_x.txt')
        angular_vel_y = read_data('angular_vel_y.txt')

        obj[i] = f_result
        filename = str(i)+"th_particle_PID_result.txt"
        # save_global(filename, X[0][i], X[1][i], X[2][i], f_result)       # save every particle
        # save_particle(filename, X[0][i], X[1][i], X[2][i], X[3][i], vz_result, az_result, dist_result, time_result, f_result)  # save every particle

        #save_beta(filename, kp, Ki, Kd, beta, dist, angular_vel_x, angular_vel_y, result)
        save_beta(filename, X[0][i], X[1][i], X[2][i], X[3][i], dist, angular_vel_x, angular_vel_y, f_result)
        os.system("rm -rf /home/lifan/.ros/* ")

    pbest[:, (pbest_obj >= obj)] = X[:, (pbest_obj >= obj)]
    pbest_obj = np.array([pbest_obj, obj]).min(axis=0)
    gbest = pbest[:, pbest_obj.argmin()]
    gbest_obj = pbest_obj.min()

    save_global("best_particle_PID_result.txt", gbest[0], gbest[1], gbest[2], gbest[3], gbest_obj)  # save best particle


for i in range(100):
    update()
    print("=================== training iter :" + str(i) + "=============== ")

print("PSO found best solution at f({})={}".format(gbest, gbest_obj))

# raise SystemExit(0)



