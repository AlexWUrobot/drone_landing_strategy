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
def save_global(filename, kp, Ki, Kd, result):
    f = open(filename, "a")
    f.write(str(kp)+'   '+str(Ki)+'   '+str(Kd)+'   '+str(result)+"\n")
    f.close()

# def clean_ros():
#     files = glob.glob('/home/lifan/.ros/*')
#     for f in files:
#         os.remove(f)
#     print("=================== Clean ROS files =============== ")

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
X = np.vstack([x0, x1, x2])
V = np.random.randn(3, n_particles) * 0.1    # PID initial velocity


# Initialize data
pbest = X
#pbest_obj = f(X[0], X[1])                 # array size:  1*n_particles
pbest_obj = np.zeros(n_particles)
for i in range(n_particles):
    print(str(i)+"th particle PID:"+str(X[0][i])+"   "+ str(X[1][i])+"   "+str(X[2][i]))
    write_data('Kp.txt',X[0][i])
    write_data('Ki.txt',X[1][i])
    write_data('Kd.txt',X[2][i])
    os.system("python Aug_14_PID_tracking.py")
    time.sleep(3)
    f_result = read_data('f_result.txt')
    pbest_obj[i] = f_result

    filename = str(i) + "th_particle_PID_result.txt"
    save_global(filename, X[0][i], X[1][i], X[2][i], f_result) # save every particle
    #clean_ros()
    #os.system("rm -rf /home/lifan/.ros/* ")


# global saving
print("pbest_obj:" + str(pbest_obj))
print("Kp:"+ str(x0))
print("Ki:"+ str(x1))
print("Kd:"+ str(x2))
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
        print(str(i) + "th particle PID:" + str(X[0][i]) + "   " + str(X[1][i]) + "   " + str(X[2][i]))
        write_data('Kp.txt', X[0][i])
        write_data('Ki.txt', X[1][i])
        write_data('Kd.txt', X[2][i])
        os.system("python Aug_14_PID_tracking.py")
        time.sleep(3)
        f_result = read_data('f_result.txt')
        obj[i] = f_result
        filename = str(i)+"th_particle_PID_result.txt"
        save_global(filename, X[0][i], X[1][i], X[2][i], f_result)       # save every particle
        os.system("rm -rf /home/lifan/.ros/* ")

    pbest[:, (pbest_obj >= obj)] = X[:, (pbest_obj >= obj)]
    pbest_obj = np.array([pbest_obj, obj]).min(axis=0)
    gbest = pbest[:, pbest_obj.argmin()]
    gbest_obj = pbest_obj.min()

    save_global("best_particle_PID_result.txt", gbest[0], gbest[1], gbest[2], gbest_obj)  # save best particle


for i in range(100):
    update()
    print("=================== training iter :" + str(i) + "=============== ")

print("PSO found best solution at f({})={}".format(gbest, gbest_obj))

# raise SystemExit(0)



