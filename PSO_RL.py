import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import qlearning  # Replace with your RL library and algorithm of choice

def f(x,y):
    "Objective function"
    return (x-3.14)**2 + (y-2.72)**2 + np.sin(3*x+1.41) + np.sin(4*y-1.73)

# Hyper-parameter of the algorithm
c1 = c2 = 0.1
w = 0.8

# Create particles
n_particles = 20
np.random.seed(100)
X = np.random.rand(2, n_particles) * 1000
V = np.random.randn(2, n_particles) * 0.1

# Initialize data
pbest = X
pbest_obj = f(X[0], X[1])
gbest = pbest[:, pbest_obj.argmin()]
gbest_obj = pbest_obj.min()

def update():
    "Function to do one iteration of particle swarm optimization"
    global V, X, pbest, pbest_obj, gbest, gbest_obj
    # Update params
    r1, r2 = np.random.rand(2)
    V = w * V + c1*r1*(pbest - X) + c2*r2*(gbest.reshape(-1,1)-X)
    X = X + V
    obj = f(X[0], X[1])
    pbest[:, (pbest_obj >= obj)] = X[:, (pbest_obj >= obj)]
    pbest_obj = np.array([pbest_obj, obj]).min(axis=0)
    gbest = pbest[:, pbest_obj.argmin()]
    gbest_obj = pbest_obj.min()



start_time = time.process_time()
best_fit_record = []
count = 0
converge_threshold = -1.79
first_time = False
for i in range(100):
    update()
    best_fit_record.append(gbest_obj)
    if count % 50 == 0:
        print("=================== training iter :" + str(i) + "=============== ")
    if gbest_obj < converge_threshold and first_time == False:
        converge_iter = i
        end_time = time.process_time()
        first_time = True
    count = count + 1


print("PSO found best solution at f({})={}".format(gbest, gbest_obj))
print("Converge at the iter {}".format(converge_iter))
elapsed_time = end_time - start_time
print("Elapsed process time:", elapsed_time, "seconds")

plt.figure(figsize=(8, 6))
plt.plot(best_fit_record, label='x = iteration ', color='blue')
plt.title('Plot of cost function')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.grid(True)
plt.show()





