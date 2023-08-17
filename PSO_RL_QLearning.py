import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import gym

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

# Only PSO algorithm
# for i in range(100):
#     update()
#     best_fit_record.append(gbest_obj)
#     if count % 50 == 0:
#         print("=================== training iter :" + str(i) + "=============== ")
#     if gbest_obj < converge_threshold and first_time == False:
#         converge_iter = i
#         end_time = time.process_time()
#         first_time = True
#     count = count + 1


# Define state and action space sizes
state_space_size = 3  # c1, c2, w
action_space_size = 3  # New values for c1, c2, w

# Initialize RL environment
env = gym.make("CartPole-v1")  # Replace with your RL environment


class MyRLAgent:
    def __init__(self, state_space_size, action_space_size, learning_rate=0.1, discount_factor=0.9):
        self.state_space_size = state_space_size
        self.action_space_size = action_space_size
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor

        # Initialize Q-table
        self.q_table = np.zeros((state_space_size, action_space_size))
        # https://towardsdatascience.com/reinforcement-learning-explained-visually-part-4-q-learning-step-by-step-b65efb731d3e

    def choose_action(self, state):
        # Choose an action using an epsilon-greedy policy
        if np.random.uniform(0, 1) < 0.1:
            return np.random.randint(self.action_space_size)
        else:
            return np.argmax(self.q_table[state])

    def update(self, state, action, reward, next_state):
        # Update Q-table using Q-learning update rule
        old_q = self.q_table[state, action]
        next_max_q = np.max(self.q_table[next_state])
        new_q = old_q + self.learning_rate * (reward + self.discount_factor * next_max_q - old_q)
        self.q_table[state, action] = new_q

# Initialize RL Agent
rl_agent = MyRLAgent(state_space_size, action_space_size)  # Replace with your RL agent implementation


for i in range(100):
    # Extract current state (hyperparameters)
    current_state = [c1, c2, w]

    # RL Agent chooses action (new hyperparameters)
    chosen_action = rl_agent.choose_action(current_state)
    new_c1, new_c2, new_w = chosen_action

    # Update hyperparameters
    c1, c2, w = new_c1, new_c2, new_w

    # Run PSO iterations with new hyperparameters
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

    # Calculate reward based on the achieved objective value
    reward = -gbest_obj  # Negative because we want to maximize

    # Update RL Agent with the observed state, chosen action, reward
    rl_agent.update(current_state, chosen_action, reward)


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


