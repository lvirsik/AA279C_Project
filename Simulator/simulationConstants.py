import numpy as np

FINAL_TIME = 90
TIMESTEP = 0.2
INITIAL_OEs = [73000000, 0.01, np.pi/2, np.pi/2, 0.1, 0.1]
INITIAL_Q = [1, 0, 0, 1] #will get normalized
INITIAL_w = np.array([0.01, 0.1, 0]) #in Body Frame