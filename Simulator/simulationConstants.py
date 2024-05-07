import numpy as np

FINAL_TIME = 60
TIMESTEP = 0.1
INITIAL_OEs = [1432000000, 0.9, np.pi/2, np.pi/2, 0.1, 0.1]
INITIAL_Q = [1, 0, 0, 1] #will get normalized
INITIAL_w = np.array([0, 0.00001, 0]) #in Body Frame