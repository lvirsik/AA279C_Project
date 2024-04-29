import numpy as np

MU_JUPITER = 1.266865349 * 10**17
R_JUPITER = 71492000


FINAL_TIME = 60
TIMESTEP = 0.1
INITIAL_OEs = [1432000000, 0.9, np.pi/2, np.pi/2, 0.1, 0.1]
INITIAL_Q = [0, 0, 0, 1] #will get normalized
INITIAL_w = np.array([1, 0.000001, 0.00001]) #in Body Frame