import numpy as np

MU_JUPITER = 1.266865349 * 10**17
R_JUPITER = 71492000


FINAL_TIME = 60
TIMESTEP = 0.1
INITIAL_OEs = [1432000000, 0.9, np.pi/2, np.pi/2, 0.1, 0.1]
INITIAL_EAs = np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.5])