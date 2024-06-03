import numpy as np

FINAL_TIME = 30
TIMESTEP = 0.2
INITIAL_OEs = [73000000, 0.01, np.pi/3, np.pi/3, 0.1, 0.1]
INITIAL_Q = [1, 0, 0, 0] #will get normalized
INITIAL_w = np.array([0.02, 0.05, 0.02]) #in Body Frame

IDEAL_STATE = np.array([0,0,0,0,0,0,0,0,0,1,0,0,0])

# Reference direction in interial frame
JUPITER2SUN = [63000000, 0, 0]
JUPITER2STAR = [0, 63000000, 0] # arbitrary star
JUPITER2STAR2 = [0, -63000000, 0] # arbitrary star
JUPITER2STAR3 = [63000000, 0, 0] # arbitrary star
JUPITER2STAR4 = [-63000000, 0, 505340340] # arbitrary star
