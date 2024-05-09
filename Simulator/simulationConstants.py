import numpy as np

FINAL_TIME = 60
TIMESTEP = 0.1
INITIAL_OEs = [1432000000, 0.9, np.pi/2, np.pi/2, 0.1, 0.1]
INITIAL_Q = [0.54741879, 0.44758537, 0.54741879, 0.44758537] #will get normalized  #PS5: Aligned with RTN frame
INITIAL_w = np.array([0, 0, 1]) #in Body Frame # mean motion n velicty on an axis is equiulibrium

#6.568265537489643e-06  

#R_RTN = calculate_RTN(OE_2_ECI(INITIAL_OEs))
    #Q = R2q(R_RTN)

    #a = INITIAL_OEs[0] 
    #mu = MU_JUPITER
    # n = np.sqrt(mu/a**3)