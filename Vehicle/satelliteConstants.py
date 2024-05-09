import numpy as np

MASS = 689.67 #kg
# I_COM_BF = np.array([[2185.6, 1.32, -0.08],
#                      [1.32, 4164.73, 2.17],
#                       [-0.08, 2.17, 2183.57]])

I_COM_BF = np.array([[3520, 0, 0],
                     [0, 4000, 0],
                     [0, 0, 3120]])


# Rotor
ROTOR_MASS = 20 #kg
ROTOR_SPEED = 10 #rad/s
ROTOR_DIRECTION = np.array([0,1,0])
I_ROTOR = 100

# Magnetic Moment
NUM_COILS = 2
CURRENT = 0.25 #A
SURF_IN_COIL = 0.5 #m^2

 