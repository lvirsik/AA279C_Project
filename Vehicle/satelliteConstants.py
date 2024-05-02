import numpy as np

MASS = 689.67 #kg
# I_COM_BF = np.array([[2185.6, 1.32, -0.08],
#                      [1.32, 4164.73, 2.17],
#                       [-0.08, 2.17, 2183.57]])

I_COM_BF = np.array([[2185.6, 0, 0],
                     [0, 4164.73, 0],
                      [0, 0, 1000.57]])


# Rotor
ROTOR_MASS = 20 #kg
ROTOR_SPEED = 10 #rad/s
ROTOR_DIRECTION = np.array([0.707,0.707,0])
I_ROTOR = 1000
