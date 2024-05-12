import numpy as np
# Jupiter
MU_JUPITER = 1.266865349 * 10**17
R_JUPITER = 71492000
ROT_PERIOD_JUPITER = 9.9258 * 3600 #s
STARTING_THETA_JUPITER = 0
SUN_POSITION_JCI = np.array([1, 0, 0])

# Magnetic Field
B0_JUPITER = 417.0 * 10**(-6) # Teslas
JUPITER_DIPOLE_ANGLE_OFFSET = np.deg2rad(10) # offset in degrees to radiatns
mu0 = 4 * np.pi * 10**(-7)

# Solar Radiation
SOLAR_CONSTANT = 0.34 * 10**(-6) #N/m^2