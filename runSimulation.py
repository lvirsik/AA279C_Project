import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Graphics.visualizeOrbits import *
from Graphics.visualizeEllipsoids import *


# Setup Simulation
initial_cartesian_state = OE_2_ECI(INITIAL_OEs)
initial_rotational_state = np.array([1, 1, 1, 0, 0, 0])
initial_state = np.concatenate((initial_cartesian_state, initial_rotational_state))
sim = Simulation(FINAL_TIME, TIMESTEP, initial_state)

# Run Simulation
trajectory = sim.propogate()
# plot_orbit(trajectory)
print("1")
plot_w_Energy_Momentum(initial_state)
print("2")
plot_w_Momentum(initial_state)