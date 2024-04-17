import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Graphics.visualizeOrbits import *
from Graphics.visualizeEllipsoids import *


# Setup Simulation
initial_cartesian_state = OE_2_ECI(INITIAL_OEs)
initial_rotational_state = np.array([0, 0, 0, 0.1, 0.1, 0.1])
initial_state = np.concatenate((initial_cartesian_state, initial_rotational_state))
sim = Simulation(FINAL_TIME, TIMESTEP, initial_state)

# Run Simulation
trajectory = sim.propogate()
# print(np.shape(trajectory))
# print(trajectory)
# plot_orbit(trajectory)
plot_w_Energy_Momentum(initial_state)
plot_polHode(initial_state, trajectory)

plt.show()