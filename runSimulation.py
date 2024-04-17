import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Graphics.visualizeOrbits import *
from Graphics.visualizeRotations import *
from Graphics.visualizeEllipsoids import *

# Setup Simulation (Orbit)
initial_cartesian_state = OE_2_ECI(INITIAL_OEs)
initial_rotational_state_orbit = np.array([0, 0, 0, 0, 0, 0])
initial_state_orbit = np.concatenate((initial_cartesian_state, initial_rotational_state_orbit))
sim_orbit = Simulation(FINAL_TIME, TIMESTEP, initial_state_orbit)

# Run Simulation (Orbit)
trajectory = sim_orbit.propogate()
plot_orbit(trajectory)

# Setup Simulation (Rotations)
initial_rotational_state_rotations = np.array([0, 0, 0, 0.5, 1, 0.7])
initial_state_rotations = np.concatenate((initial_cartesian_state, initial_rotational_state_rotations))
sim_rotations = Simulation(60, 0.1, initial_state_rotations)

# Run Simulation (Rotations)
trajectory = sim_rotations.propogate()
plot_euler(trajectory)

plot_w_Energy_Momentum(initial_state_rotations)
plot_polHode(initial_state_rotations, trajectory)
plt.show()