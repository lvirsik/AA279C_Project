import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Simulator.util import *
from Simulator.dynamics import *

def plot_euler(trajectory):
    rotational_history = trajectory[:, 6:]

    # Create a figure for the rotational dynamics plots
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))

    # Define the names for the rotational dynamics plots
    rotational_dynamics_plot_names = [
        "Pitch (degrees)",
        "Yaw (degrees)",
        "Roll (degrees)",
        "wx",
        "wy",
        "wz",
    ]

    # Define the conversion factor from radians to degrees
    RAD2DEG = 180 / np.pi

    analytical_solution = np.empty((1, 3))
    for i in range(len(rotational_history)):
        R = q2R(rotational_history[i,0:4])
        EAs = R2EAs(R)
        rotational_history[i, 3:6] = rotational_history[i, 4:7]
        rotational_history[i, 0:3] = EAs
        
        analytical_solution = np.vstack((analytical_solution, axially_symmetric_analytical_solution(i*TIMESTEP, I_COM_BF, INITIAL_w)))[1:len(rotational_history)]
        
        

    # Plot the rotational dynamics
    for i in range(6):
        row = i // 3
        col = i % 3
        if i < 3:
            axs[row, col].plot(rotational_history[:, i] * RAD2DEG)
        else:
            axs[row, col].plot(rotational_history[:, i])
            #axs[row, col].plot(analytical_solution[:, i-3])
        axs[row, col].set_title(rotational_dynamics_plot_names[i])
    
    # Adjust the layout
    plt.tight_layout()
    plt.show()