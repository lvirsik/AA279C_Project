import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *

def plot_euler(trajectory):
    rotational_history = trajectory[:, 6:]

    # Create a figure for the rotational dynamics plots
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))

    # Define the names for the rotational dynamics plots
    rotational_dynamics_plot_names = [
        "Pitch (degrees)",
        "Yaw (degrees)",
        "Roll (degrees)",
        "Pitch Rate (deg/s)",
        "Yaw Rate (deg/s)",
        "Roll Rate (deg/s)",
    ]

    # Define the conversion factor from radians to degrees
    RAD2DEG = 180 / np.pi

    # Plot the rotational dynamics
    for i in range(6):
        row = i // 3
        col = i % 3
        axs[row, col].plot(rotational_history[:, i] * RAD2DEG)
        axs[row, col].set_title(rotational_dynamics_plot_names[i])

    # Adjust the layout
    plt.tight_layout()
    plt.show()