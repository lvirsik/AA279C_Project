import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Simulator.util import *
from Simulator.dynamics import *

def plot_torque(trajectory, satellite):
    rotational_history = trajectory[:, 6:]

    torque = satellite.torque_history

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(torque[0], torque[1], torque[2])
    ax.set_xlabel('Torque X')
    ax.set_ylabel('Torque Y')
    ax.set_zlabel('Torque Z')
    
    ax.set_title("Torque vs Time")
    plt.show()