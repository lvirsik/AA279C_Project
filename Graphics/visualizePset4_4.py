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

    torque_hist = satellite.torque_history
    
    torqueX, torqueY, torqueZ = extract_columns(torque_hist)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(torqueX, torqueY, torqueZ)
    ax.set_xlabel('Torque X')
    ax.set_ylabel('Torque Y')
    ax.set_zlabel('Torque Z')

    ax.set_xlim([np.min(torqueX), np.max(torqueX)])
    ax.set_ylim([np.min(torqueY), np.max(torqueY)])
    ax.set_zlim([np.min(torqueZ), np.max(torqueZ)])
    ax.set_box_aspect([1,1,1])
    
    ax.set_title("Torque vs Time")
    plt.show()

def extract_columns(matrix):
    col1 = []
    col2 = []
    col3 = []

    for i in range(len(matrix)):
        col1.append(matrix[i][0])
        col2.append(matrix[i][1])
        col3.append(matrix[i][2])

    return col1, col2, col3