import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Simulator.enviornmentConstants import *

def plot_orbit(trajectory):
    # Plot trajectory in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Load Jupiter texture image
    jupiter_texture = imread("Graphics/jupiter.jpg") / 255.0
    jupiter_radius = R_JUPITER

    # Plot Jupiter as a textured sphere
    img = plt.imread("Graphics/jupiter.jpg")
    u = np.linspace(0, 2 * np.pi, img.shape[0])
    v = np.linspace(0, np.pi, img.shape[1])

    x = jupiter_radius * np.outer(np.sin(u), np.sin(v))
    y = jupiter_radius * np.outer(np.cos(u), np.sin(v))
    z = jupiter_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, rstride=4, cstride=4, facecolors=jupiter_texture, alpha=0.8)
    
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
    ax.set_xlabel('X (km)')
    ax.set_ylabel('Y (km)')
    ax.set_zlabel('Z (km)')
    ax.set_title('Orbital Trajectory')
    
    # Set equal axis lengths
    max_range = np.max(trajectory)*3
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    ax.set_box_aspect([1,1,1])
    plt.show(block=False)