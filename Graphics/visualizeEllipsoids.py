import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *

# Plot the possible trajectory the angular velocity can trace with the constraint of being compatible with the energy constant
def plot_w_Energy(trajectory):

# Plot the possible trajectory the angular velocity can trace with the constraint of being compatible with the magnitude of Angular Momentum
def plot_w_Momentum(trajectory):

# Plot the possible trajectory the angular velocity that is within both the energy ellipsoid and the moomentum ellipsoid, a bolhode.
def plot_polHode(trajectory):