import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Simulator.util import *
from Simulator.dynamics import *

def plot_GG_coeff(satellite):
    

    range = np.linspace(-1, 1, 1000)
    
    # Create a meshgrid from Kt and Kr ranges
    Kr, Kt = np.meshgrid(range, range)

    # Calculate the corresponding values of Kn using the given equations (There are 3 so I can countourf several times)
    Kn = (1 - Kr) / Kt
    Kn2 = (1 - Kr) / Kt
    Kn3 = (1 - Kr) / Kt

    # My Constraints
    constraint1 = Kt > Kr
    constraint2 = Kr * Kt > 0
    constraint3 = 1 + (3 * Kt) + (Kr * Kt) > (4 * np.sqrt(Kr * Kt))
    
    # Plot the constraints
    plt.contour(Kr, Kt, constraint1, colors='black')
    plt.contour(Kr, Kt, constraint2, colors='black')
    plt.contour(Kr, Kt, constraint3, colors='black')

    # Color Blue zone, unstable yaw and roll
    Kn[constraint1 == False] = np.nan
    Kn[constraint3 == True] = np.nan
    plt.contourf(Kr, Kt, Kn, colors=['blue'])
    
    # Color Green zone, unstable yaw and roll and pitch
    Kn2[constraint1 == True] = np.nan
    Kn2[constraint3 == True] = np.nan
    plt.contourf(Kr, Kt, Kn2, colors=['green'])

    # Color Yellow zone, unstable pitch
    Kn3[constraint1 == True] = np.nan
    Kn3[constraint3 == False] = np.nan
    plt.contourf(Kr, Kt, Kn3, colors=['yellow'], alpha=1)

    """
    I = satellite.I
    Ix = I[0,0]
    Iy = I[1,1]
    Iz = I[2,2]

    Kn = (Iy - Ix) / Iz 
    Kt = (Iz - Ix) / Iy 
    Kr = (Iz - Iy) / Ix 

    print("Kn: " + str(Kn))
    print("Kt: " + str(Kt))
    print("Kr: " + str(Kr))
    
    plt.plot(Kt, Kr)
     """
    

    plt.xlabel('Kr')
    plt.ylabel('Kt')
    plt.title('Stability Analysis')
    plt.grid(True)
    plt.show()
    