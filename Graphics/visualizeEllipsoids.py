import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Vehicle.satelliteConstants import *

def get_w_from_EulerAngle(trajectory):
     # 1 2 3 = phi theta psi = yaw pitch roll
    euler_angles = trajectory [6:9]
    euler_angle_rates = trajectory[9:12]

    phi = euler_angles[0]
    theta = euler_angles[1]
    psi = euler_angles[0]
    
    phiDot = euler_angle_rates[0]
    thetaDot = euler_angle_rates[1]
    psiDot = euler_angle_rates[2]

    # Angular velocity from euler angles and euler angles rates
    w = np.array([[phiDot * np.sin(theta) * np.sin(phiDot) + thetaDot * np.cos(psi)],
                  [phiDot * np.sin(theta) * np.cos(phiDot) + thetaDot * np.sin(psi)],
                  [phiDot * np.cos(theta) + psiDot]])
    return w

# Plot the possible trajectory the angular velocity can trace with the constraint of being compatible with the energy content and magnitude of the anguler momentum
def plot_w_Energy_Momentum(init_c):
    w = init_c[6:9]

    I_x = I_COM_BF[0,0]
    I_y = I_COM_BF[1,1]
    I_z = I_COM_BF[2,2]

    # Get rid of 0.5 on right side and have 2 on left side 2T = 1/2 * I * w^2 
    KE = np.dot(w, np.dot(I_COM_BF, w))
    print(KE)

    # Ellipsoid axis
    a = pow((KE / I_x), 0.5)
    b = pow((KE / I_y), 0.5)
    c = pow((KE / I_z), 0.5)
    print(a)

    # Plotting ellipsoid using spherical coordinates
    ellipsoid_phi = np.linspace(0,2*np.pi, 256).reshape(256, 1)
    ellipsoid_theta = np.linspace(0, np.pi, 256).reshape(-1, 256)
    x = a*np.sin(ellipsoid_theta)*np.cos(ellipsoid_phi)
    y = b*np.sin(ellipsoid_theta)*np.sin(ellipsoid_phi)
    z = c*np.cos(ellipsoid_theta)

    # Plot trajectory in 3D
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    print("before1")
    ax.plot_surface(x, y, z)
    print("before2")
    plt.show()
    print("after")

# Plot the possible trajectory the angular velocity can trace with the constraint of being compatible with the magnitude of Angular Momentum
def plot_w_Momentum(init_c):
    w = init_c[6:9]

    I_x = I_COM_BF[0,0]
    I_y = I_COM_BF[1,1]
    I_z = I_COM_BF[2,2]

    # Get rid of 0.5 on right side and have 2 on left side 2T = 1/2 * I * w^2 
    L = np.dot(I_COM_BF, w)
    # L_sqr = pow(L, 2)
    print(L)

    # Ellipsoid axis
    a = L / I_x
    b = L / I_y
    c = L / I_z
    print(a)

    # Plotting ellipsoid using spherical coordinates
    ellipsoid_phi = np.linspace(0,2*np.pi, 256).reshape(256, 1)
    ellipsoid_theta = np.linspace(0, np.pi, 256).reshape(-1, 256)
    x = a*np.sin(ellipsoid_theta)*np.cos(ellipsoid_phi)
    y = b*np.sin(ellipsoid_theta)*np.sin(ellipsoid_phi)
    z = c*np.cos(ellipsoid_theta)

    # Plot trajectory in 3D
    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z)
    plt.show()

# Plot the possible trajectory the angular velocity that is within both the energy ellipsoid and the momentum ellipsoid, a bolhode.
#def plot_polHode(trajectory):