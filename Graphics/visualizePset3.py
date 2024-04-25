
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

def plotVectorsOverTime(orbital_frame_vectors, body_axes_vectors, principal_axes_vectors, time):

    # Create a figure and axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot initial vectors
    quiver_orbital_frame = ax.quiver(0, 0, 0, orbital_frame_vectors[0, 0], orbital_frame_vectors[1, 0], orbital_frame_vectors[2, 0], color='r', label='Orbital Frame')
    quiver_body_axes = ax.quiver(0, 0, 0, body_axes_vectors[0, 0], body_axes_vectors[1, 0], body_axes_vectors[2, 0], color='g', label='Body Axes')
    quiver_principal_axes = ax.quiver(0, 0, 0, principal_axes_vectors[0, 0], principal_axes_vectors[1, 0], principal_axes_vectors[2, 0], color='b', label='Principal Axes')

    ax.set_xlim([-1, 1])  # Adjust limits for better visualization
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Vectors in 3D Space at Time: {:.2f}'.format(time[0]))
    ax.legend()

    # Define the slider
    ax_time = plt.axes([0.1, 0.01, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_time, 'Time', 0, len(time) - 1, valinit=0, valstep=1)

    # Update function for the slider
    def update(val):
        index = int(slider.val)
        # Update vectors
        quiver_orbital_frame.set_offsets(np.array([0, 0]))
        quiver_orbital_frame.set_segments([np.array([[0, 0, 0], orbital_frame_vectors[:, index]])])
        
        quiver_body_axes.set_offsets(np.array([0, 0]))
        quiver_body_axes.set_segments([np.array([[0, 0, 0], body_axes_vectors[:, index]])])
        
        quiver_principal_axes.set_offsets(np.array([0, 0]))
        quiver_principal_axes.set_segments([np.array([[0, 0, 0], principal_axes_vectors[:, index]])])
        
        ax.set_title('Vectors in 3D Space at Time: {:.2f}'.format(time[index]))
        fig.canvas.draw_idle()

    # Connect the slider to the update function
    slider.on_changed(update)

    plt.show()

#THIS FUNCTION GENERATES TEST DATA FOR THE FUNCTION ABOVE
def generateTestVectorsOverTime(time):

    # Example initial vectors
    orbital_frame_start = np.array([1, 0, 0])  
    body_axes_start = np.array([0, 1, 0])
    principal_axes_start = np.array([0, 0, 1])

    # Reshape initial vectors to match the time array shape
    orbital_frame_start = orbital_frame_start[:, np.newaxis]
    body_axes_start = body_axes_start[:, np.newaxis]
    principal_axes_start = principal_axes_start[:, np.newaxis]

    # Example vectors changing over time using rotation matrices
    def rotate_vector(vector, angle, axis):
        """
        Rotate a vector around a given axis by a given angle.

        Parameters:
        vector: numpy array, the vector to be rotated
        angle: float, the angle of rotation in radians
        axis: numpy array, the axis of rotation (must be a unit vector)
        """
        # Calculate the rotation matrix
        axis = axis / np.linalg.norm(axis)
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        rot_matrix = cos_theta * np.eye(3) + (1 - cos_theta) * np.outer(axis, axis) + sin_theta * np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        # Apply the rotation
        rotated_vector = np.dot(rot_matrix, vector)
        return rotated_vector

    orbital_frame_vectors = np.array([rotate_vector(orbital_frame_start, t, np.array([0, 0, 1])) for t in time]).squeeze().T
    body_axes_vectors = np.array([rotate_vector(body_axes_start, t, np.array([0, 0, 1])) for t in time]).squeeze().T
    principal_axes_vectors = np.array([rotate_vector(principal_axes_start, t, np.array([0, 0, 1])) for t in time]).squeeze().T

    return orbital_frame_vectors, body_axes_vectors, principal_axes_vectors

'''#TEST CODE FOR FUNCTION ABOVE
# Create time array
time = np.linspace(0, 10, 100)

orbital_frame_vectors, body_axes_vectors, principal_axes_vectors = generateTestVectorsOverTime(time)
plotVectorsOverTime(orbital_frame_vectors, body_axes_vectors, principal_axes_vectors, time)
'''

def plotStaticAndDynamicVector(static_vector, dynamic_vector):
    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the constant vector
    ax.quiver(0, 0, 0, static_vector[0], static_vector[1], static_vector[2], color='r', label='Constant Vector', arrow_length_ratio=0.03)

    # Plot the changing vector over time
    ax.plot(dynamic_vector[:,0], dynamic_vector[:,1], dynamic_vector[:,2], color='b', label='Changing Vector')

    # Calculate the limits for each axis separately
    static_limits = [(np.min(static_vector[i]), np.max(static_vector[i])) for i in range(3)]
    dynamic_limits = [(np.min(dynamic_vector[:,i]), np.max(dynamic_vector[:,i])) for i in range(3)]
    
    # Set the limits based on the maximum and minimum values of each axis
    ax.set_xlim(min(static_limits[0][0], dynamic_limits[0][0]), max(static_limits[0][1], dynamic_limits[0][1]))
    ax.set_ylim(min(static_limits[1][0], dynamic_limits[1][0]), max(static_limits[1][1], dynamic_limits[1][1]))
    ax.set_zlim(min(static_limits[2][0], dynamic_limits[2][0]), max(static_limits[2][1], dynamic_limits[2][1]))
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.legend()
    
    plt.title('Comparison of Changing and Constant Vectors')
    plt.show()

# TEST CODE FOR FUNCTION ABOVE
# time = np.linspace(0, 10*np.pi, 1000)  # Increase the time range
# static_vector = np.array([0, 0, 40])
# dynamic_vector = np.column_stack((np.sin(time), np.cos(time), time))

# plotStaticAndDynamicVector(static_vector, dynamic_vector, time)