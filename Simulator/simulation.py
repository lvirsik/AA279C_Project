import numpy as np
import scipy.integrate
import copy
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Simulator.stateestimation import *
from Vehicle.satellite import Satellite
from Vehicle.kalmanFilter import *
import tqdm

class Simulation:
    """ Class Representing the Simulation and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state):
        
        # Progress bar 
        self.pbar = tqdm.tqdm(total=timefinal, desc="Simulation Progress")
        
        # Create Rocket Object
        self.satellite = Satellite()
        
        # States and Histories
        self.state = starting_state
        self.state_previous = copy.copy(self.state)
        self.statedot_previous = np.zeros(len(starting_state))
        self.state_history = np.empty((0,len(starting_state)))
        
        # Simulation Variables
        self.ts = simulation_timestep
        self.tf = timefinal
        self.previous_time = 0
        self.current_step = 0
        
        self.L_inertial = calculate_L_Inertial(self.satellite, self.state)
        self.w_inertial_history = np.array([calculate_w_inertial(self.satellite, self.state)[0:3]])
        self.R_history = np.array([q2R(self.state[6:10])])
        self.R_prin_history = np.expand_dims(np.dot(self.R_history[0], self.satellite.R), axis=0)
        self.RTN_history = np.expand_dims(calculate_RTN(self.state), axis=0)
        
        # Sensors:
        self.sensed_state_history = np.empty((0,14))
        self.kalman_state_history = np.empty((0,len(starting_state)))
        self.kalman_P = np.eye(13)
        
        self.statedot = (orbital_dynamics(self.state[0:6]), rotational_dynamics(self.state, self.satellite, 0, self.ts))

    def propogate(self):
        """ Simple propogator

        Inputs:
            state_0 = initial state
            tf = end time of simulation
            ts = timestep

        Returns:
            solution = position data at each timestamp
            
        """
        # Pull Time Information
        tf = self.tf
        ts = self.ts

        state = self.state

        # Create t vector from 0 to tf with timestep ts
        t_span = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
        t = (0,self.tf)
       
        solution = scipy.integrate.solve_ivp(self.wrapper_state_to_stateDot, t, state, 
                                             args=(self.satellite, t_span), t_eval=t_span, max_step=ts/5)
        state_history = solution['y'].T
        self.pbar.close()
        return state_history
                            
    def wrapper_state_to_stateDot(self, t, state, satellite, t_vec):
        """ Wrapper for the dynamics, most of the work done in this step. It calls the controller and updates the state."""
        # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
        
        if (t == 0) or (t >= t_vec[self.current_step] and self.previous_time < t_vec[self.current_step]):
            self.pbar.update(t - self.pbar.n)
            self.pbar.refresh()
            
            # Ensure q is normalized
            state[6:10] = normalize_vector(state[6:10])

            self.state = state
            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
            
            # Run Checks on accuracy of situation and physical constraints
            self.checks()
            
            # create history arrays
            self.w_inertial_history = np.vstack((self.w_inertial_history, calculate_w_inertial(self.satellite, state)[0:3]))
            self.R_history = np.vstack((self.R_history, np.expand_dims(q2R(state[6:10]).T, axis=0)))
            self.R_prin_history = np.vstack((self.R_prin_history, np.expand_dims(np.dot(self.satellite.R, self.R_history[-1]), axis=0)))
            self.RTN_history = np.vstack((self.RTN_history, np.expand_dims(calculate_RTN(self.state).T, axis=0)))
            self.state_history = np.vstack((self.state_history, self.state))
            
            # Sense the state from sensors
            u = np.array([0,0,0])
            Y = get_Y(self.satellite, self.state)
            A = compute_A(self.state, u, self.satellite, t, self.ts)
            B = compute_B(self.state, u, self.satellite, t, self.ts)
            self.sensed_state_history = np.vstack([self.sensed_state_history, Y])
            kalman_state, self.kalman_P = kalman_filter(self.kalman_state_history[-1] if t > 0 else self.state, u, 
                                                       Y, A, B, self.ts, P=self.kalman_P)
            self.kalman_state_history = np.vstack([self.kalman_state_history, kalman_state]) 
        
            # Actual Statedot
            self.statedot = (orbital_dynamics(state[0:6]), rotational_dynamics(state, satellite, t, self.ts))

        self.statedot_previous = np.concatenate(self.statedot)
        return self.statedot_previous
    
    def checks(self):
        # Check that angular velocity is legal given satellite geometry
        I_values = np.diag(self.satellite.I_principle)

        L = np.linalg.norm(np.dot(self.satellite.R, L_BF(self.satellite, self.state)))
        T = np.linalg.norm(np.dot(self.satellite.R, T_BF(self.satellite, self.state)))
        if (L**2)/(2*T) < np.min(I_values):
            raise ValueError('PHYSICAL LAW VIOLATED: W VECTOR IS NOT LEGAL WITH THIS GEOMETRY - UNDER Imin - value = {} Imin {}'.format((L**2)/(2*T), np.min(I_values)))
        if (L**2)/(2*T) > np.max(I_values):
            raise ValueError('PHYSICAL LAW VIOLATED: W VECTOR IS NOT LEGAL WITH THIS GEOMETRY - OVER Imax - value = {} Imax {}'.format((L**2)/(2*T), np.min(I_values)))
        
        # TURNING OFF BECAUSE OF TORQUES
        # tol = 0.01
        # if (abs(self.L_inertial - calculate_L_Inertial(self.satellite, self.state)) >= tol).all():
        #     raise ValueError('PYSICAL LAW VIOLATED: L_INERTIAL CHANGING OVER TIME.')
        
        
        