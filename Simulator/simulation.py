import numpy as np
import scipy
import copy
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Vehicle.satellite import Satellite

class Simulation:
    """ Class Representing the Simulation and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state):
        
        # Create Rocket Object
        self.satellite = Satellite()
        
        # States and Histories
        self.state = starting_state
        self.state_previous = copy.copy(self.state)
        self.statedot_previous = np.zeros(len(starting_state))
        self.error_history = np.empty((0,len(starting_state)))
        
        # Simulation Variables
        self.ts = simulation_timestep
        self.tf = timefinal
        self.previous_time = 0
        self.current_step = 0

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
        return state_history
                            
    def wrapper_state_to_stateDot(self, t, state, satellite, t_vec):
        """ Wrapper for the dynamics, most of the work done in this step. It calls the controller and updates the state."""
        # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
        if (t == 0) or (t >= t_vec[self.current_step] and self.previous_time < t_vec[self.current_step]):
            
            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
        
        self.statedot_previous = np.concatenate((orbital_dynamics(state[0:6]), rotational_dynamics()))
        return self.statedot_previous