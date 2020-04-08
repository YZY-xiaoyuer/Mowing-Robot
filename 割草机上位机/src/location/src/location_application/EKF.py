#!/usr/bin/env python
# license removed for brevity

import numpy as np
import math
from threading import Lock

class EKF(object):
    def __init__(self, num_states, initial_state, initial_covar, iterate_function):
        """
        Initializes the unscented kalman filter
        :param num_states: int, the size of the state
        :param initial_state: initial values for the states, should be num_states x 1
        :param initial_covar: initial covariance matrix, should be num_states x num_states, typically large and diagonal
        :param iterate_function: function that predicts the next state
                    takes in a num_states x 1 state and a float timestep
                    returns a num_states x 1 state
        :param observation_function: function that observation the state
        """

        self.D = int(num_states)
        self.X = initial_state
        self.P = initial_covar
        self.iterate = iterate_function
        self.lock = Lock()

    def predict(self, step_time, u, Q, jF):
        # step_time : the time interval between the two iterations
        # U: the control data
        # Q: the process noise covariance per unit time, should be num_states x num_states
        self.lock.acquire()
        self.X = self.iterate(step_time, self.X, u)
        self.P = jF @ self.P @ jF.T + Q

        self.lock.release()

    def update(self, R, jH, Residual):
        # Z is observation data
        # R is the covariance of the observation's error

        self.lock.acquire()
        S = jH @ self.P @ jH.T + R
        K = self.P @ jH.T @ np.linalg.inv(S)
        x_posterior_now = self.X + K @ Residual
        p_posterior_now = (np.eye(len(self.X)) - K @ jH) @ self.P
        self.X = x_posterior_now
        self.P = p_posterior_now
        self.lock.release()

    def get_state(self, index=-1):
        """
        returns the current state (n_dim x 1), or a particular state variable (float)
        :param index: optional, if provided, the index of the returned variable
        :return:
        """
        if index >= 0:
            return self.X[index]
        else:
            return self.X

    def get_covar(self):
        """
        :return: current state covariance (n_dim x n_dim)
        """
        return self.P

    def set_state(self, value, index=-1):
        """
        Overrides the filter by setting one variable of the state or the whole state
        :param value: the value to put into the state (1 x 1 or n_dim x 1)
        :param index: the index at which to override the state (-1 for whole state)
        """
        with self.lock:
            if index != -1:
                self.X[index] = value
            else:
                self.X = value

    def reset(self, state, covar):
        """
        Restarts the UKF at the given state and covariance
        :param state: n_dim x 1
        :param covar: n_dim x n_dim
        """

        with self.lock:
            self.X = state
            self.P = covar


