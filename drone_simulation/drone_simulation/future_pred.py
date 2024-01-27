"""
provides functions needed to predict future state of the drone
"""
import math
import time
import numpy as np
import utm
from geographiclib import geodesic
from visualization_msgs.msg import Marker
from scipy.stats import multivariate_normal, norm

from drone_simulation.utils import cosine_between_vectors

'''
Ths code provides multiple models to predict the future state of the drone.
'''

def dynamics_n_obst_aware_predictor(current_state, delT, laststate, obst_loc):
    '''
    input: crrent state = [x, y, z, vx, vy, vz, t]
           delT = time step
           last state = [x, y, z, vx, vy, vz, t]
           obst_loc = [x, y, z, r]    
           
    output: new state = [x, y, z, vx, vy, vz, t]
    obstacle_effect = True/False (whether or not obstacle is affecting the state)
    '''
    #model constants:
    delt_high = 1.3 # maximum time jump allowed
    delt_low = 0.9 # minimum time jump allowed (continuity assumption)

    del_state = current_state[-1] - laststate[-1]
    xdot = current_state[3]  # (current_state[0] - laststate[0])/del_state
    ydot = current_state[4]  # (current_state[1] - laststate[1])/del_state
    # print("velocity ", vx, vy)
    xddot = 0.0
    yddot = 0.0
    
    obstacle_effect = False
    xddot, yddot = check_obstacle_effect(current_state[0], current_state[1], current_state[2], xdot, ydot, current_state[5], delT, obst_loc)

    if delt_high > abs(del_state) > delt_low:  # filtering out noisy heading data
        xddot += (current_state[3] - laststate[3]) / del_state
        yddot += (current_state[4] - laststate[4]) / del_state

    dx = xdot + xddot * delT  # + g*math.sin(pitch)*del_state
    dy = ydot + yddot * delT  # - g*math.sin(roll)*math.cos(pitch)*del_state
    newx = laststate[0] + dx * delT
    newy = laststate[1] + dy * delT
    newz = laststate[2] + current_state[5] * delT
    return newx, newy, newz, obstacle_effect


def multi_obst_aware_predictor(current_state, delT, obst_locs):
    # simulates effect of multiple obstacles on the future state of target
    # obst_locs has multiple obstacles in the form of [x, y, z]
    # first obstacle is the closest one and so on
    x, y, z, xdot, ydot, zdot, t = current_state
    x, y, zp, obstacle_effect = markov_state_prediction(
        x, y, z, xdot, ydot, zdot, delT, obst_locs[0]
    )
    obst_locs = obst_locs[1:]
    for obst in obst_locs:
        xddot, yddot = check_obstacle_effect(x, y, zp, xdot, ydot, zdot, delT, obst)
        if abs(xddot + yddot) > 0:
            x += xddot * delT * delT * 0.5
            y += yddot * delT * delT * 0.5
    return x, y, zp, obstacle_effect


def check_obstacle_effect(x, y, z, xdot, ydot, zdot, delT, obst):
    #obstacle avoidance model (charge based model)
    threshold = 2.5  # distance to obstacle for it to be considered
    heading_threshold = np.pi/4 # angle between heading and obstacle for it to be considered
    x_repel_const = 0.001
    y_repel_const = 0.001 # repulsion constants (how aggresively repel from obstacle)
    acc_scale = 0.02 # maximum acceleration allowed
    
    ox, oy, oz = x - obst[0], y - obst[1], z - obst[2]
    xp, yp, zp = x + xdot * delT, y + ydot * delT, z + zdot * delT
    xddot = 0.0
    yddot = 0.0
    dist_to_obst = math.sqrt((ox) ** 2 + (oy) ** 2 + (oz) ** 2)
    cosine = cosine_between_vectors([xp, yp, zp], [x, y, z], obst)
    if dist_to_obst < threshold or cosine < heading_threshold:
        if abs(ox) > 0 and abs(oy) > 0:
            repel_x = (ox) / abs(ox)
            repel_y = (oy) / abs(oy)
            xddot = x_repel_const * repel_x / abs(ox)
            yddot = y_repel_const * repel_y / abs(oy)
            if abs(xddot) >acc_scale:
                xddot =acc_scale * xddot / abs(xddot)
            if abs(yddot) >acc_scale:
                yddot =acc_scale * yddot / abs(yddot)

    return xddot, yddot


def markov_state_prediction(x, y, z, xdot, ydot, zdot, delT, obst):
    #no effect of laststate
    #next state only depends on current state (first order markov assumption)
    zp = z + zdot * delT
    xddot, yddot = check_obstacle_effect(x, y, z, xdot, ydot, zdot, delT, obst)
    obstacle_effect = False
    dx = xdot + xddot * delT
    dy = ydot + yddot * delT
    if abs(xddot + yddot) > 0:
        obstacle_effect = True
    x += dx * delT
    y += dy * delT
    return x, y, zp, obstacle_effect

def heading_based_predictor(x, y, z, heading, v_hor, vz):
    #1 second time horizon
    correction = 0.0
    delT = 1.0
    vx = v_hor*math.cos(heading - correction)
    vy = v_hor*math.sin(heading - correction)
    nx = x + vx*delT
    ny = y + vy*delT
    nz = z + vz*delT
    return nx, ny, nz


class EKF_predictor:
    '''
    Uses kalman filter to predict the future state of the drone. 
    does not consider obstacles in the current implementation.
    It is just Kalman filter implementation.
    After considering obstacle avoidance, it can be modified into EKF
    '''
    def __init__(self, x, P, Q, R, dt):
        self.x = x  # state vector with shape (6, 1) x, y, z, vx, vy, vz
        self.P = P  # covariance matrix with shape (6, 6)
        self.Q = Q  # process noise covariance matrix with shape (6, 6)
        self.R = R  # measurement noise covariance matrix with shape (3, 3)
        self.dt = dt  # time step
        # A is the state transition matrix with shape (6, 6)
        # effect of obstacle on the state transition matrix is simulated using
        # change in velocities or accelerations

        self.A = np.array(
            [
                [1, 0, 0, self.dt, 0, 0],
                [0, 1, 0, 0, self.dt, 0],
                [0, 0, 1, 0, 0, self.dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )

        #BU part can be used to simulate effect of obstacles on the state transition matrix
        #consider obstacles as an input to the system

        # H is the measurement matrix
        self.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]])

        self.I = np.eye(6)

    def predict(self):
        self.x = np.matmul(self.A, self.x)
        self.P = np.matmul(np.matmul(self.A, self.P), self.A.T) + self.Q
        return self.x, self.P

    def update(self, z):
        # z is the measurement vector with shape (3, 1) x, y, z
        y = z - np.matmul(self.H, self.x)  # innovation
        S = np.matmul(np.matmul(self.H, self.P), self.H.T) + self.R
        K = np.matmul(np.matmul(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.matmul(K, y)
        self.P = np.matmul((self.I - np.matmul(K, self.H)), self.P)
        return self.x, self.P

    def get_state(self):
        return self.x, self.P

    def set_state(self, x, P):
        self.x = x
        self.P = P
        return self.x, self.P