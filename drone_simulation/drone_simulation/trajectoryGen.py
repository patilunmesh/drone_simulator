from __future__ import division, print_function

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import drone_simulation.quadrocoptertrajectory as quadtraj


def trajectoryGenie(pos0, vel0, acc0, posf, velf, accf, Tf, numPlotPoints):
    # Define the input limits:
    fmin = 5  # [m/s**2]
    fmax = 25  # [m/s**2]
    wmax = 20  # [rad/s]
    minTimeSec = 0.02  # [s]

    # Define how gravity lies:
    gravity = [0, 0, -9.81]
    traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
    traj.set_goal_position(posf)
    traj.set_goal_velocity(velf)
    traj.set_goal_acceleration(accf)
    traj.generate(Tf)

    # Test input feasibility
    inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)

    # Test whether we fly into the floor
    floorPoint = [0, 0, 0]  # a point on the floor
    floorNormal = [0, 0, 1]  # we want to be in this direction of the point (upwards)
    positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)

    for i in range(3):
        print("Axis #", i)
        print(
            "\talpha = ",
            traj.get_param_alpha(i),
            "\tbeta = ",
            traj.get_param_beta(i),
            "\tgamma = ",
            traj.get_param_gamma(i),
        )
    print("Total cost = ", traj.get_cost())
    print(
        "Input feasibility result: ",
        quadtraj.InputFeasibilityResult.to_string(inputsFeasible),
        "(",
        inputsFeasible,
        ")",
    )
    print(
        "Position feasibility result: ",
        quadtraj.StateFeasibilityResult.to_string(positionFeasible),
        "(",
        positionFeasible,
        ")",
    )
    time = np.linspace(0, Tf, numPlotPoints)
    position_x = np.zeros([numPlotPoints])
    position_y = np.zeros([numPlotPoints])
    position_z = np.zeros([numPlotPoints])

    for i in range(numPlotPoints):
        t = time[i]
        positio = traj.get_position(t)
        position_x[i] += positio[0]
        position_y[i] += positio[1]
        position_z[i] += positio[2]
    return traj, position_x, position_y, position_z


def trajectoryGenieAccel(pos0, vel0, acc0, posf, velf, accf, Tf, numPlotPoints):
    """
    Identical function to above, but also returns planned acceleration 
    """
    # Define the input limits:
    fmin = 5  # [m/s**2]
    fmax = 25  # [m/s**2]
    wmax = 20  # [rad/s]
    minTimeSec = 0.02  # [s]

    # Define how gravity lies:
    gravity = [0, 0, -9.81]
    traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
    traj.set_goal_position(posf)
    traj.set_goal_velocity(velf)
    traj.set_goal_acceleration(accf)
    traj.generate(Tf)

    # Test input feasibility
    inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)

    # Test whether we fly into the floor
    floorPoint = [0, 0, 0]  # a point on the floor
    floorNormal = [0, 0, 1]  # we want to be in this direction of the point (upwards)
    positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)

    for i in range(3):
        print("Axis #", i)
        print(
            "\talpha = ",
            traj.get_param_alpha(i),
            "\tbeta = ",
            traj.get_param_beta(i),
            "\tgamma = ",
            traj.get_param_gamma(i),
        )
    print("Total cost = ", traj.get_cost())
    print(
        "Input feasibility result: ",
        quadtraj.InputFeasibilityResult.to_string(inputsFeasible),
        "(",
        inputsFeasible,
        ")",
    )
    print(
        "Position feasibility result: ",
        quadtraj.StateFeasibilityResult.to_string(positionFeasible),
        "(",
        positionFeasible,
        ")",
    )
    time = np.linspace(0, Tf, numPlotPoints)
    position_x = np.zeros([numPlotPoints])
    position_y = np.zeros([numPlotPoints])
    position_z = np.zeros([numPlotPoints])
    accels = []

    for i in range(numPlotPoints):
        t = time[i]
        positio = traj.get_position(t)
        position_x[i] += positio[0]
        position_y[i] += positio[1]
        position_z[i] += positio[2]
        accels.append(traj.get_acceleration(t))
    return traj, position_x, position_y, position_z, accels

def plotter(px, py, pz):
    f = plt.figure()
    ax = f.add_subplot(111, projection="3d")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_zlabel("z (cm)")

    ax.plot(px, py, pz)
    plt.show()
