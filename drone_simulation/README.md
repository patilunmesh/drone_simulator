## A repository to simulate dynamics and control of a drone

For a given start and goal pose, simulate a drone trajectory tracked by an LQR controller.
The ROS2 node publishes Remote_id in real time.

# Single drone obstacle avoidance

![](https://github.com/sabotagelab/DroneSim_A58/blob/master/graphics/obst_avoid.gif)

# Multiple drones spawn example

![](https://github.com/sabotagelab/DroneSim_A58/blob/master/graphics/multi-agent.gif)

(four drones test scenario):

![](https://github.com/sabotagelab/DroneSim_A58/blob/master/graphics/4drones.png)

# Set of experiments

Here is a set of experiments that can be conducted using this simulation:

1. 1 victim - 1 Target (Herding strategy Traj. rollout Vs. Greedy Vs. Coarse coding).
2. Sweep N victims - 1 Target (Same as above).
3. Sweep N victims as a f(M) where M is number of targets (Traj rollout Vs. Greedy).
4. Realistic scenarios with inturrupted access and limited range of attack deviation.
5. Effect of bystanders.
6. Effect of formation constraints (Leader follower system)
