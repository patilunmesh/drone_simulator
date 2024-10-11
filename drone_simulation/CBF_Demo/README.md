# CBF Demo

This demo uses Control Barrier Functions (CBF) for ensuring obstacle avoidance. CBFs are a control method for dealing with safety in terms of set invariance. The demo also includes for sonifying the CBF as a means of illustrating how the function values change as drone moves relative to the "unsafe set", which in this case is a single obstacle. The goal of this demo is to provide a basis for educating broad audiences about CBFs, safe autonomy, and sonification.


## Additional Prerequisites
In addition to the prerequisites and setup outlined in the top-level README of this repo,
the following is needed for the sonification aspect of this demo:

```
sudo apt-get install libportaudio2
pip install pysinewave
```

## Launching and Interacting with the Simulation
After setting up and building the workspace as outlined in the top-level README of this repo, the following launches the demo:
```
ros2 launch drone_simulation single_agent_cbf_launch.py
```

By default the sound is off. Run the following, which will prompt keyboard entry to turn the sound on/off:
```
ros2 run drone_simulation cbf_sound_client.py
> Toggle Sound (1 for on / other for off): 1
```

A CBF parameter is used to determine how cautious the controller will be when computing safe inputs for obstacle avoidance.
To change this parameter, run the following, which will prompt for keyboard entry of the new parameter value.
```
ros2 run drone_simulation drone_cbf_client.py
> Set new CBF parameter (float): 15
```
The default parameter is 15. Running the above will reset the simulation to the initial state

## About the files in this folder

- `cbf_publisher.py`: Node to compute and publish CBF values for the given drone id and safety radius.
- `cbf_sim_client.py`: A client node for testing and interfacing with the CBF simulator in `cbf_sim.py`.
- `cbf_sim.py`: Node that runs the simulation and visualizes all aspects (drone, trajectories, obstacle). This node also uses the published CBF values to recompute a safe trajectory for the drone when detecting an obstacle.
- `cbf_sound_client.py`: A client node for testing and interfacing with the CBF publisher defined in `cbf_publisher.py`

## Future Work
The sound library I opted to use is kind of finicky, and often crashed when testing within the virtual environment in which I was developing. Future work would include exploring more robust sound libraries.
Other enhancements to this project include incorporating multiple drones, and sonifying their different CBF values at the same time.

## Resources
- [](https://github.com/HybridRobotics/CBF-CLF-Helper)https://github.com/HybridRobotics/CBF-CLF-Helper: For guidance on CBF implementation
- [](https://github.com/daviddavini/pysinewave)https://github.com/daviddavini/pysinewave: Documentation for the sound library used
- [](http://docs.ros.org/en/humble/): ROS2 documentation
