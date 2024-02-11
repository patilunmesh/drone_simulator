# ROS2 Autonomous UAV Simulation

## Setup a ROS2 workspace

```sh
$ tree

ros2_ws
├── build
├── install
├── log
└── src
```

Remember to source the setup.bash file in the install folder or add it to your bashrc/zshrc file

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Clone the repository in the src folder

```sh
$ tree

ws_ros
├── build
├── install
├── log
└── src
    └── drone_simulation
        ├── drone_simulation
        └── README.md

```

## Build the workspace

```sh
cd ros2_ws
colcon build
source install/setup.bash
```

## Run the simulation

```sh
ros2 launch drone_simulation single_agent_launch.py
```
