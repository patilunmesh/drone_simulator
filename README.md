# ROS2 Autonomous UAV Simulation

## Setup a ROS2 workspace

```sh
$ tree

ws_ros
├── build
├── install
├── log
└── src
```

Remember to source the setup.bash file in the install folder or add it to your bashrc/zshrc file

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
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
colcon build --symlink-install
source install/setup.bash
```

## Run the simulation

```sh
ros2 launch single_agent_launch.py
```
