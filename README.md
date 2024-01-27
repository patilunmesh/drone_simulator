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
        ├── docs
        ├── drone_simulation
        └── readme.md

```

## Build the workspace

```sh
cd ws_ros
colcon build --symlink-install
source install/setup.bash
```

## Run the simulation

```sh
cd src/drone_simulation/drone_simulation/launch
ros2 launch 2V1T_launch.py
```
