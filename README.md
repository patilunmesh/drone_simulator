# ROS2 Autonomous UAV Simulation

## Citation
If you are using this work in your research, you can cite this paper:

Title: ROS2-Based Simulation Framework for Cyberphysical Security Analysis of UAVs

Link: https://arxiv.org/pdf/2410.03971

Citation:
```
Patil, U., Gunasekaran, A., Bobba, R., & Abbas, H. (2024). ROS2-Based Simulation Framework for Cyberphysical Security Analysis of UAVs. arXiv preprint arXiv:2410.03971.
```
## Prerequisites
```
sudo apt-get install ros-humble-tf-transformations
pip install transform3d
```
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
