"""
Launch file for the CBF Demo
"""

from os.path import expanduser
from launch import LaunchDescription
from launch_ros.actions import Node
home = expanduser("~")

# launches 1 drone
def generate_launch_description():
    pkg_name = "drone_simulation"
    nrobots = 1
    return LaunchDescription(
        [
            Node(
                package="drone_simulation",
                namespace="drone1",
                executable="drone_cbf_sim",
                name="sim",
                output="screen",
                emulate_tty=True,
                parameters=[{"start_pose": [0, 0, 0]}, {"my_id": 1}, {"cbf_param": 15.0}],
            ),
            Node(
                package="drone_simulation",
                executable="cbf_publisher",
                name="drone1_cbf_publisher",
                parameters=[{"my_id": 1, "sound_on": 0}]
            ),
            Node(
                package="drone_simulation",
                executable="obstacle_detector",
                name="detector",
                parameters=[{"N_robots": 2}, {"my_id": 1}],
                remappings=[
                    ("/local_traj", "/drone1/local_traj"),
                    ("/near_obstacles", "/drone1/near_obstacles"),
                ],
            ),
            Node(
                package="drone_simulation",
                executable="obstacle_generator",
                name="obstacle_generator"
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    [home+"/ros2_ws/src/drone_simulation/config/4drone.rviz"],
                ],
            ),
        ]
    )