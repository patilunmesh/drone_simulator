from os.path import expanduser
from launch import LaunchDescription
from launch_ros.actions import Node

home = expanduser("~")

# launches 1 drone
def generate_launch_description():
    pkg_name = "drone_simulation"
    nrobots = 2
    return LaunchDescription(
        [
            Node(
                package= pkg_name,
                namespace="drone1",
                executable="drone_gym",
                name="sim",
                output="screen",
                emulate_tty=True,
                parameters=[{"start_pose": [0, 0, 0]}, {"my_id": 1}],
            ),
            Node(
                package= pkg_name,
                executable="obstacle_detector",
                name="detector",
                parameters=[{"N_robots": nrobots}, {"my_id": 1}],
                remappings=[
                    ("/local_traj", "/drone1/local_traj"),
                    ("/near_obstacles", "/drone1/near_obstacles"),
                ],
            ),
            #client
            Node(
                package= pkg_name,
                executable="sim_client",
                name="client",
                parameters=[{"N_robots": nrobots}, {"my_id": 1}],
                remappings=[
                    ("/local_traj", "/drone1/local_traj"),
                    ("/near_obstacles", "/drone1/near_obstacles"),
                ],
            ),
            Node(
                package="drone_simulation",
                executable="obstacle_generator",
                name="obstacle_generator",
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    [home + "/ros2_ws/src/drone_simulation/config/4drone.rviz"],
                ],
            ),
        ]
    )

