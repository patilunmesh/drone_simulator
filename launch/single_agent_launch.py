from launch import LaunchDescription
from launch_ros.actions import Node
import os
#launches 4 drones
def generate_launch_description():
    pkg_name = 'drone_simulation'
    pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s && pwd"' % pkg_name).read().strip()
    return LaunchDescription([
        Node(
            package='drone_simulation',
            namespace='drone1',
            executable='Drone_sim',
            name='sim',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'start_pose': [0, 0, 0]},
                {'my_id': 1}
            ]
        ),
        
        Node(
            package='drone_simulation',
            executable= 'obstacle_detector',
            name='detector',
            remappings=[
                ('/local_traj', '/drone1/local_traj'),
                ('/near_obstacles', '/drone1/near_obstacles'),
            ]
        ),

        Node(
                package='drone_simulation',
                executable= 'obstacle_generator',
                name='obstacle_generator'),

        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', ['/home/hlab/ros2_ws/src/drone_simulation/config/drone.rviz']]
        )
    ])

#os.path.join(pkg_dir, 'config', 'drone.rviz')