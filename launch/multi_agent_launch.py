from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            name='detector1',
            remappings=[
                ('/local_traj', '/drone1/local_traj'),
                ('/near_obstacles', '/drone1/near_obstacles'),
            ]
        ),

        Node(
            package='drone_simulation',
            namespace='drone2',
            executable='Drone_sim',
            name='sim2',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'start_pose': [0, 5, 0]},
                {'my_id': 2}
            ]
        ),

        Node(
            package='drone_simulation',
            executable= 'obstacle_detector',
            name='detector2',
            output='screen',
            remappings=[
                ('/local_traj', '/drone2/local_traj'),
                ('/near_obstacles', '/drone2/near_obstacles'),
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
            arguments=['-d', ['/home/hlab/ros2_ws/src/drone_simulation/config/2drone.rviz']]
        )

    ])