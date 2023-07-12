from launch import LaunchDescription
from launch_ros.actions import Node

#creates launch description in a loop

def generate_launch_description():
    drones = [
            {'p': [-5, 5, 0], 'id': 1},
            {'p': [-5, -5, 0], 'id': 2},
            {'p': [5, -5, 0], 'id': 3},
            {'p': [5, 5, 0], 'id': 4}
        ]
    launchlist = []
    for i, drone in enumerate(drones):
        initiator1 = Node(
                package='drone_simulation',
                namespace='drone'+str(i+1),
                executable='Drone_sim',
                name='sim',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {'start_pose': drone['p']},
                    {'my_id': drone['id']}
                ]
            )
        initiator2 = Node(
                package='drone_simulation',
                executable= 'obstacle_detector',
                name='detector'+str(i+1),
                output='screen',
                parameters=[{'N_robots': len(drones)},
                    {'my_id': drone['id']}],
                remappings=[
                    ('/local_traj', '/drone' + str(i+1) + '/local_traj'),
                    ('/near_obstacles', '/drone' + str(i+1) + '/near_obstacles'),
                ]
            )
        launchlist.append(initiator1)
        launchlist.append(initiator2)
    visualizer = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', ['/home/hlab/ros2_ws/src/drone_simulation/config/4drone.rviz']]
        )
    launchlist.append(visualizer)

    obstacle_creator = Node(
                package='drone_simulation',
                executable= 'obstacle_generator',
                name='obstacle_generator')
    launchlist.append(obstacle_creator)
    
    return LaunchDescription(launchlist)
        