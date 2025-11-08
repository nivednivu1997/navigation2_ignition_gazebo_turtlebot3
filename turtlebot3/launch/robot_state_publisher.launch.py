import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        f'turtlebot3_{TURTLEBOT3_MODEL}.urdf'
    )

    # Read and clean the URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read().replace('${namespace}', '')  # remove namespace placeholders

    # Launch robot_state_publisher
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
    ])

