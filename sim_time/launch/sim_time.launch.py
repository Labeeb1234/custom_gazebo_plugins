from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )
  
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'true',
        description='Flag to enable sim time in gazebo when launched'

    )

    sim_time_node_spawner = Node(
        package='sim_time',
        executable='sim_time_node',
        output = 'screen',
        parameters=[{'use_sime_time': LaunchConfiguration('use_sim_time') }]
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        use_sim_time_arg,
        sim_time_node_spawner
    ])
