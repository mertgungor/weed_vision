from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():


    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('weed_vision'), 'rviz', 'rviz.rviz']
    )

    rosbag_path = PathJoinSubstitution(
        [FindPackageShare('weed_vision'), 'bag']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'rosbag_file',
            default_value=rosbag_path,
            description='Path to the rosbag file to be played'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        # Launch RViz
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            # parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz"))
        ),

        Node(
            package='weed_vision',
            executable='model.py',
            name='model',
            output='screen'
        ),

        # Play rosbag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_file'), '--loop', '-r', '0.5'],
            output='screen'),

    ])
