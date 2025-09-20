#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # === Launch Configurations ===
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_rviz = LaunchConfiguration('config')

    # === Launch Arguments ===
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='a200_0846',
        description='Namespace du robot'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Utiliser le temps simulé (true/false)'
    )

    declare_config = DeclareLaunchArgument(
        'config',
        default_value='robot.rviz',
        description='Fichier de configuration RViz'
    )

    # === Chemin du fichier RViz ===
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('clearpath_viz'),
        'rviz',
        config_rviz
    ])

    # === Groupe RViz dans le namespace ===
    rviz_node = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )
    ])

    # === Assemble la LaunchDescription ===
    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_config,
        rviz_node
    ])

