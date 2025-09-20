from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='')

    prefix = LaunchConfiguration('prefix')

    # Nodes
    node_lidar2d_0_gz_bridge = Node(
        name='lidar2d_0_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a200_0846/sensors/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': '/home/hiba/clearpath_sim/sensors/config/lidar2d_0.yaml'
                    ,
                }
                ,
            ]
        ,
    )

    node_lidar2d_0_static_tf = Node(
        name='lidar2d_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='a200_0846',
        output='screen',
        arguments=
            [
                '--frame-id'
                ,
                'lidar2d_0_link'
                ,
                '--child-frame-id'
                ,
                'a200_0846/robot/base_link/lidar2d_0'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
                (
                    '/tf_static'
                    ,
                    'tf_static'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_lidar2d_0_gz_bridge)
    ld.add_action(node_lidar2d_0_static_tf)
    return ld
