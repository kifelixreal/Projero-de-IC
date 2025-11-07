import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Caminho para os diretórios do pacote
    pkg_share = FindPackageShare('alexx_py')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'blender_lab.sdf'])
    models_path = PathJoinSubstitution([pkg_share, 'robot'])

    # Definir variável de ambiente para IGN_GAZEBO_RESOURCE_PATH
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path
    )

    return LaunchDescription([
        # Define argumentos (pode ser expandido no futuro)
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world', default_value=world_path, description='Path to the world file'
        ),

        # Exportar caminho dos modelos
        set_ign_resource_path,

        # Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', LaunchConfiguration('world')],
            output='screen'
        ),

        # Pontes ROS <-> Ignition
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output='screen'
        ),  
        
        # Nó de visão computacional (adicionado para integração completa)
        Node(
            package='alexx_py',  # Nome do seu pacote
            executable='vision_node',  # Executável definido no setup.py
            name='vision_node',
            output='screen'
        )                    
    ])
