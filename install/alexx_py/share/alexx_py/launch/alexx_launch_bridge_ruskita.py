from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node  # Adicionado para o nó da bridge
import os
def generate_launch_description():
    # Caminhos para mundo e robô (ajuste se necessário)
    world_path = os.path.abspath('blender_lab.sdf')  # Assumindo que está na pasta worlds/
    robot_path = os.path.abspath('bb_alexx.sdf')  # Não usado no cmd atual, mas mantido
    # Comando para iniciar o Ignition Gazebo com o mundo
    start_ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '/home/kifelix/alexx_ws/src/alexx_py/worlds/blender_lab.sdf'],  # Usando a variável world_path
        output='screen'
    )
            # Pontes ROS <-> Ignition
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/camera@sensor_msgs/msg/Image@ignition::msgs::Image'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/meu_mundo/dynamic_pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V'],
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

        # Node do pacote ros2_gazeboworld
        Node(
            package='ros2_gazeboworld',
            executable='ros2_bridge',
            name='ros2_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/camera', '/camera')]
        ),
    return LaunchDescription([
        start_ign_gazebo,
        TimerAction(  # Opcional: Delay de 2s para garantir que Gazebo inicie antes da bridge
            period=2.0,
            actions=[bridge_node]
        ),
    ])

