from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import os
def generate_launch_description():
    world_path = os.path.abspath('bb_mundo.sdf')
    robot_path = os.path.abspath('bb_alexx.sdf')
    # Comando para iniciar o Ignition Gazebo com o mundo
    start_ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '/home/kifelix/alexx_ws/src/alexx_py/worlds/bb_mundo.sdf'],
        output='screen'
    )
    return LaunchDescription([
        start_ign_gazebo,     
    ])
