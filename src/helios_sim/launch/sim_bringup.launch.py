from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #Paths
    sim_pkg_share = get_package_share_directory('helios_sim')
    desc_pkg_share = get_package_share_directory('helios_description')
    world_path = os.path.join(sim_pkg_share, 'worlds', 'collapsed_industrial', 'collapsed_industrial.sdf')
    model_path = os.path.join(desc_pkg_share, 'models', 'HELIOS', 'model.sdf')
    survivor_path = os.path.join(desc_pkg_share, 'models', 'Survivor_Male', 'model.sdf')

    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path,
             '-v', '4',
             '--gui-config', os.path.join(sim_pkg_share, 'worlds', 'gui.config')],
        output='screen')
    
    spawn_helios_cmd = [
        'ign', 'service',
        '-s', '/world/collapsed_industrial/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000',
        '--req', f'sdf_filename: "{model_path}"'
    ]
    
    spawn_survivor_cmd = [
        'ign', 'service',
        '-s', '/world/collapsed_industrial/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000',
        '--req', f'sdf_filename: "{survivor_path}"'
    ]
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/collapsed_industrial/create[ros_gz_interfaces/srv/SpawnEntity]'
        ],
        output='screen'
    )

    multi_spawner = TimerAction(period=5.0, 
    actions = [
        Node(
        package='helios_sim',
        executable='multi_spawner.py',
        name='multi_spawner',
        output='screen'
    )])
    
    return LaunchDescription([
        ign_gazebo,
        # Delay and then spawn HELIOS
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=spawn_helios_cmd,
                    output='screen'
                )
            ]
        ),

        # Delay and then spawn Survivor_Male
        TimerAction(
            period=6.0,  # Spawn slightly later to ensure separation
            actions=[
                ExecuteProcess(
                    cmd=spawn_survivor_cmd,
                    output='screen'
                )
            ]
        ),
    ])
