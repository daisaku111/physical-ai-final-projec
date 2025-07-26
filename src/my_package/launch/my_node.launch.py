import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # worldファイルのパスを取得
    world_file = os.path.join(get_package_share_directory('my_package'), 'worlds', 'my_world.world')

    # Gazeboを起動
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # URDFファイルを読み込み
    urdf_file = os.path.join(get_package_share_directory('my_package'), 'urdf', 'my_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # robot_state_publisherを起動
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # spawn_entityを起動
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
