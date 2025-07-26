import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('my_package'), 'urdf', 'my_robot.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('my_package'), 'rviz', 'display.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # 静的な座標変換を world -> odom に変更
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    # 新しく作成した simple_odom ノードを起動する設定を追加
    simple_odom_node = Node(
        package='my_package',
        executable='simple_odom.py',
        name='simple_odom_node'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        static_transform_publisher,
        simple_odom_node, # <-- 新しいノードを追加
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
