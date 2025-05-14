from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 启用全局 use_sim_time 参数
    use_sim_time = True

    # 启动gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={
                    'world': os.path.join(get_package_share_directory('multi_axle_vehicle_model'), 'worlds', 'city.world'),
                    'pause': 'false'
                }.items(),
            )

    xacro_path = PathJoinSubstitution([
        FindPackageShare('multi_axle_vehicle_model'), 'xacro', 'car_combine.xacro'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_path
    ])

    # 发布 robot_description 到 Gazebo 和 RViz
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        name='robot_state_publisher'
    )

    # 定义spawn_entity节点
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_gazebo',
            '-x', '45',  # x 位置
            '-y', '14.0',    # y 位置
            '-z', '1.678', # z 位置
            '-R', '0.0',    # roll
            '-P', '0.0',    # pitch
            '-Y', '-1.5707963'   # yaw
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    string_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["string_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    wheel_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--controller-manager", "/controller_manager"],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_config_file = os.path.join(get_package_share_directory('multi_axle_vehicle_model'), 'rviz', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    road_visualization = Node(
        package='road_visualization',
        executable='road_publisher_node',
        name='road_publisher_node',
        output='screen'
    )

    return LaunchDescription([
        SetLaunchConfiguration('use_sim_time', str(use_sim_time)),  # 设置全局use_sim_time
        gazebo,                 # 启动gazebo
        robot_state_publisher,  # 启动robot_state_publisher
        spawn_entity,           # 启动spawn_entity
        joint_state_controller, # 启动joint_state_controller
        string_position_controller, # 启动string_position_controller
        wheel_velocity_controller,  # 启动wheel_velocity_controller
        rviz,                    # 启动RViz
        road_visualization
    ])