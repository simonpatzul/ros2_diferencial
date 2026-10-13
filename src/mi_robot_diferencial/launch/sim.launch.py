import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('mi_robot_diferencial')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # Procesar archivo Xacro
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Nodo robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # Iniciar Gazebo Sim (Forzando inicio en tiempo real con -r)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn del robot en Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'yahboom_microros_4wd',
            '-z', '0.05'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge (INCLUYE EL PUENTE DE /clock CRUCIAL)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.PoseVM',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Nodo de cinemática de llantas propio
    node_kinematics = Node(
        package='mi_robot_diferencial',
        executable='kinematics_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        node_kinematics,
        rviz
    ])