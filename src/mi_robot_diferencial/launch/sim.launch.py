import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('mi_robot_diferencial')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    default_world = os.path.join(pkg_path, 'worlds', 'pista.sdf')
    default_rviz = os.path.join(pkg_path, 'rviz', 'robot.rviz')

    # Argumentos de lanzamiento
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    x, y, yaw = (LaunchConfiguration(n) for n in ('x', 'y', 'yaw'))

    args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Ruta al world SDF'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: Gazebo sin interfaz gráfica'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir RViz2'),
        # Punto de spawn definido sobre la línea de salida de la pista
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

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

    # Iniciar Gazebo Sim (-r: inicia corriendo; -s: solo servidor)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        condition=UnlessCondition(headless)
    )
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
        condition=IfCondition(headless)
    )

    # Spawn del robot en Gazebo (se retrasa para dar tiempo a que cargue el world)
    spawn_entity = TimerAction(period=3.0, actions=[Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'yahboom_microros_4wd',
            '-x', x, '-y', y, '-z', '0.05', '-Y', yaw
        ],
        output='screen'
    )])

    # ROS-Gazebo Bridge
    #   [  Gazebo -> ROS      ]  ROS -> Gazebo      @  bidireccional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        parameters=[{'use_sim_time': True}],
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
        arguments=['-d', default_rviz],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription(args + [
        node_robot_state_publisher,
        gazebo,
        gazebo_headless,
        spawn_entity,
        bridge,
        node_kinematics,
        rviz
    ])
