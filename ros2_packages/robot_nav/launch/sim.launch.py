"""Launch de simulação: Gazebo Harmonic + robô diff-drive + bridges ROS↔GZ.

Substitui robot.launch.py/lidar.launch.py/hoverboard no modo --sim.
O Gazebo cuida de:
  - Publicar /scan (LiDAR GPU da SDF)
  - Publicar /odom e TF odom→base_link (plugin DiffDrive)
  - Consumir /cmd_vel do servidor web

O robot_state_publisher ainda roda porque a URDF fornece os TFs estáticos
(base_link → base_laser, rodas) que o slam_toolbox e o Nav2 usam.
"""
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_nav = get_package_share_directory('robot_nav')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Caminhos padrão (sobrescrevíveis via argumentos de launch)
    default_world = os.environ.get('SIM_WORLD', '')  # preenchido pelo launch.sh
    default_robot_sdf = os.path.join(pkg_robot_nav, 'urdf', 'husky.sdf')
    urdf_xacro = os.path.join(pkg_robot_nav, 'urdf', 'husky.urdf.xacro')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Caminho absoluto para o arquivo .sdf/.world do Gazebo',
    )
    robot_sdf_arg = DeclareLaunchArgument(
        'robot_sdf',
        default_value=default_robot_sdf,
        description='Caminho para o SDF do robô simulado',
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='2.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='2.5')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.2')

    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    # Carrega a URDF (xacro) para os TFs estáticos do robot_state_publisher
    robot_description = xacro.process_file(urdf_xacro).toxml()

    # --- Gazebo (gz sim) ---
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -r = start paused=false, -v4 = verbose
            'gz_args': ['-r -v 4 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # --- Spawn do robô no mundo ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf,
            '-name', 'husky',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
        ],
        output='screen',
    )

    # --- Bridges ROS ↔ GZ ---
    # Formato: /topic@ros_type[gz_type   (GZ → ROS)
    #          /topic@ros_type]gz_type   (ROS → GZ)
    #          /topic@ros_type@gz_type   (bidirecional)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Câmera RGB-D — RGB pra viz web, point cloud pro VoxelLayer
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    # --- robot_state_publisher: usa o mesmo URDF do robô real ---
    # Garante base_link → base_laser para o slam_toolbox e o Nav2.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        robot_sdf_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gz_sim_launch,
        bridge,
        spawn_robot,
        rsp,
    ])
