import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Chemin vers le fichier de configuration YAML créé à l'étape 1
    # Assurez-vous que le chemin est correct ! Ici : ~/Documents/slam_params.yaml
    config_file = os.path.join(os.getenv('HOME'), 'Documents', 'ekf_params.yaml')
    ekf_config_file = os.path.join(os.getenv('HOME'), 'Documents', 'ekf_params.yaml')

    return LaunchDescription([
        
        # === MESSAGE IMPORTANT POUR LA SYNCHRO ===
        LogInfo(msg="""
        ===============================================================
        ATTENTION - PROCÉDURE DE SYNCHRONISATION REQUISE :
        1. Lancez ce fichier.
        2. Attendez que l'agent Micro-ROS dise 'Listening on port 8888'.
        3. APPUYEZ SUR LE BOUTON 'RESET' DU ROBOT.
        
        Si vous ne faites pas ça, la map restera vide (Problème 1970).
        ===============================================================
        """),

        # 1. Micro-ROS Agent
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
            output='screen'
        ),

        # 2. Transformations (TF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.1', '--y', '0', '--z', '0.05', 
                       '--yaw', '0', '--pitch', '0', '--roll', '0', 
                       '--frame-id', 'base_link', '--child-frame-id', 'laser']
        ),
        
        # 3. Odométrie Laser (rf2o)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
                'verbose': True
            }],
        ),
        # 4. EKF Node (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file]
        ),

        # 5. SLAM Toolbox (Avec chargement du YAML)
        # On utilise le Node direct plutôt que le launch file inclus pour mieux contrôler les paramètres
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file] # <--- C'est ici que la magie QoS opère
        ),

        # 6. Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'slam_toolbox_default.rviz')]
        )
    ])
