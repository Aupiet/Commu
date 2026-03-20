import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    home = os.getenv('HOME')
    config_file = os.path.join(os.getenv('HOME'), 'Documents', 'ekf_params.yaml')
    ekf_config_file = os.path.join(os.getenv('HOME'), 'Documents', 'ekf_params.yaml')

    return LaunchDescription([
        # ============================================================
        # INFO DE DÉMARRAGE
        # ============================================================
        LogInfo(msg=f"""
        ===============================================================
        MODE AUTONOMIE
        Carte chargée depuis :
        {os.path.join(home, 'Documents', 'maps', 'circuit_map.yaml')}
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

        # ============================================================
        # 5. MAP SERVER (publication de la carte pour localisation & RViz)
        # ============================================================
        Node(
	    package='slam_toolbox',
	    executable='async_slam_toolbox_node',
	    name='slam_toolbox',
	    parameters=[{
		'mode': 'localization',
		'map_file': os.path.join(home, 'Documents/maps/circuit_map.yaml'),
		'use_sim_time': False
	    }],
	    output='screen'
	),

        # ============================================================
        # 6. NAV2 : navigation complète
        # ============================================================
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'map:Documents/maps/circuit_map.yaml'
            ],
            output='screen'
        ),

        # ============================================================
        # 7. RViz2 : vue Nav2 par défaut
        # ============================================================
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                os.path.join(
                    home,
                    'nav2_ws/src/navigation2/nav2_bringup/rviz/nav2_default_view.rviz'
                )
            ],
            output='screen'
        )
    ])
