# Code partiellement généré par IA

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

#  Bridge QoS : /scan (BEST_EFFORT) -> /scan_reliable (RELIABLE)
QOS_BRIDGE_CODE = """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class InlineBridge(Node):
    def __init__(self):
        super().__init__('inline_qos_bridge')

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos_in)
        self.pub = self.create_publisher(LaserScan, '/scan_reliable', qos_out)
        self.get_logger().info('Bridge QoS /scan -> /scan_reliable actif')

    def cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = InlineBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""

# Suite d'instructions au lancement du fichier

def generate_launch_description():
    home = os.getenv('HOME')

    bridge_path = '/tmp/inline_qos_bridge.py'
    with open(bridge_path, 'w') as f:
        f.write(QOS_BRIDGE_CODE.strip())

    map_file           = os.path.join(home, 'Documents', 'maps', 'circuit_map.yaml')
    nav2_bringup_dir   = get_package_share_directory('nav2_bringup')
    nav2_params        = os.path.join(home, 'Documents', 'nav2_params.yaml')

    if not os.path.exists(map_file):
        print(f"\n\033[91m[ERREUR] CARTE INTROUVABLE : {map_file}\033[0m\n")

    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    return LaunchDescription([

        LogInfo(msg='=== AUTONOMIE NAV2 : micro-ROS + rf2o + bringup_launch ==='),

        # 1. Bridge QoS LIDAR
        ExecuteProcess(
            cmd=['python3', bridge_path],
            output='screen'
        ),

        # 2. Agent Micro-ROS
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
            output='screen'
        ),

        # 3. TF statique : base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.1', '--y', '0', '--z', '0.05',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ]
        ),

        # 4. Odométrie LIDAR (rf2o)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic':     '/scan_reliable',   # CORRECTIF 2
                'odom_topic':           '/odom',
                'publish_tf':           True,
                'base_frame_id':        'base_link',
                'odom_frame_id':        'odom',
                'init_pose_from_topic': '',
                'freq':                 20.0,
                'verbose':              True
            }]
        ),

        # 5. NAV2 via bringup_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map':          map_file,
                'use_sim_time': 'false',
                'params_file':  nav2_params,
                'autostart':    'true'
            }.items()
        ),

        # 6. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
            ],
            remappings=[('/scan', '/scan_reliable')],
            output='screen'
        ),
    ])
