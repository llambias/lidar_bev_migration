# Este launch file está basado en el 32bv.launch del repositorio original.
# Están ya ingresados los valores de cloud_topic y lidar_tf_frame, pero no el de camera_tf_frame.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('lidar_bev'),
        "config",
        "params.yaml"
    )

    lidar_bev_node = Node(
        package="lidar_bev",
        executable="lidar_bev",
        name="lidar_bev",
        output="screen",
        parameters=[config]
    )

    ld.add_action(lidar_bev_node)

    return ld