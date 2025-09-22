# Este launch file está basado en el 32bv.launch del repositorio original.
# Están ya ingresados los valores de cloud_topic y lidar_tf_frame, pero no el de camera_tf_frame.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Declare all launch arguments
    return LaunchDescription([

        # Tópicos
        DeclareLaunchArgument('cloud_topic', default_value='/wamv/sensors/lidars/ouster_lidar_sensor/points'),
        DeclareLaunchArgument('lidar_tf_frame', default_value='/wamv/wamv/base_link/ouster_lidar_sensor'),
        DeclareLaunchArgument('camera_tf_frame', default_value='/stereo_camera'),
        
        # Map configuration
        DeclareLaunchArgument('camera_fov', default_value='110.0'),
        DeclareLaunchArgument('intensity_threshold', default_value='0.05'),
        DeclareLaunchArgument('cell_size', default_value='0.05'),
        DeclareLaunchArgument('cell_size_height_map', default_value='0.25'),
        DeclareLaunchArgument('max_height', default_value='3.0'),
        DeclareLaunchArgument('num_slices', default_value='1'),
        DeclareLaunchArgument('grid_dim', default_value='70'),
        DeclareLaunchArgument('grid_dim_height_map', default_value='300'),
        DeclareLaunchArgument('height_threshold', default_value='0.10'),
        DeclareLaunchArgument('crop_180', default_value='true'),
        DeclareLaunchArgument('remove_floor', default_value='false'),
        
        # HDL -32E  ->  "El sensor Lidar de Detección de Peligros (HDL) es un sistema de imágenes 3D basado en láser que escanea una superficie para crear un mapa 3D del campo de aterrizaje."
        DeclareLaunchArgument('planes', default_value='32'),
        DeclareLaunchArgument('h_res', default_value='0.2'),
        DeclareLaunchArgument('v_res', default_value='1.33'),
        DeclareLaunchArgument('low_opening', default_value='-30.67'),
        DeclareLaunchArgument('max_expected_intensity', default_value='30'),

        # Node
        Node(
            package='lidar_bev',
            executable='lidar_bev',
            name='lidar_bev',
            output='screen',
            parameters=[{
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'lidar_tf_frame': LaunchConfiguration('lidar_tf_frame'),
                'camera_tf_frame': LaunchConfiguration('camera_tf_frame'),
                'camera_fov': LaunchConfiguration('camera_fov'),
                'planes': LaunchConfiguration('planes'),
                'h_res': LaunchConfiguration('h_res'),
                'v_res': LaunchConfiguration('v_res'),
                'low_opening': LaunchConfiguration('low_opening'),
                'cell_size': LaunchConfiguration('cell_size'),
                'grid_dim': LaunchConfiguration('grid_dim'),
                'max_height': LaunchConfiguration('max_height'),
                'num_slices': LaunchConfiguration('num_slices'),
                'height_threshold': LaunchConfiguration('height_threshold'),
                'cell_size_height_map': LaunchConfiguration('cell_size_height_map'),
                'grid_dim_height_map': LaunchConfiguration('grid_dim_height_map'),
                'max_expected_intensity': LaunchConfiguration('max_expected_intensity'),
                'crop_180': LaunchConfiguration('crop_180'),
                'remove_floor': LaunchConfiguration('remove_floor')
            }]
        )
    ])


# Más información de cómo migrar launch files: https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html