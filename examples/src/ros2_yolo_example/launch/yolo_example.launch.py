from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        Node(
            package='ros2_yolo_example',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'model': LaunchConfiguration('model'),
                'conf': LaunchConfiguration('conf'),
            }]
        ),
    ])
