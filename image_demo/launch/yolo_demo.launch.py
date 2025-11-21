from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pathlib import Path

def generate_launch_description():
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=str(Path.home() / 'Desktop' / 'thesi' / 'ROS2-Tutorial' / 'examples' / 'data' / 'sample.mp4'),
        description='Absolute path to the input video'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=str(Path.home() / 'Desktop' / 'thesi' / 'ROS2-Tutorial' / 'examples' / 'data' / 'yolov8n.pt'),
        description='Absolute path to YOLO model .pt'
    )
    conf_thres_arg = DeclareLaunchArgument(
        'conf_thres',
        default_value='0.25',
        description='YOLO confidence threshold'
    )

    yolo = Node(
        package='image_demo',
        executable='video_yolo',
        name='video_yolo',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'model_path': LaunchConfiguration('model_path'),
            'conf_thres': LaunchConfiguration('conf_thres'),
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(Path('/ws/src/ROS2-Tutorial/image_demo/rviz/yolo_image.rviz'))],
        output='screen'
    )

    return LaunchDescription([
        video_path_arg, model_path_arg, conf_thres_arg,
        yolo, rviz
    ])
