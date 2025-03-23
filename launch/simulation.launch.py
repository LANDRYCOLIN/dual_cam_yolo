# launch/simulation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_cam_yolo',
            executable='video_publisher',
            name='camera_publisher',
            parameters=[{
                'left_source': 'left.mp4',
                'right_source': 'right.mp4',
                'frame_rate': 30.0
            }]
        ),
        Node(
            package='dual_cam_yolo',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[{
                'model_path': 'models/yolov8n.onnx',
                'confidence_threshold': 0.6
            }]
        )
    ])