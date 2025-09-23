#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'model_path',
            default_value='/workspace/BeamDetectorInference/models/best.onnx',
            description='Path to ONNX model file'
        ),
        
        DeclareLaunchArgument(
            'labels_path',
            default_value='/workspace/BeamDetectorInference/models/Dota.names',
            description='Path to class labels file'
        ),
        
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.5',
            description='Confidence threshold for detections'
        ),
        
        DeclareLaunchArgument(
            'use_gpu',
            default_value='true',
            description='Use GPU for inference'
        ),
        
        DeclareLaunchArgument(
            'input_topic',
            default_value='/camera/camera/color/image_raw',
            description='Input image topic'
        ),
        
        DeclareLaunchArgument(
            'output_topic',
            default_value='/beam_detector/image',
            description='Output image topic with detections'
        ),
        
        # Beam detector node
        Node(
            package='beam_detector_pkg',
            executable='beam_detector_node',
            name='beam_detector_node',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'labels_path': LaunchConfiguration('labels_path'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'use_gpu': LaunchConfiguration('use_gpu'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }],
            output='screen'
        )
    ])