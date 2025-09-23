# Beam Detector ROS 2 Package

A ROS 2 node for detecting beams using YOLO11-OBB model, converted from the original image inference code.

## Features

- Subscribes to image topics and performs real-time beam detection
- Publishes annotated images with detected beams
- Tracks horizontal and vertical beams with scale correction
- Configurable confidence threshold and model parameters

## Dependencies

- ROS 2 Humble
- OpenCV
- ONNX Runtime
- cv_bridge

## Usage

### Build the package
```bash
cd /workspace
colcon build --packages-select beam_detector_pkg
source install/setup.bash
```

### Run with launch file
```bash
ros2 launch beam_detector_pkg beam_detector.launch.py
```

### Run with custom parameters
```bash
ros2 launch beam_detector_pkg beam_detector.launch.py \
  model_path:=/path/to/your/model.onnx \
  labels_path:=/path/to/your/labels.names \
  conf_threshold:=0.6 \
  input_topic:=/your_camera/image_raw \
  output_topic:=/beam_detector/annotated_image
```

### Run the node directly
```bash
ros2 run beam_detector_pkg beam_detector_node
```

## Topics

### Subscribed Topics
- `/camera/camera/color/image_raw` (sensor_msgs/Image) - RealSense color camera images

### Published Topics  
- `/beam_detector/image` (sensor_msgs/Image) - Annotated images with detected beams

## Parameters

- `model_path` (string): Path to ONNX model file
- `labels_path` (string): Path to class labels file  
- `conf_threshold` (double): Confidence threshold for detections (default: 0.5)
- `use_gpu` (bool): Use GPU for inference (default: true)
- `input_topic` (string): Input image topic (default: "/camera/camera/color/image_raw")
- `output_topic` (string): Output image topic (default: "/beam_detector/image")

## Model Files

The node expects the following files from the original BeamDetectorInference:
- `/workspace/BeamDetectorInference/models/best.onnx` - YOLO11-OBB model
- `/workspace/BeamDetectorInference/models/Dota.names` - Class labels

## Beam Detection Output

The node detects and tracks:
- **Horizontal beams**: Top and bottom horizontal structural elements
- **Vertical beams**: Up to 2 vertical structural elements  
- **Scale factor**: Dynamic scaling based on beam distance

Detection results are visualized with:
- Green lines overlaid on detected beam positions
- Text labels (H_TOP, H_BOT, V0, V1)
- Scale factor display