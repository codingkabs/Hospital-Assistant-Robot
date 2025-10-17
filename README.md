# RoboKab - Healthcare Assistant Robot

A ROS-based healthcare assistant robot system that autonomously navigates through a hospital environment to check on patients, verify medication compliance, and report back to doctors.

## Project Overview

RoboKab is an intelligent healthcare assistant robot designed to:
- Navigate autonomously through hospital rooms
- Detect patients using computer vision (YOLO)
- Verify medication compliance through voice interaction
- Generate and deliver reports to doctors
- Use state machine architecture for robust task execution

## System Architecture

The system consists of several interconnected ROS nodes:

### Core Components

1. **Main Node (`main_node.py`)** - Central state machine orchestrating the entire workflow
2. **YOLO ROS Node (`yolo_ros.py`)** - Computer vision service for person detection
3. **Room2Med Service (`room2med.py`)** - Medication lookup service
4. **Launch File (`itr_cw.launch`)** - System startup configuration

### State Machine Workflow

The robot follows this sequence:
1. **Initialize** - Boot up and announce mission
2. **Move to Doctor** - Navigate to doctor's location
3. **Talk to Doctor** - Brief the doctor on the mission
4. **Visit Room C** - Navigate, detect patient, ask about medication
5. **Visit Room F** - Navigate, detect patient, ask about medication  
6. **Report Back** - Return to doctor and deliver patient reports

## Prerequisites

### System Requirements
- Ubuntu 18.04/20.04 (ROS Melodic/Noetic)
- Python 3.6+
- CUDA-capable GPU (recommended for YOLO)

### ROS Dependencies
```bash
sudo apt-get install ros-melodic-desktop-full
sudo apt-get install ros-melodic-cv-bridge
sudo apt-get install ros-melodic-tf
sudo apt-get install ros-melodic-move-base-msgs
sudo apt-get install ros-melodic-geometry-msgs
sudo apt-get install ros-melodic-smach
sudo apt-get install ros-melodic-smach-ros
sudo apt-get install ros-melodic-actionlib
```

### External Dependencies

#### YOLOv4 and Darknet
```bash
# Install Darknet
cd /opt
sudo git clone https://github.com/AlexeyAB/darknet.git
cd darknet
sudo make

# Download YOLOv4 weights
sudo wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
```

#### Python Dependencies
```bash
pip3 install opencv-python
pip3 install rospy
pip3 install smach
pip3 install actionlib
```

#### Additional ROS Packages
```bash
# Vosk speech recognition
sudo apt-get install ros-melodic-ros-vosk

# Stage simulator (for testing)
sudo apt-get install ros-melodic-stage-ros
sudo apt-get install ros-melodic-rosplan-stage-demo
```

## Installation

1. **Clone the repository:**
```bash
cd ~/catkin_ws/src
git clone <repository-url> resit_coursework
```

2. **Build the workspace:**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. **Configure paths:**
   - Update the video folder path in `launch/itr_cw.launch` (line 5)
   - Ensure YOLO configuration paths are correct in `scripts/yolo_ros.py` (lines 16-19)

4. **Set up environment:**
```bash
# Add to ~/.bashrc
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src
source ~/catkin_ws/devel/setup.bash
```

## Configuration

### Room Coordinates
The robot navigates to predefined coordinates:
- **Doctor's Office**: (6.0, 8.5, 0.0)
- **Room C**: (10.6, 8.5, 0.0) 
- **Room F**: (10.6, 2.5, 0.0)
- **Report Location**: (6.0, 8.5, 1.57)

### Medication Mapping
Room-to-medication mapping is defined in `scripts/room2med.py`:
- Room C: Paracetamol
- Room F: Ibuprofen

### YOLO Configuration
- Model: YOLOv4
- Confidence threshold: 0.2
- Classes: COCO dataset (80 classes)
- GPU acceleration enabled

## Usage

### Running the System

1. **Start the complete system:**
```bash
roslaunch resit_coursework itr_cw.launch
```

2. **Run individual components:**
```bash
# Main state machine
rosrun resit_coursework main_node.py

# YOLO detection service
rosrun resit_coursework yolo_ros.py

# Medication lookup service
rosrun resit_coursework room2med.py
```

### Testing Individual Services

```bash
# Test medication lookup
rosservice call /room2med "room_name: 'room_c'"

# Test person detection
rosservice call /detect_frame "object_class: 'person'"
```

## System Services

### Room2Med Service
- **Service Name**: `/room2med`
- **Request**: `string room_name`
- **Response**: `string medication`
- **Purpose**: Maps room names to medication types

### YOLO Detection Service
- **Service Name**: `/detect_frame`
- **Request**: `string object_class`
- **Response**: `bool detected`
- **Purpose**: Detects specified objects in camera feed

## Robot Behavior

### Navigation
- Uses ROS `move_base` action server for autonomous navigation
- Implements 360-degree spinning for patient detection
- Handles navigation failures gracefully

### Patient Interaction
- Spins 360° to detect patients in rooms
- Uses text-to-speech for medication inquiries
- Records patient presence and medication compliance

### Reporting
- Generates comprehensive patient reports
- Returns to doctor's location for report delivery
- Implements retry logic for doctor detection

## Troubleshooting

### Common Issues

1. **YOLO not detecting objects:**
   - Check camera feed: `rostopic echo /camera/image`
   - Verify YOLO weights and config files exist
   - Ensure GPU drivers are properly installed

2. **Navigation failures:**
   - Check if `move_base` is running
   - Verify map server is active
   - Ensure robot has valid pose in map

3. **Service call failures:**
   - Check if all services are running: `rosservice list`
   - Verify service definitions: `rosservice info /service_name`

### Debug Commands

```bash
# Check running nodes
rosnode list

# Monitor topics
rostopic list
rostopic echo /topic_name

# Check services
rosservice list
rosservice info /service_name

# View logs
rosrun rqt_console rqt_console
```

## File Structure

```
resit_coursework/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package dependencies
├── config/
│   └── coco.data          # YOLO class definitions
├── launch/
│   └── itr_cw.launch      # System launch file
├── scripts/
│   ├── main_node.py       # Main state machine
│   ├── yolo_ros.py        # YOLO detection service
│   ├── room2med.py        # Medication lookup service
│   └── itr_cwCS24_resit   # Test script
└── srv/
    ├── Room2Med.srv       # Medication service definition
    └── YOLOLastFrame.srv  # Detection service definition
```

## Development Notes

- The system uses SMACH (State Machine) for robust task execution
- All Python scripts are executable and include proper shebangs
- Error handling is implemented for service calls and navigation
- The robot includes speech synthesis for patient interaction
- State machine allows for easy modification of workflow

## License

This project is developed for educational purposes as part of a robotics coursework.

## Author

**Kabir Suri** (k21082509@kcl.ac.uk)
King's College London

---

*For technical support or questions, please refer to the troubleshooting section or contact the maintainer.*
