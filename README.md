# AR Ghost Mentor

Visualizing Tacit Knowledge of Skilled Veterans using an AR Ghost Skeleton to Bridge Technical Training and Language Barriers

This repository features a robust, real-time autonomous gesture matching and guidance system designed for technical skill transfer. The system integrates real-time skeleton normalization, weighted cosine similarity, and dynamic time warping (DTW) window search to guide learners through complex manual workflows via AR overlays.
Tech Stack & Environment

- OS: Ubuntu 22.04 LTS

- Framework: ROS 2 (Humble)

- Language: Python 3

- Libraries: MediaPipe, OpenCV, NumPy

- Hardware: RGB-D Camera / Webcam, Terminal Dashboard Display

# Key Modules & Architecture

- Perception & Preprocessing

- Spatial Normalization using hip center and shoulder width

- 11 Major Bone Segment Vector Extraction

- Frame Filtering & Multi-Iteration Ensemble Preprocessing

- Estimation & Matching

- Weighted Cosine Similarity Scoring

- Unidirectional Dynamic Time Warping (DTW) Sliding Window Search

- Error Vector Estimation for Angular Discrepancy Guidance

- Control & Visualization

- Real-Time AR Skeleton Overlay UI

- Event-Driven State and Milestone Publishing

- Terminal-Based Live Scoring Dashboard

# Project Structure
```
├── ros2_ws/src/ghost_mentor/
│   ├── ghost_mentor/               <- ROS 2 Node Package
│   │   ├── init.py
│   │   ├── veteran_recorder.py     # [Node] Records veteran movements
│   │   ├── match_engine.py         # [Library] Gesture matching & scoring algorithm
│   │   ├── ar_overlay_node.py      # [Node] Main AR overlay UI for learners
│   │   └── score_dashboard.py      # [Node] Terminal-based score dashboard
│   ├── launch/
│   │   └── demo_launch.py          # Launches the entire system at once
│   ├── package.xml
│   └── setup.py
│
├── tools/
│   └── ghost_builder.py                    # Preprocesses raw recordings into Master Ghost (No ROS 2 required)
│
└── data/
├── raw/                                # Raw JSON recordings from veterans (Multiple files)
└── sequences/                          # Processed Master Ghost JSON sequences
```

# Installation & Setup
Prerequisites & Dependencies
```
# Install Python packages
pip install mediapipe==0.10.9 opencv-python==4.8.1.78 "numpy<2" --break-system-packages --no-deps

# Install ROS 2 package dependencies
sudo apt install ros-humble-cv-bridge ros-humble-visualization-msgs
```

Build the Workspace
```
# Navigate and build
cd ~/ar_ghost_mentor/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ghost_mentor
source install/setup.bash

# (Optional) Auto-source on opening a new terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ar_ghost_mentor/ros2_ws/install/setup.bash" >> ~/.bashrc
```
# Workflow & Usage
STEP 1 - Record Veteran Movements
```
ros2 run ghost_mentor veteran_recorder --ros-args -p task_name:=screw_tightening -p save_dir:=/home/your_name/ar_ghost_mentor/data/raw/screw_tightening
```
Controls: [Space] to start/stop recording, [q] to save and exit.

STEP 2 - Generate the Master Ghost
```
# Multi Mode (Ensemble for production - Recommended)
python3 ~/ar_ghost_mentor/tools/ghost_builder.py --mode multi --input_dir ~/ar_ghost_mentor/data/raw/screw_tightening --output ~/ar_ghost_mentor/data/sequences/screw_tightening_master.json --task screw_tightening
```
STEP 3 - Run the AR Demonstration
```
ros2 launch ghost_mentor demo_launch.py ghost_path:=/home/rhkrgusdud/ar_ghost_mentor/data/sequences/screw_tightening_master.json
```
