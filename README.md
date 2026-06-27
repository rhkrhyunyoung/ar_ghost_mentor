# 🤖 Industrial "Just Dance": AR Ghost Mentor

Externalizing Tacit Knowledge via AR-Driven Skill Transfer & Real-time Biometric Alignment

![alt text](https://img.shields.io/badge/Competition-Engineering_Industry_Design_Contest-blue?style=for-the-badge)


![alt text](https://img.shields.io/badge/ROS2-Humble-0A0FF9?style=for-the-badge&logo=ros&logoColor=white)


![alt text](https://img.shields.io/badge/python-3.10+-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

"Human-Centric Engineering for the Next Industrial Era."
AR Ghost Mentor bridges the gap between veteran expertise and novice execution. By transforming the "tacit knowledge" of skilled workers into "explicit digital data," this system enables intuitive, language-barrier-free training using AR skeleton overlays and high-precision motion matching.

# Tech Stack & Environment
- OS: Ubuntu 22.04 LTS
- Framework: ROS 2 (Humble)
- Language: Python 3
- Libraries: MediaPipe, OpenCV, NumPy
- Hardware: RGB-D Camera / Webcam, Terminal Dashboard Display

# Vision & Motivation
-The Labor Gap: Addressing the crisis of an aging skilled workforce and the linguistic barriers of migrant workers.
-Frugal Engineering: Delivering high-performance industrial training using accessible hardware (Webcams/Smartphones) instead of multi-million dollar robotics.
-Safety First: Reducing industrial accidents by ensuring precise posture and workflow adherence in high-risk environments.

# Key Modules & Architecture

1. Spatial Invariance (Dynamic Skeleton Normalization)
To ensure consistent matching regardless of the user's height or distance from the camera:
-Origin-Centering: Resets the coordinate system based on the Hip Center.
-Adaptive Scaling: Normalizes joint vectors based on the Inter-shoulder Width, allowing for universal comparison across different body types.

2. Precision-Weighted Cosine Similarity
Not all joints are equal. Our algorithm assigns dynamic weights to specific bone segments based on the task's criticality:
-High-Precision Focus: Wrists (2.5x) and Elbows (2.0x) are weighted more heavily for intricate tasks like screw tightening.
-Stability: Ensures the scoring system reflects actual work quality rather than insignificant torso movements.

3. Real-time Temporal Alignment (DTW Window Search)
To accommodate different working tempos between masters and learners:
-Implements a Sliding Window DTW (Dynamic Time Warping) algorithm.
-Supports a ±10 frame unidirectional search, allowing the system to maintain synchronization even if the learner is slightly faster or slower than the expert.

4. Ensemble Ghost Building
Instead of relying on a single recording, the system aggregates multiple expert performances:
-Noise Reduction: Filters out outlier movements.
-Gold Standard Generation: Creates a robust, "stable" master sequence by averaging expert data, resulting in higher reliability.

# System Architecture
graph TD
    A[Expert/Veteran] -->|Action Capture| B(Multi-Iteration Ensemble)
    B -->|Master Ghost JSON| C{Matching Engine}
    D[Learner/User] -->|Real-time Pose| C
    C -->|Weighted Cosine + DTW| E[Visual Feedback Loop]
    E -->|Success / Retry / Correction| D
    F[ROS 2 Node Network] --- C

# Project Structure
```
├── ros2_ws/src/ghost_mentor/
│   ├── ghost_mentor/               
│   │   ├── veteran_recorder.py     # [Node] Captures master motion sequences
│   │   ├── match_engine.py         # [Lib] Core logic: DTW & Weighted Similarity
│   │   ├── ar_overlay_node.py      # [Node] Main AR UI (MediaPipe + OpenCV)
│   │   └── score_dashboard.py      # [Node] Live telemetry & performance monitoring
│   └── launch/
│       └── demo_launch.py          # Full-stack system orchestration
├── tools/
│   └── ghost_builder.py            # Data pipeline for Ensemble Master Ghost generation
└── data/
    ├── raw/                        # Expert raw data (Pre-processing)
    └── sequences/                  # Production-ready Ghost JSONs
```

# Installation & Setup
Prerequisites & Dependencies
```
# Install Dependencies
pip install mediapipe==0.10.9 opencv-python==4.8.1.78 "numpy<2" --break-system-packages --no-deps
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
STEP 1 - Knowledge Externalization (Expert Recording)
```
ros2 run ghost_mentor veteran_recorder --ros-args -p task_name:=screw_tightening -p save_dir:=/home/your_name/ar_ghost_mentor/data/raw/screw_tightening
```
Controls: [Space] to start/stop recording, [q] to save and exit.

STEP 2 - Data Refinement (Ensemble Processing)
```
# Multi Mode (Ensemble for production - Recommended)
python3 ~/ar_ghost_mentor/tools/ghost_builder.py --mode multi --input_dir ~/ar_ghost_mentor/data/raw/screw_tightening --output ~/ar_ghost_mentor/data/sequences/screw_tightening_master.json --task screw_tightening
```
STEP 3 - Real-time Guidance (Learner Mode)
```
ros2 launch ghost_mentor demo_launch.py ghost_path:=/home/rhkrgusdud/ar_ghost_mentor/data/sequences/screw_tightening_master.json
```
