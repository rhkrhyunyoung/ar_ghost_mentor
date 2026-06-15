# ar_ghost_mentor

# AR Ghost Mentor

> **Visualizing the tacit knowledge of skilled veterans using an AR Ghost skeleton to bridge technical training and language barriers.**

---

## 📂 Directory Structure
ar_ghost_mentor/
│
├── ros2_ws/
│   └── src/
│       └── ghost_mentor/
│           ├── ghost_mentor/               ← ROS2 Node Package
│           │   ├── init.py
│           │   ├── veteran_recorder.py     # [Node] Records veteran movements
│           │   ├── match_engine.py         # [Library] Gesture matching & scoring algorithm
│           │   ├── ar_overlay_node.py      # [Node] Main AR overlay UI for learners
│           │   └── score_dashboard.py      # [Node] Terminal-based score dashboard
│           ├── launch/
│           │   └── demo_launch.py          # Launches the entire system at once
│           ├── resource/ghost_mentor
│           ├── package.xml
│           └── setup.py
│
├── tools/
│   └── ghost_builder.py                    # Preprocesses raw recordings into Master Ghost (No ROS2 required)
│
└── data/
├── raw/                                # Raw JSON recordings from veterans (Multiple files)
│   └── screw_tightening/
│       ├── screw_tightening_20240501_143022.json
│       ├── screw_tightening_20240501_143501.json
│       └── ...
└── sequences/                          # Processed Master Ghost JSON sequences
└── screw_tightening_master.json

# Navigate and build
cd ~/ar_ghost_mentor/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ghost_mentor
source install/setup.bash

# (Optional) Auto-source on opening a new terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ar_ghost_mentor/ros2_ws/install/setup.bash" >> ~/.bashrc

# STEP 1 — Record Veteran Movements

It is highly recommended to record multiple times (10 to 30+ iterations) to build a stable and robust Master Ghost.
ros2 run ghost_mentor veteran_recorder \
    --ros-args -p task_name:=screw_tightening \
               -p save_dir:=/home/rhkrgusdud/ar_ghost_mentor/data/raw/screw_tightening

# Controls:
#   [Space] → Start / Stop Recording
#   [q]     → Save and Exit
# Execute multiple times to stack raw data in the raw/ directory.

# STEP 2 — Generate the Master Ghost
# Single Mode (For quick testing)
python3 ~/ar_ghost_mentor/tools/ghost_builder.py \
    --mode single \
    --input  ~/ar_ghost_mentor/data/raw/screw_tightening/YOUR_FILE_NAME.json \
    --output ~/ar_ghost_mentor/data/sequences/screw_tightening_master.json

# Multi Mode (Ensemble for production — Recommended)
python3 ~/ar_ghost_mentor/tools/ghost_builder.py \
    --mode multi \
    --input_dir ~/ar_ghost_mentor/data/raw/screw_tightening \
    --output    ~/ar_ghost_mentor/data/sequences/screw_tightening_master.json \
    --task      screw_tightening

# STEP 3 — Run the AR Demonstration
# Option A: Run individual nodes manually
# Terminal 1 — AR Main Overlay
ros2 run ghost_mentor ar_overlay_node \
    --ros-args -p ghost_path:=/home/rhkrgusdud/ar_ghost_mentor/data/sequences/screw_tightening_master.json \
               -p threshold:=87.0

# Terminal 2 — Dashboard Display
ros2 run ghost_mentor score_dashboard

# Option B: Run everything at once via Launch File (Recommended)
ros2 launch ghost_mentor demo_launch.py \
    ghost_path:=/home/rhkrgusdud/ar_ghost_mentor/data/sequences/screw_tightening_master.json

# UI & Display Layout
┌─────────────────────────────────────────────────────┐
│  [Ghost: 14/42] ━━━━░░░   ← Top-Right: Ghost Progress Bar   │
│                                                     │
│   Cyan Translucent Skeleton → Veteran Ghost         │
│   Golden Skeleton          → Learner's Real-time Pose│
│   Blue Arrows              → Correction Direction   │
│                                                     │
│         [NEXT] Preview    ← Bottom-Right: Upcoming Gesture  │
│                                                     │
│  72.4% [━━━━━░│90%░░]    ← Bottom: Similarity Match Bar     │
│  Red(<80%) / Yellow(80~90%) / Green(90%+)          │
└─────────────────────────────────────────────────────┘
  SUCCESS ✓ NEXT STEP     ← Visible only when similarity >= 90%

# ROS2 Architecture & Topics
[veteran_recorder] ──/veteran_pose──▶ (Save to File)
                   ──/veteran_image──▶ (Save to File)
                         │
                         ▼
                   ghost_builder.py (Preprocessing)
                         │
                         ▼
                   ghost_master.json

[ar_overlay_node] ◀── ghost_master.json (Direct Load)
       │
       ├──▶ /match_score    (Float32)  Current similarity percentage
       ├──▶ /process_passed (Bool)     Whether threshold (90%) is met
       └──▶ /ar_status      (String)   Status details in JSON format
                 │
                 ▼
       [score_dashboard]               Terminal output
