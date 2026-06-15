import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rhkrgusdud/ar_ghost_mentor/ros2_ws/install/ghost_mentor'
