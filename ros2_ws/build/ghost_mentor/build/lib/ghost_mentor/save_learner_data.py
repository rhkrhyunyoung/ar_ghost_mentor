import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
import numpy as np
from datetime import datetime

# NOTE: 이 스크립트는 Colab이 아닌, ROS 2가 설치된 로컬 환경에서 실행해야 합니다.
#       save_learner_data.py 파일로 저장하여 사용해주세요.

class LearnerDataSaver(Node):
    def __init__(self):
        super().__init__('learner_data_saver')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/learner_joint_data',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.get_logger().info('Subscribing to /learner_joint_data topic...')

        self.learner_frames = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9 # Record start time in seconds

    def listener_callback(self, msg):
        # Assuming Float32MultiArray is a flattened array of [x1, y1, z1, x2, y2, z2, ...]
        # We need to reshape it into a list of [x, y, z] tuples for each joint
        joint_data = np.array(msg.data).reshape(-1, 3).tolist()

        # Get current time relative to the start of recording
        current_time = self.get_clock().now().nanoseconds / 1e9
        relative_time = current_time - self.start_time

        # Create a dictionary for the current frame, similar to master ghost format
        frame_entry = {
            't': round(relative_time, 4), # Time in seconds, rounded for readability
            'joints': {}
        }

        # Assuming KEY_JOINTS from ar_overlay_node.py, add joint indices and xyz data
        # NOTE: If you need joint names, you would need to define KEY_JOINTS here as well
        # or modify ar_overlay_node to publish joint IDs/names.
        # For simplicity, we'll use a generic numbering (0 to len(joint_data)-1)
        # If you know the actual KEY_JOINTS (e.g., [11, 12, ...]), replace range() with that list.
        # For now, let's use dummy int indices to represent joints.
        for i, pos in enumerate(joint_data):
            frame_entry['joints'][str(i)] = {'x': pos[0], 'y': pos[1], 'z': pos[2]}

        self.learner_frames.append(frame_entry)
        # self.get_logger().info(f'Received frame at t={frame_entry["t"]:.2f}')

    def save_data_on_shutdown(self):
        if not self.learner_frames:
            self.get_logger().warn('No learner data collected. Not saving.')
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'learner_motion_data_{timestamp}.json'

        # Construct the final JSON structure, similar to the master ghost file
        output_data = {
            'task': 'learner_recording',
            'num_frames': len(self.learner_frames),
            'joints_order': [str(i) for i in range(len(self.learner_frames[0]['joints']))], # Infer joint order from first frame
            'description': 'Learner motion data recorded from ROS 2 topic',
            'frames': self.learner_frames
        }

        with open(filename, 'w') as f:
            json.dump(output_data, f, indent=4)
        self.get_logger().info(f'Learner motion data saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    learner_saver = LearnerDataSaver()
    try:
        rclpy.spin(learner_saver)
    except KeyboardInterrupt:
        learner_saver.get_logger().info('KeyboardInterrupt received, stopping...')
    finally:
        learner_saver.save_data_on_shutdown()
        learner_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
