"""
veteran_recorder.py
────────────────────────────────────────────────────────────
노트북 웹캠으로 베테랑 작업자의 동작을 녹화하고
관절 데이터를 ROS2 토픽으로 퍼블리시 + JSON 저장

실행:
    ros2 run ghost_mentor veteran_recorder --ros-args -p task_name:=screw_tightening
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import mediapipe as mp
import json
import time
import os
from datetime import datetime
from pathlib import Path


# MediaPipe에서 뽑을 핵심 관절 인덱스 (노이즈 많은 얼굴 제외)
KEY_JOINTS = {
    11: "left_shoulder",
    12: "right_shoulder",
    13: "left_elbow",
    14: "right_elbow",
    15: "left_wrist",
    16: "right_wrist",
    23: "left_hip",
    24: "right_hip",
    25: "left_knee",
    26: "right_knee",
    27: "left_ankle",
    28: "right_ankle",
}


class VeteranRecorder(Node):
    def __init__(self):
        super().__init__("veteran_recorder")

        # 파라미터
        self.declare_parameter("task_name", "default_task")
        self.declare_parameter("camera_index", 0)
        default_save = str(Path.home() / "ar_ghost_mentor" / "data" / "sequences")
        self.declare_parameter("save_dir", default_save)

        self.task_name = self.get_parameter("task_name").value
        cam_idx = self.get_parameter("camera_index").value
        self.save_dir = self.get_parameter("save_dir").value

        # ROS2 퍼블리셔
        self.pose_pub = self.create_publisher(String, "/veteran_pose", 10)
        self.img_pub = self.create_publisher(Image, "/veteran_image", 10)
        self.bridge = CvBridge()

        # MediaPipe Pose (정확도 최우선)
        self.mp_pose = mp.solutions.pose
        self.mp_draw = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=2,          # 0/1/2 중 가장 정확
            smooth_landmarks=True,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )

        # 웹캠 열기
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 녹화 상태
        self.recording = False
        self.frames: list[dict] = []
        self.start_time: float = 0.0

        # 타이머: 30Hz 처리
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info(
            f"VeteranRecorder 시작 — task: {self.task_name} | [Space] 녹화 시작/중지, [q] 저장 후 종료"
        )

    # ──────────────────────────────────────────────
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("카메라 프레임 읽기 실패")
            return

        frame = cv2.flip(frame, 1)  # 거울 모드
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.pose.process(rgb)

        joints = None
        if result.pose_landmarks:
            joints = self._extract_joints(result.pose_landmarks)
            self.mp_draw.draw_landmarks(
                frame,
                result.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_draw.DrawingSpec(
                    color=(0, 255, 180), thickness=2, circle_radius=3
                ),
            )

            if self.recording and joints:
                ts = time.time() - self.start_time
                frame_data = {"t": round(ts, 4), "joints": joints}
                self.frames.append(frame_data)
                self.pose_pub.publish(String(data=json.dumps(frame_data)))

        self._draw_ui(frame, joints)
        cv2.imshow("Veteran Recorder", frame)
        self._handle_keys()

    def _extract_joints(self, landmarks) -> dict:
        """MediaPipe 랜드마크 → {joint_id: [x, y, z, visibility]} 딕셔너리"""
        joints = {}
        for idx, name in KEY_JOINTS.items():
            lm = landmarks.landmark[idx]
            joints[str(idx)] = {
                "name": name,
                "x": round(lm.x, 5),   # 정규화 좌표 (0~1)
                "y": round(lm.y, 5),
                "z": round(lm.z, 5),   # MediaPipe 추정 상대깊이
                "vis": round(lm.visibility, 3),
            }
        return joints

    def _draw_ui(self, frame, joints):
        h, w = frame.shape[:2]
        color = (0, 60, 220) if self.recording else (60, 60, 60)
        status = f"● REC  {len(self.frames)} frames" if self.recording else "■ READY"
        cv2.rectangle(frame, (0, 0), (w, 40), color, -1)
        cv2.putText(frame, status, (12, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"task: {self.task_name}", (w - 280, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, "Space: rec/stop  |  q: save & quit",
                    (12, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    def _handle_keys(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            self.recording = not self.recording
            if self.recording:
                self.frames = []
                self.start_time = time.time()
                self.get_logger().info("녹화 시작")
            else:
                self.get_logger().info(f"녹화 일시정지 — {len(self.frames)} 프레임")
        elif key == ord("q"):
            self._save_and_quit()

    def _save_and_quit(self):
        if self.frames:
            os.makedirs(self.save_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.save_dir, f"{self.task_name}_{ts}.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(
                    {"task": self.task_name, "fps": 30, "frames": self.frames},
                    f, ensure_ascii=False, indent=2,
                )
            self.get_logger().info(f"저장 완료: {path}  ({len(self.frames)} 프레임)")
        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VeteranRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
