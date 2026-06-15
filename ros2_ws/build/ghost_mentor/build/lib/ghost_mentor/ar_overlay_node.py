import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Float32MultiArray # Float32MultiArray 추가

import cv2
import mediapipe as mp
import numpy as np
import json
import time

from ghost_mentor.match_engine import MatchEngine, KEY_JOINTS


# ── 색상 상수 ──────────────────────────────────────────────
COLOR_GHOST = (0, 220, 180)   # 청록 — Ghost 골격
COLOR_LEARNER = (220, 180, 0)   # 황금 — 학습자 골격
COLOR_ARROW = (0, 80, 255)    # 파랑 — 보정 화살표
COLOR_SUCCESS = (50, 220, 50)   # 초록 — SUCCESS
COLOR_FAIL = (30, 30, 220)   # 빨강 — 재시도

# 뼈대 연결 — MediaPipe 관절 번호 기준 (상체만)
BONES_MP = [
    (11, 13), (13, 15),   # 왼팔
    (12, 14), (14, 16),   # 오른팔
    (11, 12),             # 어깨
    (11, 23), (12, 24),   # 몸통
    (23, 24),             # 골반
]

# Ghost 드로잉 — KEY_JOINTS 배열 인덱스 기준
# KEY_JOINTS = [11,12,13,14,15,16,23,24,25,26,27,28]
#               0   1  2  3  4  5  6  7  8  9 10 11
BONES_GHOST = [
    (0, 2), (2, 4),   # 왼팔
    (1, 3), (3, 5),   # 오른팔
    (0, 1),           # 어깨
    (0, 6), (1, 7),   # 몸통
    (6, 7),           # 골반
]


class AROverlayNode(Node):
    def __init__(self):
        super().__init__("ar_overlay_node")

        self.declare_parameter("ghost_path", "")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("threshold", 80.0)

        self.processes = []
        self.current_process = 0

        ghost_path = self.get_parameter("ghost_path").value
        cam_idx    = self.get_parameter("camera_index").value
        threshold  = self.get_parameter("threshold").value

        # 퍼블리셔
        self.score_pub  = self.create_publisher(Float32, "/match_score", 10)
        self.passed_pub = self.create_publisher(Bool,    "/process_passed", 10)
        self.status_pub = self.create_publisher(String,  "/ar_status", 10)

        # 새로운 발행자 추가: 학습자 관절 3D 데이터를 위한 토픽
        self.learner_joint_data_pub = self.create_publisher(Float32MultiArray, "/learner_joint_data", 10)

        # 판정 엔진
        self.engine = MatchEngine(threshold=threshold)
        if ghost_path:
            self.engine.load_ghost(ghost_path)
        else:
            self.get_logger().warn("ghost_path가 지정되지 않음 — Ghost 없이 학습자 골격만 표시")

        # MediaPipe
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )

        # Ghost 시퀀스 캐시
        self.ghost_seq: np.ndarray | None = (
            self.engine.ghost_seq if ghost_path else None
        )

        # 웹캠
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 상태
        self.last_score: float = 0.0
        self.success_flash: float = 0.0

        # 30Hz 루프
        self.timer = self.create_timer(1.0 / 30.0, self.loop)
        self.get_logger().info("AROverlayNode 시작 — 'q' 로 종료")

    def load_manifest(self, manifest_path):
        with open(manifest_path) as f:
            data = json.load(f)
        for p in data["processes"]:
            engine = MatchEngine(threshold=90.0)
            engine.load_ghost(p["file"])
            self.processes.append({"name": p["name"], "engine": engine})
        self.engine = self.processes[0]["engine"]

    def next_process(self):
        """현재 공정 90% 달성 → 다음 공정으로"""
        self.current_process += 1
        if self.current_process < len(self.processes):
            self.engine = self.processes[self.current_process]["engine"]
            self.engine.reset()


    # ──────────────────────────────────────────────────────
    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        rgb    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.pose.process(rgb)

        learner_joints = None
        if result.pose_landmarks:
            # Extract 3D World Landmarks for publishing
            learner_joints_world = np.zeros((len(KEY_JOINTS), 3))
            if result.pose_world_landmarks:
                for i, mp_idx in enumerate(KEY_JOINTS):
                    landmark = result.pose_world_landmarks.landmark[mp_idx] # Use world_landmarks for 3D
                    learner_joints_world[i] = [landmark.x, landmark.y, landmark.z]

                # Publish learner joint data
                msg = Float32MultiArray()
                msg.data = learner_joints_world.flatten().tolist()
                self.learner_joint_data_pub.publish(msg)

            # Original drawing and other processing (using pixel coordinates for AR overlay)
            learner_joints = self._landmarks_to_array(result.pose_landmarks, w, h)
            self._last_learner_joints = learner_joints
            self._draw_skeleton(frame, result.pose_landmarks, w, h,
                                color=COLOR_LEARNER, thickness=2)

        # Ghost 오버레이 + 판정
        if self.ghost_seq is not None and learner_joints is not None:
            ghost_frame_idx = self.engine.current_ghost_frame
            ghost_lm = self.ghost_seq[ghost_frame_idx]
            self._draw_ghost_skeleton(frame, ghost_lm, w, h)

            normed_learner = self._normalize_array(learner_joints)
            normed_ghost   = self._normalize_array(ghost_lm[:, :3])

            match = self.engine.evaluate(normed_learner)
            self.last_score = match.score

            # 보정 화살표 그리기
            if not match.passed:
                self._draw_corrections(frame, learner_joints,
                                       match.corrections, w, h)

            # 성공 플래시
            if match.passed:
                self.success_flash = time.time()

            # 퍼블리시
            self.score_pub.publish(Float32(data=match.score))
            self.passed_pub.publish(Bool(data=match.passed))
            self.status_pub.publish(
                String(data=json.dumps({
                    "score": match.score,
                    "passed": match.passed,
                    "ghost_frame": match.closest_frame,
                }))
            )

        self._draw_hud(frame, w, h)
        cv2.imshow("AR Ghost Mentor", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    # ──────────────────────────────────────────────────────
    def _landmarks_to_array(self, landmarks, w: int, h: int) -> np.ndarray:
        """랜드마크 → (J, 4) 픽셀+깊이 배열"""
        arr = np.zeros((len(KEY_JOINTS), 4), dtype=np.float32)
        for j, idx in enumerate(KEY_JOINTS):
            lm = landmarks.landmark[idx]
            arr[j] = [lm.x * w, lm.y * h, lm.z, lm.visibility]
        return arr

    def _normalize_array(self, arr: np.ndarray) -> np.ndarray:
        """픽셀 좌표 → 골반 중심 정규화 (match_engine 입력용)"""
        normed = np.copy(arr)
        if arr.ndim == 2 and arr.shape[1] >= 3:
            hip_l, hip_r = arr[6, :3], arr[7, :3]
            center = (hip_l + hip_r) / 2.0
            normed[:, :3] -= center
            shoulder_w = np.linalg.norm(arr[0, :3] - arr[1, :3])
            if shoulder_w > 1e-4:
                normed[:, :3] /= shoulder_w
        return normed

    # ──────────────────────────────────────────────────────
    def _draw_skeleton(self, frame, landmarks, w, h, color, thickness=2):
        for a_idx, b_idx in BONES_MP:
            la = landmarks.landmark[a_idx]
            lb = landmarks.landmark[b_idx]
            if la.visibility < 0.4 or lb.visibility < 0.4:
                continue
            pt_a = (int(la.x * w), int(la.y * h))
            pt_b = (int(lb.x * w), int(lb.y * h))
            cv2.line(frame, pt_a, pt_b, color, thickness)
        # 관절 점
        for idx in KEY_JOINTS:
            lm = landmarks.landmark[idx]
            if lm.visibility < 0.4:
                continue
            pt = (int(lm.x * w), int(lm.y * h))
            cv2.circle(frame, pt, 5, color, -1)

    def _draw_ghost_skeleton(self, frame, ghost_frame: np.ndarray, w: int, h: int):
        """
        학습자 골격의 어깨 중심에 Ghost를 정렬해서 오버레이
        → 학습자 위치에 Ghost가 따라붙음
        """
        if self.ghost_seq is None or not hasattr(self, '_last_learner_joints'):
            return

        lj = self._last_learner_joints

        anchor_x = int((lj[0, 0] + lj[1, 0]) / 2.0)
        anchor_y = int((lj[0, 1] + lj[1, 1]) / 2.0)

        g_anchor_x = (ghost_frame[0, 0] + ghost_frame[1, 0]) / 2.0
        g_anchor_y = (ghost_frame[0, 1] + ghost_frame[1, 1]) / 2.0

        learner_shoulder_w = max(abs(lj[0, 0] - lj[1, 0]), 1.0)
        ghost_shoulder_w = max(abs(ghost_frame[0, 0] - ghost_frame[1, 0]), 1e-4)
        scale = learner_shoulder_w / ghost_shoulder_w

        def to_px(j: int):
            x = int(anchor_x + (ghost_frame[j, 0] - g_anchor_x) * scale)
            y = int(anchor_y + (ghost_frame[j, 1] - g_anchor_y) * scale)
            return (x, y)

        overlay = frame.copy()
        for a, b in BONES_GHOST:
            cv2.line(overlay, to_px(a), to_px(b), COLOR_GHOST, 3)
        for j in range(len(KEY_JOINTS)):
            cv2.circle(overlay, to_px(j), 6, COLOR_GHOST, -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
    # ──────────────────────────────────────────────────────
    def _draw_corrections(self, frame, learner: np.ndarray,
                           corrections: dict, w: int, h: int):
        for joint_id, diff in corrections.items():
            j = KEY_JOINTS.index(joint_id) if joint_id in KEY_JOINTS else -1
            if j < 0:
                continue
            px = int(learner[j, 0])
            py = int(learner[j, 1])
            scale = w * 0.12
            ex = int(px + diff[0] * scale)
            ey = int(py + diff[1] * scale)
            cv2.arrowedLine(frame, (px, py), (ex, ey),
                            COLOR_ARROW, 3, tipLength=0.4)
    def _draw_next_preview(self, frame, w: int, h: int):
        """우하단에 다음 Ghost 프레임 미리보기"""
        if self.ghost_seq is None:
            return
        next_idx = min(self.engine.current_ghost_frame + 8, len(self.ghost_seq) - 1)
        next_frame = self.ghost_seq[next_idx]
        pw, ph = 200, 150
        px, py = w - pw - 16, h - ph - 70
        overlay = frame.copy()
        cv2.rectangle(overlay, (px, py), (px+pw, py+ph), (15, 15, 25), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.rectangle(frame, (px, py), (px+pw, py+ph), (80, 80, 100), 1)
        cv2.putText(frame, "NEXT", (px+8, py+18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 200), 1)
        cx = px + pw // 2
        cy = py + ph // 2 + 10
        scale = pw * 0.28
        hip_cx = (next_frame[6, 0] + next_frame[7, 0]) / 2.0
        hip_cy = (next_frame[6, 1] + next_frame[7, 1]) / 2.0
        def to_px_prev(j):
            x = int(cx + (next_frame[j, 0] - hip_cx) * scale)
            y = int(cy + (next_frame[j, 1] - hip_cy) * scale)
            return (x, y)
        prev_overlay = frame.copy()
        for a, b in BONES_GHOST:
            cv2.line(prev_overlay, to_px_prev(a), to_px_prev(b), (100, 200, 160), 2)
        for j in range(len(KEY_JOINTS)):
           cv2.circle(prev_overlay, to_px_prev(j), 3, (100, 200, 160), -1)
        cv2.addWeighted(prev_overlay, 0.7, frame, 0.3, 0, frame)

    def _draw_hud(self, frame, w: int, h: int):
        """점수 바 + 상태 텍스트 HUD"""
        # 점수 바 배경
        cv2.rectangle(frame, (0, h - 60), (w, h), (20, 20, 20), -1)

        # 점수 바 채우기
        bar_w = int((self.last_score / 100.0) * (w - 40))
        if self.last_score >= 87:
            bar_color = (50, 220, 50)    # 초록
        elif self.last_score >= 80:
            bar_color = (0, 200, 255)    # 노랑
        else:
            bar_color = (50, 50, 220)    # 빨강
        cv2.rectangle(frame, (20, h - 44), (20 + bar_w, h - 20), bar_color, -1)
        cv2.rectangle(frame, (20, h - 44), (w - 20, h - 20), (100, 100, 100), 1)

        # 80% 기준선
        threshold_x = int(20 + 0.9 * (w - 40))
        cv2.line(frame, (threshold_x, h - 48), (threshold_x, h - 16),
                 (255, 255, 100), 2)
        cv2.putText(frame, "87%", (threshold_x - 16, h - 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 100), 1)

        # 점수 텍스트
        cv2.putText(frame, f"{self.last_score:.1f}%",
                    (24, h - 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)

        # SUCCESS 표시 — 현재 점수가 87% 이상일 때만 표시 (점수 떨어지면 즉시 사라짐)
        if self.last_score >= 87.0:
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), COLOR_SUCCESS, -1)
            cv2.addWeighted(overlay, 0.15, frame, 0.87, 0, frame)
            cv2.putText(frame, "SUCCESS  ✓  NEXT STEP",
                        (w // 2 - 200, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4,
                        COLOR_SUCCESS, 3)

        # Ghost 프레임 진행도
        if self.ghost_seq is not None:
            progress = self.engine.current_ghost_frame / max(len(self.ghost_seq) - 1, 1)
            cv2.putText(
                frame,
                f"Ghost: {self.engine.current_ghost_frame}/{len(self.ghost_seq)}",
                (w - 220, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1,
            )
            cv2.rectangle(frame, (w - 220, 40), (w - 20, 50), (80, 80, 80), -1)
            cv2.rectangle(frame, (w - 220, 40),
                          (int(w - 220 + 200 * progress), 50),
                          COLOR_GHOST, -1)
        """우하단에 다음 Ghost 프레임 미리보기"""
        if self.ghost_seq is None:
            return
    
        # 다음 프레임 인덱스 (현재 + 미리보기 offset)
        next_idx = min(self.engine.current_ghost_frame + 8, len(self.ghost_seq) - 1)  # 마지막 공정이면 표시 안 함
    
        next_frame = self.ghost_seq[next_idx]
    
        # 미리보기 박스 크기·위치 (우하단)
        pw, ph = 200, 150
        px, py = w - pw - 16, h - ph - 70  # 점수바 위
    
        # 반투명 배경
        overlay = frame.copy()
        cv2.rectangle(overlay, (px, py), (px+pw, py+ph), (15, 15, 25), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.rectangle(frame, (px, py), (px+pw, py+ph), (80, 80, 100), 1)
    
        # "NEXT" 라벨
        cv2.putText(frame, "NEXT", (px+8, py+18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 200), 1)
    
        # 미리보기 Ghost 골격 그리기
        cx = px + pw // 2
        cy = py + ph // 2 + 10
        scale = pw * 0.28
    
        hip_cx = (next_frame[6, 0] + next_frame[7, 0]) / 2.0
        hip_cy = (next_frame[6, 1] + next_frame[7, 1]) / 2.0
    
        def to_px_prev(j):
            x = int(cx + (next_frame[j, 0] - hip_cx) * scale)
            y = int(cy + (next_frame[j, 1] - hip_cy) * scale)
            return (x, y)
    
        prev_overlay = frame.copy()
        for a, b in BONES_GHOST:
            cv2.line(prev_overlay, to_px_prev(a), to_px_prev(b), (100, 200, 160), 2)
        for j in range(len(KEY_JOINTS)):
           cv2.circle(prev_overlay, to_px_prev(j), 3, (100, 200, 160), -1)
        cv2.addWeighted(prev_overlay, 0.7, frame, 0.3, 0, frame)
        self._draw_next_preview(frame, w, h)


def main(args=None):
    rclpy.init(args=args)
    node = AROverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


