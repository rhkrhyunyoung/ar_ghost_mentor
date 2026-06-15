"""
match_engine.py
────────────────────────────────────────────────────────────
Ghost 시퀀스와 학습자 현재 자세를 비교해
일치도(0~100%)와 보정 화살표 데이터를 반환하는 핵심 모듈

비교 방식: 뼈대 벡터(bone vector) 각도 차이 기반
→ 코사인 유사도 대비 포즈 민감도 훨씬 높음

다른 노드에서 import해서 사용:
    from ghost_mentor.match_engine import MatchEngine
"""

import numpy as np
from dataclasses import dataclass

KEY_JOINTS = [11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]

# (parent, child) 뼈대 쌍 — KEY_JOINTS 내 인덱스 기준
BONES = [
    (0, 2),   # left_shoulder  → left_elbow
    (2, 4),   # left_elbow     → left_wrist
    (1, 3),   # right_shoulder → right_elbow
    (3, 5),   # right_elbow    → right_wrist
    (0, 1),   # left_shoulder  → right_shoulder
    (0, 6),   # left_shoulder  → left_hip
    (1, 7),   # right_shoulder → right_hip
    (6, 8),   # left_hip       → left_knee
    (7, 9),   # right_hip      → right_knee
    (8, 10),  # left_knee      → left_ankle
    (9, 11),  # right_knee     → right_ankle
]

# 뼈대별 가중치 (손목 쪽 뼈대를 더 중요하게)
BONE_WEIGHTS = [2.0, 2.5, 2.0, 2.5, 1.0, 0.8, 0.8, 0.5, 0.5, 0.3, 0.3]


@dataclass
class MatchResult:
    score: float                         # 0.0 ~ 100.0
    passed: bool                         # score >= threshold
    corrections: dict[int, list[float]]  # {joint_idx: [dx, dy, dz]} 보정 벡터
    closest_frame: int = 0


class MatchEngine:
    def __init__(self, threshold: float = 80.0):
        self.threshold = threshold
        self.ghost_seq: np.ndarray | None = None  # shape (T, J, 4)
        self.current_ghost_frame: int = 0
        self._window: int = 10

    # ──────────────────────────────────────────────
    def load_ghost(self, path: str):
        import json
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        raw = np.array(data["frames"], dtype=np.float32)  # (T, J, 4)
        # Ghost는 로드 시 한 번만 정규화
        self.ghost_seq = np.stack([self._normalize(raw[t]) for t in range(len(raw))])
        self.current_ghost_frame = 0
        print(f"[MatchEngine] Ghost 로드: {path}  ({len(self.ghost_seq)} frames)")

    # ──────────────────────────────────────────────
    def evaluate(self, learner_raw: np.ndarray) -> MatchResult:
        """
        learner_raw: shape (J, 4) — 픽셀 좌표 그대로 (정규화 전)
        """
        if self.ghost_seq is None:
            raise RuntimeError("ghost를 먼저 load_ghost()로 불러오세요")

        learner_normed = self._normalize(learner_raw)

        best_score = -1.0
        best_frame = self.current_ghost_frame
        lo = max(0, self.current_ghost_frame - self._window)
        hi = min(len(self.ghost_seq), self.current_ghost_frame + self._window + 1)

        for t in range(lo, hi):
            s = self._angle_similarity(learner_normed, self.ghost_seq[t])
            if s > best_score:
                best_score = s
                best_frame = t

        if best_frame > self.current_ghost_frame:
            self.current_ghost_frame = best_frame

        score = float(best_score * 100.0)
        corrections = self._compute_corrections(learner_normed, self.ghost_seq[best_frame])

        return MatchResult(
            score=round(score, 1),
            passed=(score >= self.threshold),
            corrections=corrections,
            closest_frame=best_frame,
        )

    # ──────────────────────────────────────────────
    def _normalize(self, joints: np.ndarray) -> np.ndarray:
        """
        골반 중심 원점 + 어깨 너비 스케일 정규화
        joints: (J, 4)  →  반환: (J, 4) xyz 정규화, vis 유지
        """
        normed = joints.copy()
        hip_l, hip_r = joints[6, :3], joints[7, :3]
        center = (hip_l + hip_r) / 2.0
        normed[:, :3] -= center

        shoulder_w = np.linalg.norm(joints[0, :3] - joints[1, :3])
        if shoulder_w > 1e-4:
            normed[:, :3] /= shoulder_w
        return normed

    # ──────────────────────────────────────────────
    def _bone_vectors(self, joints: np.ndarray) -> np.ndarray:
        """
        각 뼈대의 방향 단위벡터 계산
        반환: (len(BONES), 3)
        """
        vecs = []
        for p, c in BONES:
            v = joints[c, :3] - joints[p, :3]
            n = np.linalg.norm(v)
            vecs.append(v / n if n > 1e-6 else np.zeros(3))
        return np.array(vecs, dtype=np.float32)

    # ──────────────────────────────────────────────
    def _angle_similarity(self, a: np.ndarray, b: np.ndarray) -> float:
        """
        뼈대 벡터 간 각도 차이로 유사도 계산
        각 뼈대: cos(θ) = dot(va, vb)  →  θ=0° → 유사도 1.0, θ=180° → 0.0
        가중 평균으로 최종 점수 산출
        """
        va = self._bone_vectors(a)   # (B, 3)
        vb = self._bone_vectors(b)   # (B, 3)
        weights = np.array(BONE_WEIGHTS, dtype=np.float32)

        # 가시성 낮은 관절 포함 뼈대 가중치 감소
        for i, (p, c) in enumerate(BONES):
            vis = min(a[p, 3], a[c, 3], b[p, 3], b[c, 3])
            weights[i] *= max(vis, 0.2)

        # cos(θ) 계산 — 이미 단위벡터라 dot product = cos
        cos_angles = np.clip(np.sum(va * vb, axis=1), -1.0, 1.0)

        # cos → 유사도 (0~1): (cos+1)/2  →  0°=1.0, 90°=0.5, 180°=0.0
        similarities = (cos_angles + 1.0) / 2.0

        w_sum = weights.sum()
        if w_sum < 1e-8:
            return 0.0
        return float(np.dot(similarities, weights) / w_sum)

    # ──────────────────────────────────────────────
    def _compute_corrections(
        self,
        learner: np.ndarray,
        ghost: np.ndarray,
        angle_thresh: float = 0.3,  # cos 차이 임계값 (약 30° 이상만 표시)
    ) -> dict[int, list[float]]:
        """보정이 필요한 관절의 {joint_id: [dx, dy, dz]} 반환"""
        va = self._bone_vectors(learner)
        vb = self._bone_vectors(ghost)
        corrections = {}

        for i, (p, c) in enumerate(BONES):
            cos_diff = abs(np.dot(va[i], vb[i]) - 1.0)  # 0=일치, 2=정반대
            if cos_diff > angle_thresh:
                # child 관절을 ghost 방향으로 당기는 벡터
                diff = ghost[c, :3] - learner[c, :3]
                corrections[KEY_JOINTS[c]] = diff.tolist()
        return corrections

    # ──────────────────────────────────────────────
    def reset(self):
        self.current_ghost_frame = 0
