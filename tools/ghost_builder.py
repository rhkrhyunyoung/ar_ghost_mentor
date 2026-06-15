"""
ghost_builder.py
────────────────────────────────────────────────────────────
녹화된 JSON 시퀀스를 불러와 노이즈 제거 + 구간 분절 후
'Ghost 마스터 모델'로 저장하는 전처리 도구

실행 (ROS2 불필요 — 독립 실행):
    python3 tools/ghost_builder.py \
        --input  data/sequences/screw_tightening_20240501_143022.json \
        --output data/sequences/screw_tightening_master.json
"""

import json
import argparse
import numpy as np
from pathlib import Path


KEY_JOINTS = [11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]


# ──────────────────────────────────────────────────────────
# 1. 로드
# ──────────────────────────────────────────────────────────
def load_sequence(path: str) -> list[dict]:
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    print(f"[load] task={data['task']}  frames={len(data['frames'])}")
    return data["frames"], data.get("task", "unknown")


# ──────────────────────────────────────────────────────────
# 2. 프레임 → numpy 배열
# ──────────────────────────────────────────────────────────
def frames_to_array(frames: list[dict]) -> np.ndarray:
    """
    반환: shape (T, J, 4)  — T=프레임수, J=관절수, 4=[x,y,z,vis]
    """
    T = len(frames)
    J = len(KEY_JOINTS)
    arr = np.zeros((T, J, 4), dtype=np.float32)

    for t, frame in enumerate(frames):
        joints = frame["joints"]
        for j, idx in enumerate(KEY_JOINTS):
            jd = joints.get(str(idx), {})
            arr[t, j, 0] = jd.get("x", 0.0)
            arr[t, j, 1] = jd.get("y", 0.0)
            arr[t, j, 2] = jd.get("z", 0.0)
            arr[t, j, 3] = jd.get("vis", 0.0)
    return arr


# ──────────────────────────────────────────────────────────
# 3. 이동 평균 스무딩 (노이즈 제거)
# ──────────────────────────────────────────────────────────
def smooth(arr: np.ndarray, window: int = 5) -> np.ndarray:
    """시간축(axis=0)에 이동 평균 적용"""
    smoothed = np.copy(arr)
    half = window // 2
    for t in range(len(arr)):
        lo = max(0, t - half)
        hi = min(len(arr), t + half + 1)
        smoothed[t] = arr[lo:hi].mean(axis=0)
    return smoothed


# ──────────────────────────────────────────────────────────
# 4. 정규화 (골반 중심 기준, 스케일 불변)
# ──────────────────────────────────────────────────────────
def normalize(arr: np.ndarray) -> np.ndarray:
    """
    - 골반 중심(left_hip=index 6, right_hip=index 7)을 원점으로 이동
    - 어깨 너비로 나눠 스케일 정규화 → 키/거리 무관하게 비교 가능
    """
    normed = np.copy(arr)
    HIP_L, HIP_R = 6, 7      # KEY_JOINTS에서의 인덱스
    SHLDR_L, SHLDR_R = 0, 1

    for t in range(len(arr)):
        center = (arr[t, HIP_L, :3] + arr[t, HIP_R, :3]) / 2.0
        normed[t, :, :3] -= center

        shoulder_width = np.linalg.norm(
            arr[t, SHLDR_L, :3] - arr[t, SHLDR_R, :3]
        )
        if shoulder_width > 1e-6:
            normed[t, :, :3] /= shoulder_width

    return normed


# ──────────────────────────────────────────────────────────
# 5. 저장
# ──────────────────────────────────────────────────────────
def save_master(arr: np.ndarray, task: str, output: str):
    """정제된 시퀀스를 JSON으로 저장 (numpy → list 직렬화)"""
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    master = {
        "task": task,
        "num_frames": len(arr),
        "joints_order": KEY_JOINTS,
        "description": "정규화·스무딩된 Ghost 마스터 시퀀스",
        "frames": arr.tolist(),   # shape: (T, J, 4)
    }
    with open(output, "w", encoding="utf-8") as f:
        json.dump(master, f, ensure_ascii=False)
    print(f"[save] Ghost 마스터 저장: {output}  ({len(arr)} frames)")


# ──────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────
def build(input_path: str, output_path: str):
    frames, task = load_sequence(input_path)

    print("[build] numpy 변환 중...")
    arr = frames_to_array(frames)

    print("[build] 스무딩 (window=5)...")
    arr = smooth(arr, window=5)

    print("[build] 정규화 (골반 중심, 어깨 너비)...")
    arr = normalize(arr)

    save_master(arr, task, output_path)
    print("[build] 완료!")




# ──────────────────────────────────────────────────────────
# 6. 다중 영상 앙상블
# ──────────────────────────────────────────────────────────
def build_from_multiple(input_dir: str, output_path: str, task: str):
    """
    여러 녹화 JSON을 median 앙상블 → 강건한 Ghost 마스터 생성
    최소 10개, 권장 30개 이상
    """
    import glob
    files = sorted(glob.glob(f"{input_dir}/*.json"))
    if not files:
        print(f"[ERROR] {input_dir} 에 JSON 파일 없음")
        return

    print(f"[multi] {len(files)}개 영상 발견")
    all_arrays = []
    for f in files:
        try:
            frames_data, _ = load_sequence(f)
            arr = frames_to_array(frames_data)
            arr = smooth(arr, window=5)
            arr = normalize(arr)
            all_arrays.append(arr)
            print(f"  OK {f.split('/')[-1]}  ({len(arr)} frames)")
        except Exception as e:
            print(f"  SKIP {f.split('/')[-1]}: {e}")

    if not all_arrays:
        print("[ERROR] 유효한 파일 없음")
        return

    min_len = min(len(a) for a in all_arrays)
    print(f"[multi] 공통 길이: {min_len} frames")
    trimmed = np.stack([a[:min_len] for a in all_arrays], axis=0)
    master = np.median(trimmed, axis=0).astype(np.float32)
    print(f"[multi] {len(all_arrays)}개 median 앙상블 완료")
    save_master(master, task, output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ghost 마스터 모델 빌더")
    parser.add_argument("--mode", choices=["single", "multi"], default="single",
                        help="single: 영상 1개 / multi: 폴더 내 여러 영상 앙상블")
    parser.add_argument("--input",     help="[single] raw JSON 경로")
    parser.add_argument("--input_dir", help="[multi]  raw JSON 폴더 경로")
    parser.add_argument("--output",  required=True, help="마스터 JSON 저장 경로")
    parser.add_argument("--task",    default="task", help="[multi] 공정 이름")
    args = parser.parse_args()

    if args.mode == "single":
        if not args.input:
            parser.error("--mode single 은 --input 이 필요합니다")
        build(args.input, args.output)
    else:
        if not args.input_dir:
            parser.error("--mode multi 는 --input_dir 이 필요합니다")
        build_from_multiple(args.input_dir, args.output, args.task)
