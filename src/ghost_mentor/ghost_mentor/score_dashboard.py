"""
score_dashboard.py
────────────────────────────────────────────────────────────
/match_score, /process_passed 토픽을 구독해
터미널에 실시간 점수 대시보드를 출력

실행:
    ros2 run ghost_mentor score_dashboard
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
import time
import json


class ScoreDashboard(Node):
    def __init__(self):
        super().__init__("score_dashboard")

        self.create_subscription(Float32, "/match_score",    self.on_score,  10)
        self.create_subscription(Bool,    "/process_passed", self.on_passed, 10)
        self.create_subscription(String,  "/ar_status",      self.on_status, 10)

        self.score:  float = 0.0
        self.passed: bool  = False
        self.ghost_frame: int = 0
        self.pass_count: int  = 0
        self.start_time = time.time()

        # 1Hz 출력 타이머
        self.create_timer(1.0, self.print_dashboard)

    def on_score(self, msg):
        self.score = msg.data

    def on_passed(self, msg):
        if msg.data and not self.passed:
            self.pass_count += 1
        self.passed = msg.data

    def on_status(self, msg):
        try:
            d = json.loads(msg.data)
            self.ghost_frame = d.get("ghost_frame", 0)
        except Exception:
            pass

    def print_dashboard(self):
        elapsed = int(time.time() - self.start_time)
        bar_len = 30
        filled  = int(bar_len * self.score / 100.0)
        bar     = "█" * filled + "░" * (bar_len - filled)
        status  = "✓ PASS" if self.passed else "✗ adjusting"

        print("\033[2J\033[H", end="")   # 터미널 클리어
        print("┌─────────────────────────────────────┐")
        print("│      AR Ghost Mentor — Dashboard    │")
        print("├─────────────────────────────────────┤")
        print(f"│  일치도:  [{bar}] {self.score:5.1f}%  │")
        print(f"│  상태:    {status:<28}│")
        print(f"│  Ghost:   frame {self.ghost_frame:<20}│")
        print(f"│  누적 성공: {self.pass_count}회   경과: {elapsed}s       │")
        print("└─────────────────────────────────────┘")


def main(args=None):
    rclpy.init(args=args)
    node = ScoreDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
