"""앞축 홀 스텝 → 지도 위 위치 (추측 항법).

ESP32 보고 줄의 odo 두 값 = 앞축 (마스터, 슬레이브) 바퀴의 홀 스텝. 마스터 = 오른쪽 바퀴 (ISSUES H3).
바퀴마다 '앞으로 가면 +' 가 되게 부호를 곱하고 (odo_sign — 거울로 붙은 모터라 한쪽은 거꾸로 셀 수 있다),
스텝당 거리 (m_per_step) 와 스키드 스티어 유효 윤거 (track_eff_m — 미끄럼 때문에 실제 윤거보다 크다) 로 바꾼다.
세 값 모두 calibrate.py 로 실측해 robot.json 에 적는다. 돌 때 미끄럼이 커서 방향은 나중에 IMU 로 바꾸는 것이 좋다.
"""

import math


class Odometry:
    def __init__(self, cfg, start=(0.0, 0.0, 0.0)):
        self.m = cfg["m_per_step"]
        self.sign = cfg["odo_sign"]                  # [마스터 = 오른쪽, 슬레이브 = 왼쪽]
        self.track = cfg["track_eff_m"]
        self.x, self.y, self.yaw = start
        self.sigma = 0.05
        self.last = None
        self.dist = 0.0                              # 누적 주행 거리 (진단용)
        self.jumps = 0

    def update(self, odo):
        if self.last is None:
            self.last = odo
            return
        dr = (odo[0] - self.last[0]) * self.sign[0] * self.m     # 오른쪽 바퀴
        dl = (odo[1] - self.last[1]) * self.sign[1] * self.m     # 왼쪽 바퀴
        self.last = odo
        if abs(dr) > 1.0 or abs(dl) > 1.0:           # 보고 줄 사이 (0.2 s) 1 m 넘게 = 보드가 다시 켜져 카운터가 바뀜
            self.jumps += 1
            return
        d = (dr + dl) / 2
        dyaw = (dr - dl) / self.track                # 오른쪽이 더 가면 왼쪽으로 돈다 (+)
        mid = self.yaw + dyaw / 2
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        self.dist += abs(d)
        self.sigma = min(3.0, self.sigma + 0.04 * abs(d) + 0.02 * abs(dyaw))   # 시뮬레이터와 같은 어림 [가정]

    def pose(self):
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "sigma": self.sigma, "src": "오도메트리"}

    def reset(self, x, y, yaw, sigma=0.05):
        self.x, self.y, self.yaw, self.sigma = x, y, yaw, sigma
