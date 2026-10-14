"""오도메트리에 카메라 위치 (SfM 지도) 를 섞는다 — 실차 (drive.py) 와 다시 돌려 보기 (replay.py) 에서 같이 쓴다.

카메라 답은 사진을 찍은 때의 자세라서, 그때의 오도메트리 자세와 비교한 차이 (혁신) 를 지금 자세에 더한다 (계산이 1 ~ 3 s
늦어도 맞게). 오도메트리와 너무 다르면 버리고 (게이트는 마지막 고정 뒤 간 거리만큼 넓어진다), 연속 몇 번 버리면 길을
잃은 것으로 보고, 그 뒤 서로 맞는 카메라 답이 두 번 이어지면 (혁신 차이 0.5 m · 10° 안) 그것을 받는다
(2026-10-14 다시 돌려 보기: 로비 입구에서 인라이어 100 넘게 틀린 답이 연달아 나와 한 번만에 받으면 2 m 튄다).
"""

import math
from collections import deque

wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


class PoseFusion:
    def __init__(self, odo, cam_off=(0.35, 0.0, 0.0), gate_m=1.0, gate_per_m=0.08, gate_deg=25.0, trust=0.8, lost_after=3):
        self.odo = odo
        self.off = cam_off                                   # 카메라가 차 중심에서 (앞 m, 왼쪽 m, 왼쪽으로 돌린 각 °)
        self.gate_m, self.gate_per_m, self.gate_deg = gate_m, gate_per_m, gate_deg
        self.trust, self.lost_after = trust, lost_after
        self.hist = deque(maxlen=2000)                       # (t, x, y, yaw, 누적 거리)
        self.n_fix = self.rejects = 0
        self.dist_at_fix = 0.0
        self.cand = None                                     # 길 잃었을 때 다시 잡을 후보 혁신

    def record(self, t):
        o = self.odo
        self.hist.append((t, o.x, o.y, o.yaw, o.dist))

    def _at(self, t):
        best = None
        for h in reversed(self.hist):
            if h[0] <= t:
                return h
            best = h
        return best or (t, self.odo.x, self.odo.y, self.odo.yaw, self.odo.dist)

    def base_from_cam(self, cx, cy, cyaw_deg):
        fx, fy, fyaw = self.off
        yaw = math.radians(cyaw_deg - fyaw)
        return (cx - fx * math.cos(yaw) + fy * math.sin(yaw), cy - fx * math.sin(yaw) - fy * math.cos(yaw), yaw)

    def cam_from_base(self, x, y, yaw):
        """prior 로 쓸 카메라 자세 (x, y, yaw°)."""
        fx, fy, fyaw = self.off
        return (x + fx * math.cos(yaw) - fy * math.sin(yaw), y + fx * math.sin(yaw) + fy * math.cos(yaw),
                math.degrees(yaw) + fyaw)

    def fix(self, t, cx, cy, cyaw_deg):
        """카메라 답 하나. 돌려줌: (받음 여부, 이유, 혁신 (dx, dy, dyaw°))."""
        bx, by, byaw = self.base_from_cam(cx, cy, cyaw_deg)
        o = self.odo
        if self.n_fix == 0:
            o.x, o.y, o.yaw = bx, by, byaw
            self.n_fix, self.dist_at_fix, self.rejects = 1, o.dist, 0
            return True, "처음", (0.0, 0.0, 0.0)
        h = self._at(t)
        dx, dy, dyaw = bx - h[1], by - h[2], wrap(byaw - h[3])
        gate = self.gate_m + self.gate_per_m * (o.dist - self.dist_at_fix)
        innov = (dx, dy, math.degrees(dyaw))
        lost = self.rejects >= self.lost_after
        if lost:                                             # 연속 두 답이 서로 맞아야 다시 잡는다
            c, self.cand = self.cand, innov
            if c is None or math.hypot(dx - c[0], dy - c[1]) > 0.5 or abs(innov[2] - c[2]) > 10:
                return False, "길 잃음 — 확인 중", innov
        elif math.hypot(dx, dy) > gate or abs(math.degrees(dyaw)) > self.gate_deg:
            self.rejects += 1
            return False, f"게이트 밖 ({math.hypot(dx, dy):.2f} m > {gate:.2f}, {math.degrees(dyaw):+.0f}°)", innov
        k = 1.0 if lost else self.trust
        why = "길 잃음 → 다시 잡음" if lost else "받음"
        o.x += k * dx
        o.y += k * dy
        o.yaw = wrap(o.yaw + k * dyaw)
        o.sigma = 0.1
        self.n_fix += 1
        self.rejects = 0
        self.cand = None
        self.dist_at_fix = o.dist
        return True, why, innov
