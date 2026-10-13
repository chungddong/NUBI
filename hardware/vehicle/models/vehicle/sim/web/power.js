// 전원 추정 — DECISIONS.md D-04 와 같은 가정. 시뮬레이터의 순간 값이라 추정치다.
//   바퀴 힘 = m·g·(sinθ + 구름) + m·a  (+ 제자리 회전 미끄럼 = μ_lat·m·g·(a/c))
//   상전류 = 바퀴 하나 토크 / Kt,  배터리 전류 = (기계 출력 / η + 구리 손실 + 전자) / V
export function makePower(data) {
  const P = data.power, G = 9.81;
  let m = data.mass;
  const r = data.wheel_radius;
  const a_half = data.wheelbase / 2, c_half = data.track / 2;
  const R_PHASE = 0.15;                     // 상 저항 [가정]
  const V = P.v_nom;

  function update({ v, w, a, pitch, estop = false, lights = false, hold = false }) {
    const pelec = P.elec_w_typ + (lights ? 10 : 0);       // 전자 (젯슨 · 카메라 · ESP32 …) + 전조등 2 × 5 W
    if (hold) {                                            // 정지 유지 — 경사에서 버티는 토크만 (구리 손실)
      const iph = m * G * Math.abs(Math.sin(pitch)) / 4 * r / P.kt;
      const pcu = 4 * 1.5 * iph * iph * R_PHASE;
      return { iphase: iph, ibat: (pcu + pelec) / V, p: pcu + pelec, pmotor: pcu, pelec };
    }
    if (estop) return { iphase: 0, ibat: pelec / V, p: pelec, pmotor: 0, pelec };
    const fLong = m * G * (Math.sin(pitch) * Math.sign(v || 1) + P.f_roll) * (Math.abs(v) > 1e-3 ? 1 : 0) + m * a * Math.sign(v || 1);
    const turning = Math.abs(w) > 1e-3;
    const fScrub = turning ? P.mu_lat * m * G * (a_half / c_half) : 0;   // 미끄럼 회전 — 속도와 무관한 마찰
    const fWheel = (Math.abs(fLong) + fScrub) / 4;
    const torque = fWheel * r;
    const iphase = torque / P.kt;
    const pmech = Math.max(0, fLong * Math.abs(v)) + (turning ? P.mu_lat * m * G * a_half * Math.abs(w) : 0);
    const pcu = 4 * 1.5 * iphase * iphase * R_PHASE;
    const pmotor = pmech / P.eta + pcu;
    const p = pmotor + pelec;
    return { iphase, ibat: p / V, p, pmotor, pelec };
  }
  return { update, setMass: (kg) => { m = kg; } };
}
