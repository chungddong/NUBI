// ② 전복 · 무게중심 — 바퀴 네 접점 (지지 사각형) 과 무게중심으로 넘어지는 각을 낸다. 추가 모듈 무게 · 높이를 바꿔 본다.
//   옆 전복각 = atan((접점 y − |무게중심 y|) / 무게중심 z), 앞 · 뒤 = atan((휠베이스/2 ∓ 무게중심 x) / z)
//   미끄러짐 = atan(μ) — 고무 바퀴 · 마른 보도 μ ≈ 0.6 [가정]. 회전 중 원심 가속 v·ω 는 옆 기울기에 atan(v·ω / g) 로 더한다.
export function makeStability({ THREE, car, data }) {
  const G = 9.81, MU = 0.6;
  const rail_top = 0.3169;                       // 모듈 레일 윗면 (DECISIONS D-16)
  const payload = { m: 0, h: 0.3, x: 0.0 };       // 레일 위 높이 (무게중심), 앞뒤 위치
  const yC = Math.min(...data.wheels.map((w) => Math.abs(w.y)));
  const xF = Math.max(...data.wheels.map((w) => w.x)), xR = Math.min(...data.wheels.map((w) => w.x));

  // 표시: 무게중심 공 · 바닥 투영선 · 지지 사각형 · 추가 모듈 상자
  const ball = new THREE.Mesh(new THREE.SphereGeometry(0.025, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff3b30, depthTest: false }));
  ball.renderOrder = 5;
  const dropGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  const drop = new THREE.Line(dropGeo, new THREE.LineDashedMaterial({ color: 0xff3b30, dashSize: 0.02, gapSize: 0.015, depthTest: false }));
  drop.renderOrder = 5;
  const sup = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(
    [[xR, -yC], [xF, -yC], [xF, yC], [xR, yC]].map(([x, y]) => new THREE.Vector3(x, y, 0.008))),
    new THREE.LineBasicMaterial({ color: 0xffd43b }));
  const box = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.4, 0.3), new THREE.MeshStandardMaterial({ color: 0xf08c00, transparent: true, opacity: 0.45 }));
  const grp = new THREE.Group();
  grp.add(ball, drop, sup, box);
  grp.visible = false;
  car.add(grp);

  function compute() {
    const m = data.mass + payload.m;
    const pz = rail_top + payload.h;
    const com = [(data.com[0] * data.mass + payload.x * payload.m) / m, (data.com[1] * data.mass) / m,
      (data.com[2] * data.mass + pz * payload.m) / m];
    const deg = (r) => r * 180 / Math.PI;
    const r = {
      m, com,
      side: deg(Math.atan((yC - Math.abs(com[1])) / com[2])),
      fwd: deg(Math.atan((xF - com[0]) / com[2])),
      back: deg(Math.atan((com[0] - xR) / com[2])),
      slide: deg(Math.atan(MU)),
    };
    ball.position.set(...com);
    dropGeo.setFromPoints([new THREE.Vector3(...com), new THREE.Vector3(com[0], com[1], 0.008)]);
    drop.computeLineDistances();
    box.visible = payload.m > 0;
    const s = Math.cbrt(Math.max(payload.m, 0.5) / 5);                 // 5 kg 이면 0.3 × 0.4 × 0.3 상자 (보기용)
    box.scale.set(s, s, s);
    box.position.set(payload.x, 0, pz);
    return r;
  }
  let res = compute();
  // 지금 자세 (롤 · 피치) 와 회전 원심 가속으로 여유를 본다
  function check({ roll, pitch, v, w }) {
    const lat = Math.atan(Math.abs(v * w) / G) * 180 / Math.PI;
    const side = Math.abs(roll * 180 / Math.PI) + lat;
    const fb = pitch * 180 / Math.PI;
    const tipM = Math.min(res.side - side, fb >= 0 ? res.back - fb : res.fwd + fb);
    const slideM = res.slide - Math.max(Math.abs(roll * 180 / Math.PI), Math.abs(fb));
    return { side, lat, fb, tipMargin: tipM, slideMargin: slideM };
  }
  return {
    set(p) { Object.assign(payload, p); res = compute(); return res; },
    get: () => res, payload, check,
    setVisible: (on) => { grp.visible = on; },
  };
}
