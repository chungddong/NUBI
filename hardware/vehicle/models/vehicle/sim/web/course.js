// 시험 코스 — 지면 높이 함수 (경사로 · 턱) + 장애물 (상자 · 기둥 · 통로 벽) + 광선 투사용 메시 목록
import * as THREE from 'three';

export function makeCourse(scene) {
  const group = new THREE.Group();
  scene.add(group);
  const solids = [];                     // 광선 투사 대상 (센서)
  const boxes = [];                      // 충돌: [x0, x1, y0, y1, z1]
  const circles = [];                    // 충돌: [x, y, r, h]

  // 지면
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60),
    new THREE.MeshStandardMaterial({ color: 0x9aa48f, roughness: 1 }));
  ground.receiveShadow = true;
  group.add(ground);
  solids.push(ground);
  const grid = new THREE.GridHelper(60, 60, 0x6c7563, 0x86907d);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.001;
  group.add(grid);

  // ---- 높이 지형: 경사로 (올라가는 면 · 윗판 · 내려가는 면) 와 턱 (계단 블록)
  const ramps = [
    { name: 'ramp10', x0: 4.0, y0: -4.5, y1: -2.5, ang: 10, top: 0.35, flat: 1.2 },   // 오르막 길이 약 2.0 m
    { name: 'ramp15', x0: 4.0, y0: 2.5, y1: 4.5, ang: 15, top: 0.35, flat: 1.2 },     // 오르막 길이 약 1.3 m
  ];
  for (const r of ramps) {
    r.run = r.top / Math.tan(r.ang * Math.PI / 180);
    r.x1 = r.x0 + r.run;                 // 오르막 끝
    r.x2 = r.x1 + r.flat;                // 윗판 끝
    r.x3 = r.x2 + r.run;                 // 내리막 끝
  }
  // 옆 경사 (차가 x 로 달리면 옆으로 기운다) — y0 에서 ym 까지 오르고 y1 까지 내린다 (전복 · 미끄럼 시험)
  const sides = [{ name: 'side20', x0: -5.0, x1: -1.0, y0: -9.4, ym: -8.0, y1: -6.6, ang: 20 }];
  for (const r of sides) r.top = (r.ym - r.y0) * Math.tan(r.ang * Math.PI / 180);
  const steps = [                        // 턱 (도로 연석) — 높이가 다른 두 개
    { name: 'curb20', x0: -4.0, x1: -2.6, y0: -4.5, y1: -2.5, h: 0.020 },
    { name: 'curb40', x0: -4.0, x1: -2.6, y0: 2.5, y1: 4.5, h: 0.040 },
  ];

  function height(x, y) {
    let h = 0;
    for (const r of ramps) {
      if (y < r.y0 || y > r.y1) continue;
      if (x >= r.x0 && x < r.x1) h = Math.max(h, (x - r.x0) / r.run * r.top);
      else if (x >= r.x1 && x < r.x2) h = Math.max(h, r.top);
      else if (x >= r.x2 && x < r.x3) h = Math.max(h, r.top * (1 - (x - r.x2) / r.run));
    }
    for (const s of steps) if (x >= s.x0 && x <= s.x1 && y >= s.y0 && y <= s.y1) h = Math.max(h, s.h);
    for (const r of sides) {
      if (x < r.x0 || x > r.x1 || y < r.y0 || y > r.y1) continue;
      h = Math.max(h, y < r.ym ? (y - r.y0) / (r.ym - r.y0) * r.top : (r.y1 - y) / (r.y1 - r.ym) * r.top);
    }
    return h;
  }

  const rampMat = new THREE.MeshStandardMaterial({ color: 0xb9b3a6, roughness: 0.95 });
  for (const r of ramps) {
    // 사다리꼴 단면 (x-z) 을 y 로 밀어낸 것
    const sh = new THREE.Shape([new THREE.Vector2(r.x0, 0), new THREE.Vector2(r.x1, r.top), new THREE.Vector2(r.x2, r.top),
      new THREE.Vector2(r.x3, 0)]);
    const g = new THREE.ExtrudeGeometry(sh, { depth: r.y1 - r.y0, bevelEnabled: false });
    g.rotateX(Math.PI / 2);
    g.translate(0, r.y1, 0);
    const m = new THREE.Mesh(g, rampMat);
    m.castShadow = m.receiveShadow = true;
    group.add(m);
    solids.push(m);
    label(`경사 ${r.ang}°`, (r.x0 + r.x3) / 2, (r.y0 + r.y1) / 2, r.top + 0.35);
  }
  for (const r of sides) {                // 단면 (y-z) 삼각형을 x 로 밀어낸 삼각기둥 — 꼭짓점을 직접 찍는다
    const A = (x) => [[x, r.y0, 0], [x, r.ym, r.top], [x, r.y1, 0]];
    const [a0, a1, a2] = A(r.x0), [b0, b1, b2] = A(r.x1);
    const tri = [a0, b0, b1, a0, b1, a1, a1, b1, b2, a1, b2, a2, a0, a2, b2, a0, b2, b0, a0, a1, a2, b0, b2, b1];
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(tri.flat(), 3));
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0xb9b3a6, roughness: 0.95, side: THREE.DoubleSide }));
    m.castShadow = m.receiveShadow = true;
    group.add(m);
    solids.push(m);
    label(`옆 경사 ${r.ang}°`, (r.x0 + r.x1) / 2, r.ym, r.top + 0.35);
  }
  const curbMat = new THREE.MeshStandardMaterial({ color: 0xcfcac0, roughness: 0.9 });
  for (const s of steps) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(s.x1 - s.x0, s.y1 - s.y0, s.h), curbMat);
    m.position.set((s.x0 + s.x1) / 2, (s.y0 + s.y1) / 2, s.h / 2);
    m.castShadow = m.receiveShadow = true;
    group.add(m);
    solids.push(m);
    label(`턱 ${Math.round(s.h * 1000)} mm`, (s.x0 + s.x1) / 2, (s.y0 + s.y1) / 2, 0.35);
  }

  // ---- 장애물
  const boxMat = new THREE.MeshStandardMaterial({ color: 0xc08a4a, roughness: 0.8 });
  const wallMat = new THREE.MeshStandardMaterial({ color: 0x8d949c, roughness: 0.9 });
  const poleMat = new THREE.MeshStandardMaterial({ color: 0x4a6fa5, roughness: 0.7 });
  function box(x, y, sx, sy, sz, mat = boxMat) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat);
    m.position.set(x, y, sz / 2);
    m.castShadow = m.receiveShadow = true;
    group.add(m);
    solids.push(m);
    boxes.push([x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, sz]);
  }
  function pole(x, y, r, h) {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 24), poleMat);
    m.rotation.x = Math.PI / 2;
    m.position.set(x, y, h / 2);
    m.castShadow = m.receiveShadow = true;
    group.add(m);
    solids.push(m);
    circles.push([x, y, r, h]);
  }
  box(2.5, 0.0, 0.4, 0.4, 0.4);          // 정면 상자
  box(2.8, 1.0, 0.3, 0.3, 0.12);         // 낮은 상자 (12 cm — 카메라 · 라이다 높이 시험)
  box(1.6, -1.2, 0.6, 0.4, 0.8);
  pole(3.5, -0.8, 0.18, 1.7);            // 사람 크기 기둥
  pole(-1.8, 1.2, 0.18, 1.7);
  pole(-1.0, -1.6, 0.05, 1.2);           // 가는 기둥 (볼라드)
  // 좁은 통로 — 폭 0.8 m (차 폭 약 0.58 m)
  const cx = -6.0;
  box(cx, -0.40 - 0.05, 3.0, 0.1, 0.6, wallMat);
  box(cx, 0.40 + 0.05, 3.0, 0.1, 0.6, wallMat);
  label('통로 폭 0.8 m', cx, 0, 0.85);
  // 움직이는 사람 2 — 두 점 사이를 오가고, 차가 앞을 막으면 기다린다
  const movers = [];
  const walkMat = new THREE.MeshStandardMaterial({ color: 0xd9480f, roughness: 0.7 });
  for (const [ax, ay, bx, by, sp] of [[1.5, -2.8, 1.5, 2.8, 0.7], [-3.4, -2.2, -3.4, 2.2, 0.6]]) {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.7, 24), walkMat);
    m.rotation.x = Math.PI / 2;
    m.position.set(ax, ay, 0.85);
    m.castShadow = true;
    group.add(m);
    solids.push(m);
    const c = [ax, ay, 0.22, 1.7];
    circles.push(c);
    movers.push({ a: [ax, ay], b: [bx, by], sp, t: 0, dir: 1, m, c, on: true });
  }
  function update(dt, carPoly) {
    for (const w of movers) {
      if (!w.on) continue;
      const L = Math.hypot(w.b[0] - w.a[0], w.b[1] - w.a[1]);
      let t = w.t + w.dir * w.sp * dt / L;
      if (t > 1) { t = 1; w.dir = -1; } else if (t < 0) { t = 0; w.dir = 1; }
      const x = w.a[0] + (w.b[0] - w.a[0]) * t, y = w.a[1] + (w.b[1] - w.a[1]) * t;
      // 차에 0.37 m 안으로 '더 가까워지는' 걸음만 막는다 (멀어지는 쪽은 간다) — 막히면 기다리다 1.2 s 넘으면 돌아선다
      const d0 = carPoly ? polyDist(carPoly, w.c[0], w.c[1]) : 1e9, d1 = carPoly ? polyDist(carPoly, x, y) : 1e9;
      if (d1 < 0.22 + 0.15 && d1 < d0 - 1e-6) {
        w.wait = (w.wait || 0) + dt;
        if (w.wait > 1.2) { w.dir *= -1; w.wait = 0; }
        continue;
      }
      w.wait = 0;
      w.t = t;
      w.m.position.set(x, y, 0.85);
      w.m.updateMatrixWorld();                                         // 광선 투사는 matrixWorld 를 본다 (화면을 안 그려도)
      w.c[0] = x; w.c[1] = y;
    }
  }
  function polyDist(poly, x, y) {                                    // 차 윤곽까지 거리 (안이면 0)
    if (inside(poly, x, y)) return 0;
    let d = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const [ax, ay] = poly[i], [bx, by] = poly[(i + 1) % poly.length];
      const dx = bx - ax, dy = by - ay, t = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)));
      d = Math.min(d, Math.hypot(ax + dx * t - x, ay + dy * t - y));
    }
    return d;
  }

  // 바깥 울타리
  for (const [x, y, sx, sy] of [[0, -10, 24, 0.2], [0, 10, 24, 0.2], [-12, 0, 0.2, 20], [12, 0, 0.2, 20]]) box(x, y, sx, sy, 0.5, wallMat);

  function label(text, x, y, z) {
    const c = document.createElement('canvas');
    c.width = 256; c.height = 64;
    const g = c.getContext('2d');
    g.fillStyle = 'rgba(20,24,28,0.75)';
    g.fillRect(0, 0, 256, 64);
    g.fillStyle = '#fff';
    g.font = 'bold 28px Malgun Gothic, sans-serif';
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.fillText(text, 128, 33);
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
    sp.scale.set(0.9, 0.225, 1);
    sp.position.set(x, y, z);
    group.add(sp);
  }

  // 차 바닥 윤곽 점들이 장애물 안에 있으면 그 이름을 돌려준다 (경사로 · 턱은 높이로 따로 본다)
  function hits(pts) {
    for (const [x, y] of pts) {
      for (const b of boxes) if (x > b[0] && x < b[1] && y > b[2] && y < b[3]) return `장애물 (${(b[4] * 1000).toFixed(0)} mm 높이)`;
      for (const c of circles) if (!c.off && Math.hypot(x - c[0], y - c[1]) < c[2]) return '기둥';
    }
    // 장애물 모서리가 차 윤곽 안에 들어오는 경우 (가는 기둥) — 윤곽 사각형 안에 점이 있는지
    const xs = pts.slice(0, 4);
    for (const c of circles) if (!c.off && inside(xs, c[0], c[1])) return '기둥';
    return null;
  }
  function inside(poly, x, y) {
    let s = 0;
    for (let i = 0; i < poly.length; i++) {
      const [ax, ay] = poly[i], [bx, by] = poly[(i + 1) % poly.length];
      const cr = (bx - ax) * (y - ay) - (by - ay) * (x - ax);
      if (cr === 0) continue;
      if (s === 0) s = Math.sign(cr); else if (Math.sign(cr) !== s) return false;
    }
    return true;
  }

  const spots = {
    start: { x: 0, y: 0, yaw: 0 },
    ramp10: { x: 2.8, y: -3.5, yaw: 0 },
    ramp15: { x: 2.8, y: 3.5, yaw: 0 },
    curb: { x: -1.6, y: 3.5, yaw: Math.PI },
    corridor: { x: -3.6, y: 0, yaw: Math.PI },
    side20: { x: -4.4, y: -8.7, yaw: 0 },
  };
  group.updateMatrixWorld(true);         // 화면을 한 번도 안 그렸어도 광선 투사가 제자리를 보게
  const names = { start: '출발점', ramp10: '경사 10°', ramp15: '경사 15°', curb: '턱', corridor: '통로', side20: '옆 경사 20°' };
  return { kind: 'course', group, solids, height, hits, spots, names, ramps, steps, sides, movers, update, bounds: [-12, -10, 12, 10],
    // 끄면 화면 · 광선 투사 (깊이 · 라이다 — Raycaster 는 visible 을 안 보고 레이어만 본다) · 충돌에서 모두 뺀다
    setMovers: (on) => movers.forEach((w) => { w.on = on; w.m.visible = on; w.m.layers.set(on ? 0 : 31); w.c.off = !on; }) };
}
