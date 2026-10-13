// 복도 환경 — 영상으로 복원한 실제 건물 3 층 복도 · 라운지 (D:\Develop\ClaudeGame\hallway-3d, import_hallway.py 로 복사).
// 원본 형상 코드 (buildArchitecture · buildObjects) 를 그대로 쓰고, y 가 위인 원본을 x 축으로 +90° 돌려 z 가 위로 맞춘다:
//   우리 (x, y, z) = 원본 (x, −z, y)  — 원본 z 는 라운지 쪽으로 걸을 때 오른쪽, 우리 y 는 왼쪽.
// course.js 와 같은 모양의 객체를 돌려준다 (solids · height · hits · spots · update · movers · bounds).
import * as THREE from 'three';

export async function makeHallway(scene) {
  const base = 'hallway/';
  const [{ MaterialLib }, { buildArchitecture }, { buildObjects }, { setMaxAnisotropy }] = await Promise.all([
    import('./hallway/world/materials.js'), import('./hallway/world/architecture.js'),
    import('./hallway/world/objects.js'), import('./hallway/world/textures.js')]);
  const data = await (await fetch(base + 'data/scene.json')).json();
  setMaxAnisotropy(8);
  const mats = new MaterialLib(data.palette, base);
  const arch = buildArchitecture(data, mats);
  const { root: objRoot, colliders } = buildObjects(data, mats, base);
  const group = new THREE.Group();
  group.rotation.x = Math.PI / 2;                       // 원본 y (위) → 우리 z
  group.add(arch.root, objRoot);
  arch.ceilings.visible = false;                        // 위에서 보려고 천장은 숨긴다 (광선 투사에서도 뺀다)
  group.traverse((m) => { if (m.isMesh) { m.castShadow = false; m.receiveShadow = true; } });
  scene.add(group);
  group.updateMatrixWorld(true);

  // 광선 투사 대상 — 바닥 · 벽 · 오브젝트 (천장 빼고). 벽은 안쪽 면만 있는 판이라 안에서 쏘면 맞는다.
  const solids = [];
  for (const g of [arch.floors, arch.walls, objRoot]) g.traverse((m) => { if (m.isMesh && m.visible !== false) solids.push(m); });

  // 충돌 — 원본 바닥 평면 (x, z) → 우리 (x, −z)
  const segs = colliders.segments.map(([ax, az, bx, bz]) => [ax, -az, bx, -bz]);
  const boxes = colliders.boxes.map((b) => {
    const c = Math.cos(b.rot), s = Math.sin(b.rot);
    const cx = b.cx, cy = -b.cz;
    const ux = [c, s], uy = [-s, c];                    // 원본 회전 rot → 우리 xy 평면에서 같은 각
    return { cx, cy, hx: b.hx, hy: b.hz, ux, uy,
      corners: [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([i, j]) => [cx + ux[0] * b.hx * i + uy[0] * b.hz * j, cy + ux[1] * b.hx * i + uy[1] * b.hz * j]) };
  });
  const circles = [];

  // 움직이는 사람 2 — 복도를 따라 오간다
  const movers = [];
  const walkMat = new THREE.MeshStandardMaterial({ color: 0xd9480f, roughness: 0.7 });
  for (const [ax, ay, bx, by, sp] of [[4.0, 0.55, 22.0, 0.55, 0.8], [21.0, -0.5, 6.0, -0.5, 0.7]]) {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.7, 24), walkMat);
    m.rotation.x = Math.PI / 2;
    m.position.set(ax, ay, 0.85);
    m.updateMatrixWorld();
    scene.add(m);
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
      const d0 = carPoly ? polyDist(carPoly, w.c[0], w.c[1]) : 1e9, d1 = carPoly ? polyDist(carPoly, x, y) : 1e9;
      if (d1 < 0.37 && d1 < d0 - 1e-6) { w.wait = (w.wait || 0) + dt; if (w.wait > 1.2) { w.dir *= -1; w.wait = 0; } continue; }
      w.wait = 0;
      w.t = t;
      w.m.position.set(x, y, 0.85);
      w.m.updateMatrixWorld();
      w.c[0] = x; w.c[1] = y;
    }
  }

  // ---- 기하 도우미
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
  function polyDist(poly, x, y) {
    if (inside(poly, x, y)) return 0;
    let d = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const [ax, ay] = poly[i], [bx, by] = poly[(i + 1) % poly.length];
      const dx = bx - ax, dy = by - ay, t = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)));
      d = Math.min(d, Math.hypot(ax + dx * t - x, ay + dy * t - y));
    }
    return d;
  }
  function segX(p1, p2, p3, p4) {                       // 두 선분이 만나는가
    const d = (a, b, c) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    const d1 = d(p3, p4, p1), d2 = d(p3, p4, p2), d3 = d(p1, p2, p3), d4 = d(p1, p2, p4);
    return ((d1 > 0) !== (d2 > 0)) && ((d3 > 0) !== (d4 > 0));
  }
  function inBox(b, x, y) {
    const dx = x - b.cx, dy = y - b.cy;
    return Math.abs(dx * b.ux[0] + dy * b.ux[1]) < b.hx && Math.abs(dx * b.uy[0] + dy * b.uy[1]) < b.hy;
  }
  // 차 바닥 윤곽 (앞 네 점이 모서리) 이 벽 · 가구 · 사람과 닿는가
  function hits(pts) {
    const poly = pts.slice(0, 4);
    for (const [ax, ay, bx, by] of segs) {
      if (inside(poly, ax, ay) || inside(poly, bx, by)) return '벽';
      for (let i = 0; i < 4; i++) if (segX(poly[i], poly[(i + 1) % 4], [ax, ay], [bx, by])) return '벽';
    }
    for (const b of boxes) {
      for (const [x, y] of pts) if (inBox(b, x, y)) return '가구';
      for (const [x, y] of b.corners) if (inside(poly, x, y)) return '가구';
    }
    for (const c of circles) {
      if (inside(poly, c[0], c[1]) || polyDist(poly, c[0], c[1]) < c[2]) return '사람';
    }
    return null;
  }

  const spots = {
    start: { x: 1.4, y: 0.0, yaw: 0 },
    mid: { x: 14.0, y: 0.0, yaw: 0 },
    lounge: { x: 27.0, y: 0.0, yaw: 0 },
    back: { x: 24.0, y: 0.0, yaw: Math.PI },
  };
  const names = { start: '복도 입구', mid: '복도 가운데', lounge: '라운지 앞', back: '되돌아오기' };
  return {
    kind: 'hallway', title: data.title, subtitle: data.subtitle, group, solids, height: () => 0, hits, spots, names,
    ramps: [], steps: [], sides: [], movers, update, bounds: [-3.5, -13.0, 48.0, 6.0],
    setMovers: (on) => movers.forEach((w) => { w.on = on; w.m.visible = on; }),
  };
}
