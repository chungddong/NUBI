// ① 센서 사각지대 지도 — 차 주변 바닥 (차체 좌표) 을 칸으로 나눠, 높이 H 인 장애물 기둥을 누가 보는지 칠한다.
//   카메라: 기둥 [0, H] 가 시야 (90° × 65°, 깊이 near~far) 와 겹치면 본다 — 진한 초록 = 이상 범위 0.3~3 m
//   라이다: 기둥이 스캔 평면 (z) 까지 올라오면 (H ≥ z) 거리 안에서 본다
//   차체 자기 가림 (범퍼 · 테두리 돌출 18~25 mm) 은 넣지 않았다 — 시야가 렌즈면에서 바로 시작한다고 본다.
export function makeCoverage({ THREE, car, data }) {
  const R = 3.0, CELL = 0.05, N = Math.round(2 * R / CELL);
  const cv = document.createElement('canvas');
  cv.width = cv.height = N;
  const g2 = cv.getContext('2d');
  const img = g2.createImageData(N, N);
  const tex = new THREE.CanvasTexture(cv);
  tex.magFilter = THREE.NearestFilter;
  const plane = new THREE.Mesh(new THREE.PlaneGeometry(2 * R, 2 * R),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
  plane.position.z = 0.006;
  plane.renderOrder = 2;
  plane.visible = false;
  car.add(plane);

  const bb = data.bbox;
  const foot = { x0: bb.min[0], x1: bb.max[0], y0: bb.min[1], y1: bb.max[1] };
  const D2R = Math.PI / 180;
  const cams = data.cameras.map((c) => ({ ...c, cy: Math.cos(c.yaw * D2R), sy: Math.sin(c.yaw * D2R),
    tx: Math.tan(c.hfov / 2 * D2R), tz: Math.tan(c.vfov / 2 * D2R) }));
  const L = data.lidar;
  let H = 0.1, stats = null;
  const code = new Uint8Array(N * N);          // 0 차체 · 1 사각 · 2 카메라 · 3 카메라 이상 · 4 라이다 · 5 둘 다

  function camSees(c, x, y) {
    const rx = x - c.x, ry = y - c.y;
    const d = rx * c.cy + ry * c.sy;             // 광축 방향 깊이
    if (d < c.near || d > c.far) return 0;
    const u = -rx * c.sy + ry * c.cy;
    if (Math.abs(u) > d * c.tx) return 0;
    if (c.z - d * c.tz > H) return 0;            // 시야 아래끝이 기둥 꼭대기보다 위 (가까운 낮은 장애물)
    return d >= c.ideal[0] && d <= c.ideal[1] ? 2 : 1;
  }
  function lidarSees(x, y) {
    if (!L || H < L.z) return false;
    const d = Math.hypot(x - L.x, y - L.y);
    return d >= 0.03 && d <= L.range;
  }
  function edgeDist(x, y) {                      // 차 바닥 윤곽까지 거리 (밖이면 +)
    const dx = Math.max(foot.x0 - x, 0, x - foot.x1), dy = Math.max(foot.y0 - y, 0, y - foot.y1);
    return Math.hypot(dx, dy);
  }

  function compute(h) {
    H = h;
    let blindNear = 0, nearCells = 0;
    for (let j = 0; j < N; j++) {
      for (let i = 0; i < N; i++) {
        const x = -R + (i + 0.5) * CELL, y = R - (j + 0.5) * CELL;   // 텍스처 위쪽 = +y
        const k = j * N + i;
        if (x > foot.x0 && x < foot.x1 && y > foot.y0 && y < foot.y1) { code[k] = 0; continue; }
        let cam = 0;
        for (const c of cams) cam = Math.max(cam, camSees(c, x, y));
        const li = lidarSees(x, y);
        code[k] = cam && li ? 5 : li ? 4 : cam === 2 ? 3 : cam ? 2 : 1;
        if (edgeDist(x, y) <= 1.0) { nearCells++; if (code[k] === 1) blindNear++; }
      }
    }
    const COL = [[40, 44, 52, 150], [224, 49, 49, 150], [148, 216, 45, 120], [47, 158, 68, 150], [51, 154, 240, 130], [21, 170, 191, 150]];
    for (let k = 0; k < N * N; k++) {
      let c = COL[code[k]];
      if (code[k] === 1) {                        // 1.5 m 밖 사각은 옅게
        const i = k % N, j = Math.floor(k / N);
        const x = -R + (i + 0.5) * CELL, y = R - (j + 0.5) * CELL;
        if (edgeDist(x, y) > 1.5) c = [224, 49, 49, 40];
      }
      img.data.set(c, 4 * k);
    }
    g2.putImageData(img, 0, 0);
    tex.needsUpdate = true;
    // 방향별 — 바닥 윤곽에서 바깥으로 나가며 처음 보이는 거리
    const dirs = { 앞: [1, 0], 뒤: [-1, 0], 왼: [0, 1], 오른: [0, -1], 앞왼: [Math.SQRT1_2, Math.SQRT1_2], 뒤오른: [-Math.SQRT1_2, -Math.SQRT1_2] };
    const first = {};
    for (const [nm, [dx, dy]] of Object.entries(dirs)) {
      const sx = dx > 0 ? foot.x1 : dx < 0 ? foot.x0 : 0, sy = dy > 0 ? foot.y1 : dy < 0 ? foot.y0 : 0;
      first[nm] = null;
      for (let s = 0.0; s <= R - 0.3; s += 0.02) {
        const x = sx + dx * s, y = sy + dy * s;
        const i = Math.floor((x + R) / CELL), j = Math.floor((R - y) / CELL);
        if (i < 0 || j < 0 || i >= N || j >= N) break;
        if (code[j * N + i] >= 2) { first[nm] = s; break; }
      }
    }
    stats = { H, blindNear: blindNear * CELL * CELL, nearArea: nearCells * CELL * CELL, first };
    return stats;
  }
  return { compute, setVisible: (on) => { plane.visible = on; }, stats: () => stats };
}
