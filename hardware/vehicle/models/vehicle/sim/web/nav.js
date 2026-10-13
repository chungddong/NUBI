// ④ 자율주행 — 센서로 점유 격자를 만들고 (라이다 광선이 지나간 칸은 비우고 맞은 칸은 채움, 깊이 점은 장애물 면만),
//   A* 로 경로 (차 반폭 + 여유로 부풀림), 순수 추종 (pure pursuit) 으로 따라간다. 막히면 다시 짠다.
export function makeNav({ THREE, scene, data, bounds = [-12, -10, 12, 10] }) {
  const CELL = 0.1, [X0, Y0] = bounds, NX = Math.ceil((bounds[2] - bounds[0]) / CELL), NY = Math.ceil((bounds[3] - bounds[1]) / CELL);
  // 두 겹 — 라이다 층 (광선이 지나간 칸을 비운다) · 깊이 층 (바닥이 보이는 칸만 비운다).
  // 라이다는 52 cm 높이라 그보다 낮은 상자 위를 지나가므로, 라이다가 깊이 층을 지우면 낮은 장애물이 사라진다.
  const hitsL = new Uint8Array(NX * NY), hitsD = new Uint8Array(NX * NY), hits = new Uint8Array(NX * NY);   // hits = 둘 중 큰 값
  const CAP = 6;
  // 잊기 — 3 s 넘게 다시 안 보인 칸은 0.5 s 마다 하나씩 줄인다 (지나간 사람 흔적이 복도를 영영 막지 않게).
  // 벽 · 가구는 라이다 · 카메라가 계속 보니 남고, 안 보이는 곳에서 잊은 것은 다가가면 다시 본다.
  const seen = new Float32Array(NX * NY);
  let clock = 0, lastDecay = 0;
  function decay(dt) {
    clock += dt;
    if (clock - lastDecay < 0.5) return;
    lastDecay = clock;
    for (let k = 0; k < NX * NY; k++) {
      if (hits[k] === 0 || clock - seen[k] < 3.0) continue;
      if (hitsL[k] > 0) hitsL[k]--;
      if (hitsD[k] > 0) hitsD[k]--;
      hits[k] = Math.max(hitsL[k], hitsD[k]);
    }
  }
  const blocked = new Uint8Array(NX * NY);
  const bb = data.bbox;
  const INFL = Math.max(-bb.min[1], bb.max[1]) + 0.05;          // 차 반폭 (바퀴 포함) + 5 cm
  const st = { active: false, goal: null, path: [], status: '대기', lastPlan: 0, stuckT: 0, lastPos: null, backT: 0, reached: false };

  // ---- 표시: 점유 격자 · 경로 · 목표
  const cv = document.createElement('canvas');
  cv.width = NX; cv.height = NY;
  const g2 = cv.getContext('2d');
  const img = g2.createImageData(NX, NY);
  const tex = new THREE.CanvasTexture(cv);
  tex.magFilter = THREE.NearestFilter;
  const grid = new THREE.Mesh(new THREE.PlaneGeometry(NX * CELL, NY * CELL), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
  grid.position.set(X0 + NX * CELL / 2, Y0 + NY * CELL / 2, 0.012);
  grid.renderOrder = 1;
  grid.visible = false;
  scene.add(grid);
  const pathGeo = new THREE.BufferGeometry();
  const pathLine = new THREE.Line(pathGeo, new THREE.LineBasicMaterial({ color: 0x00e676, depthTest: false }));
  pathLine.renderOrder = 4;
  pathLine.frustumCulled = false;
  scene.add(pathLine);
  const flag = new THREE.Group();
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.6), new THREE.MeshBasicMaterial({ color: 0x00c853 }));
  pole.rotation.x = Math.PI / 2; pole.position.z = 0.3;
  const cloth = new THREE.Mesh(new THREE.PlaneGeometry(0.18, 0.12), new THREE.MeshBasicMaterial({ color: 0x00e676, side: THREE.DoubleSide }));
  cloth.rotation.x = Math.PI / 2; cloth.position.set(0.09, 0, 0.54);
  flag.add(pole, cloth);
  flag.visible = false;
  scene.add(flag);

  const idx = (x, y) => {
    const i = Math.floor((x - X0) / CELL), j = Math.floor((y - Y0) / CELL);
    return i < 0 || j < 0 || i >= NX || j >= NY ? -1 : j * NX + i;
  };

  // ---- 지도 갱신: 광선 (원점 → 끝점, 맞았는지) 과 장애물 점
  const touched = new Set();
  function addRays(rays) {
    for (const [ox, oy, ex, ey, hit] of rays) {
      const L = Math.hypot(ex - ox, ey - oy), n = Math.ceil(L / (CELL * 0.5));
      for (let s = 0; s < n - 1; s++) {                           // 지나간 칸은 비운다
        const k = idx(ox + (ex - ox) * s / n, oy + (ey - oy) * s / n);
        if (k >= 0 && hitsL[k] > 0) { hitsL[k]--; touched.add(k); }
      }
      if (hit) { const k = idx(ex, ey); if (k >= 0) { hitsL[k] = Math.min(CAP, hitsL[k] + 3); seen[k] = clock; touched.add(k); } }
    }
    sync();
  }
  function addPoints(pts, ground = []) {
    for (const [x, y] of ground) { const k = idx(x, y); if (k >= 0 && hitsD[k] > 0) { hitsD[k] = Math.max(0, hitsD[k] - 2); touched.add(k); } }
    for (const [x, y] of pts) { const k = idx(x, y); if (k >= 0) { hitsD[k] = Math.min(CAP, hitsD[k] + 2); seen[k] = clock; touched.add(k); } }   // 한 번 보이면 바로 장애물 (움직이는 사람)
    sync();
  }
  function sync() { for (const k of touched) hits[k] = Math.max(hitsL[k], hitsD[k]); touched.clear(); }
  function clear() { hits.fill(0); hitsL.fill(0); hitsD.fill(0); }

  function inflate() {
    blocked.fill(0);
    const r = Math.ceil(INFL / CELL);
    const disk = [];
    for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) if (Math.hypot(di, dj) * CELL <= INFL) disk.push([di, dj]);
    for (let j = 0; j < NY; j++) for (let i = 0; i < NX; i++) {
      if (hits[j * NX + i] < 2) continue;
      for (const [di, dj] of disk) {
        const a = i + di, b = j + dj;
        if (a >= 0 && b >= 0 && a < NX && b < NY) blocked[b * NX + a] = Math.max(blocked[b * NX + a], di === 0 && dj === 0 ? 2 : 1);
      }
    }
  }

  // ---- A* (8 방향)
  function astar(sx, sy, gx, gy) {
    const s = idx(sx, sy), g = idx(gx, gy);
    if (s < 0 || g < 0) return null;
    const free = (k) => blocked[k] === 0;
    // 출발 칸 둘레는 비운다 (벽 옆에서 출발해도 빠져나오게)
    const si = s % NX, sj = Math.floor(s / NX);
    for (let dj = -2; dj <= 2; dj++) for (let di = -2; di <= 2; di++) {
      const k = (sj + dj) * NX + si + di;
      if (k >= 0 && k < NX * NY && blocked[k] === 1) blocked[k] = 0;
    }
    let goal = g;
    if (!free(goal)) {                                              // 막힌 목표 → 가장 가까운 빈 칸
      let best = -1, bd = 1e9;
      const gi = g % NX, gj = Math.floor(g / NX);
      for (let dj = -8; dj <= 8; dj++) for (let di = -8; di <= 8; di++) {
        const k = (gj + dj) * NX + gi + di;
        if (k >= 0 && k < NX * NY && free(k) && Math.hypot(di, dj) < bd) { bd = Math.hypot(di, dj); best = k; }
      }
      if (best < 0) return null;
      goal = best;
    }
    const gI = goal % NX, gJ = Math.floor(goal / NX);
    const gs = new Float32Array(NX * NY).fill(Infinity), came = new Int32Array(NX * NY).fill(-1), closed = new Uint8Array(NX * NY);
    const heap = [];                                                // [f, k]
    const push = (f, k) => { heap.push([f, k]); let c = heap.length - 1; while (c > 0) { const p = (c - 1) >> 1; if (heap[p][0] <= heap[c][0]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let c = 0; for (;;) { const l = 2 * c + 1, r = l + 1; let m = c; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === c) break; [heap[m], heap[c]] = [heap[c], heap[m]]; c = m; } } return top; };
    const h = (k) => Math.hypot(k % NX - gI, Math.floor(k / NX) - gJ);
    gs[s] = 0; push(h(s), s);
    const nb = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
    let n = 0;
    while (heap.length && n++ < 60000) {
      const [, k] = pop();
      if (closed[k]) continue;
      if (k === goal) break;
      closed[k] = 1;
      const i = k % NX, j = Math.floor(k / NX);
      for (const [di, dj, c] of nb) {
        const a = i + di, b = j + dj;
        if (a < 0 || b < 0 || a >= NX || b >= NY) continue;
        const q = b * NX + a;
        if (!free(q) || closed[q]) continue;
        if (di && dj && (!free(j * NX + a) || !free(b * NX + i))) continue;   // 모서리 자르기 금지
        const ng = gs[k] + c;
        if (ng < gs[q]) { gs[q] = ng; came[q] = k; push(ng + h(q), q); }
      }
    }
    if (came[goal] < 0 && goal !== s) return null;
    const cells = [];
    for (let k = goal; k >= 0; k = came[k]) { cells.push(k); if (k === s) break; }
    cells.reverse();
    const pts = cells.map((k) => [X0 + (k % NX + 0.5) * CELL, Y0 + (Math.floor(k / NX) + 0.5) * CELL]);
    return prune(pts);
  }
  function lineFree(a, b) {
    const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / (CELL * 0.5));
    for (let s = 0; s <= n; s++) { const k = idx(a[0] + (b[0] - a[0]) * s / n, a[1] + (b[1] - a[1]) * s / n); if (k < 0 || blocked[k]) return false; }
    return true;
  }
  function prune(pts) {                                             // 눈에 보이는 점까지 건너뛴다
    if (pts.length < 3) return pts;
    const out = [pts[0]];
    let i = 0;
    while (i < pts.length - 1) {
      let j = pts.length - 1;
      while (j > i + 1 && !lineFree(pts[i], pts[j])) j--;
      out.push(pts[j]);
      i = j;
    }
    return out;
  }

  function replan(pose, now = 0) {
    inflate();
    const p = astar(pose.x, pose.y, st.goal[0], st.goal[1]);
    // 지금 길이 사람 등으로 잠깐 막혀 훨씬 먼 길이 나오면 4 s 까지 기다린다 (되돌아갔다 왔다 하지 않게)
    const here = [pose.x, pose.y];
    const newLen = p ? pathLen([here, ...p]) : Infinity;
    const curLen = st.path.length ? pathLen([here, ...st.path]) : Infinity;
    if (isFinite(curLen) && newLen > Math.max(1.5 * curLen, curLen + 4)) {
      if (!st.waitSince) st.waitSince = now;
      if (now - st.waitSince < 4000) {
        st.wait = true;
        st.status = `앞이 막혀 기다림 ${((now - st.waitSince) / 1000).toFixed(1)} s (돌아가는 길 ${isFinite(newLen) ? newLen.toFixed(1) + ' m' : '없음'})`;
        return;
      }
    }
    st.wait = false; st.waitSince = 0;
    st.path = p || [];
    st.status = p ? `경로 ${p.length} 점 · ${pathLen(p).toFixed(1)} m` : '경로 없음 — 막혔다';
    pathGeo.setFromPoints([[pose.x, pose.y], ...st.path].map(([x, y]) => new THREE.Vector3(x, y, 0.03)));
  }
  const pathLen = (p) => p.reduce((s, q, i) => i ? s + Math.hypot(q[0] - p[i - 1][0], q[1] - p[i - 1][1]) : 0, 0);
  function pathBlocked(pose) {
    let prev = [pose.x, pose.y], acc = 0;
    for (const q of st.path) {
      if (!lineFree(prev, q)) return true;
      acc += Math.hypot(q[0] - prev[0], q[1] - prev[1]);
      if (acc > 2.5) break;
      prev = q;
    }
    return false;
  }

  // ---- 미리 보기 — 명령 (v, w) 로 T 초 가면 차 윤곽이 점유 칸 (장애물 그 자체) 에 닿는가
  const perim = [];
  {
    const x0 = bb.min[0] - 0.05, x1 = bb.max[0] + 0.05, y0 = bb.min[1] - 0.05, y1 = bb.max[1] + 0.05;   // 여유 5 cm
    for (let x = x0; x <= x1; x += 0.08) perim.push([x, y0], [x, y1]);
    for (let y = y0; y <= y1; y += 0.08) perim.push([x0, y], [x1, y]);
  }
  function collides(pose, v, w, T = 0.5) {
    let x = pose.x, y = pose.y, yaw = pose.yaw;
    const n = 5, dt = T / n;
    for (let i = 0; i < n; i++) {
      x += v * Math.cos(yaw) * dt; y += v * Math.sin(yaw) * dt; yaw += w * dt;
      const c = Math.cos(yaw), s = Math.sin(yaw);
      for (const [u, q] of perim) { const k = idx(x + u * c - q * s, y + u * s + q * c); if (k >= 0 && hits[k] >= 2) return true; }
    }
    return false;
  }
  // 후보 명령을 0.8 s 굴려 보고 안 닿는 것 중 점수가 가장 높은 것 (간이 DWA). 점수 = 앞 볼 점과 거리 · 방향 + 속도
  function choose(pose, tgt, pref) {
    const T = 0.8, n = 8, dt = T / n;
    let best = null, bs = -1e9;
    for (const v of [0.8, 0.5, 0.25, 0.0, -0.2]) {
      for (const w of [-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2]) {
        if (v === 0 && w === 0) continue;
        if (collides(pose, v, w, T)) continue;
        let x = pose.x, y = pose.y, yaw = pose.yaw;
        for (let i = 0; i < n; i++) { x += v * Math.cos(yaw) * dt; y += v * Math.sin(yaw) * dt; yaw += w * dt; }
        let err = Math.atan2(tgt[1] - y, tgt[0] - x) - yaw;
        err = Math.atan2(Math.sin(err), Math.cos(err));
        const d = Math.hypot(tgt[0] - x, tgt[1] - y);
        const sc = -2.0 * d - 0.8 * Math.abs(err) + 0.3 * v - (v < 0 ? 0.6 : 0) - 0.3 * Math.abs(w - pref[1]) / 1.2;
        if (sc > bs) { bs = sc; best = [v, w]; }
      }
    }
    return best;
  }
  function safe(pose, cmd, tgt) {
    if (!collides(pose, cmd[0], cmd[1])) return cmd;
    const alt = tgt ? choose(pose, tgt, cmd) : null;
    if (alt) { st.status = '피하는 중'; return alt; }
    st.status = '막혀서 기다림';
    return [0, 0];
  }

  // ---- 추종 — 반환 [v, w]
  const V_AUTO = 0.8, W_AUTO = 1.2, LD = 0.6;
  function command(pose, now, dt, gapAhead) {
    if (!st.active || !st.goal) return [0, 0];
    const dg = Math.hypot(st.goal[0] - pose.x, st.goal[1] - pose.y);
    if (dg < 0.15) { st.active = false; st.reached = true; st.status = '도착'; return [0, 0]; }
    if (st.backT > 0) { st.backT -= dt; return safe(pose, [-0.25, 0]); }   // 끼었을 때 잠깐 뒤로
    if (now - st.lastPlan > 500 || !st.path.length || (pathBlocked(pose) && now - st.lastPlan > 150)) { st.lastPlan = now; replan(pose, now); }
    if (!st.path.length || st.wait) return [0, 0];
    while (st.path.length > 1 && Math.hypot(st.path[0][0] - pose.x, st.path[0][1] - pose.y) < 0.2) st.path.shift();
    // 앞을 보는 점 — 경로를 따라 LD 만큼
    let tgt = st.path[st.path.length - 1], prev = [pose.x, pose.y], left = LD;
    for (const q of st.path) {
      const d = Math.hypot(q[0] - prev[0], q[1] - prev[1]);
      if (d >= left) { const t = left / d; tgt = [prev[0] + (q[0] - prev[0]) * t, prev[1] + (q[1] - prev[1]) * t]; break; }
      left -= d; prev = q;
    }
    let al = Math.atan2(tgt[1] - pose.y, tgt[0] - pose.x) - pose.yaw;
    al = Math.atan2(Math.sin(al), Math.cos(al));
    let v, w;
    if (Math.abs(al) > 0.8) { v = 0; w = Math.sign(al) * W_AUTO; }          // 크게 틀어졌으면 제자리 회전
    else {
      v = V_AUTO * Math.cos(al) * Math.min(1, dg / 0.8 + 0.25);
      if (gapAhead < 0.6) v *= Math.max(0, (gapAhead - 0.08) / 0.52);       // 지나갈 띠 안 장애물이 범퍼 앞 0.6 m 안이면 줄이고 8 cm 에서 멈춤
      w = Math.max(-W_AUTO, Math.min(W_AUTO, 2 * Math.max(v, 0.25) * Math.sin(al) / LD));
    }
    // 끼임 감지 — 2 초 동안 거의 안 움직이면 뒤로 0.6 s 후 다시 짠다
    if (!st.lastPos || Math.hypot(pose.x - st.lastPos[0], pose.y - st.lastPos[1]) > 0.05 || Math.abs(w) > 0.3) { st.lastPos = [pose.x, pose.y]; st.stuckT = 0; }
    else if ((st.stuckT += dt) > 2.0) { st.stuckT = 0; st.backT = 0.6; st.path = []; }
    return safe(pose, [v, w], tgt);
  }

  function draw() {
    if (!grid.visible) return;
    for (let k = 0; k < NX * NY; k++) {
      const o = 4 * k, h = hits[k];
      const j = Math.floor(k / NX), i = k % NX;
      const kk = ((NY - 1 - j) * NX + i) * 4;                       // 캔버스 위쪽 = +y
      if (h >= 2) img.data.set([20, 20, 24, 210], kk);
      else if (blocked[k]) img.data.set([255, 146, 43, 70], kk);
      else img.data.set([0, 0, 0, 0], kk);
      void o;
    }
    g2.putImageData(img, 0, 0);
    tex.needsUpdate = true;
  }

  return {
    st, addRays, addPoints, clear, command, draw, decay,
    setGoal(x, y, pose) { st.goal = [x, y]; st.active = true; st.reached = false; st.lastPlan = 0; st.path = []; st.wait = false; st.waitSince = 0; flag.position.set(x, y, 0); flag.visible = true; if (pose) replan(pose); },
    stop() { st.active = false; st.status = '멈춤'; },
    setMapVisible: (on) => { grid.visible = on; },
    INFL,
  };
}
