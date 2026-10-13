// 센서 — 뎁스 카메라 시야 · 깊이 점 (광선 투사) · 앞 카메라 화면 (PiP) · 2D 라이다 · 전조등 · 상태 표시등
export function makeSensors({ scene, renderer, car, data, course, THREE }) {
  const D2R = Math.PI / 180;
  const ray = new THREE.Raycaster();
  ray.firstHitOnly = true;                       // BVH: 가장 가까운 것 하나만
  const show = { frustum: true, points: true, pip: true, lidar: true };

  // ---------------------------------------------------------------- 카메라 시야 (차체에 붙인다)
  const frusta = [];
  const camRigs = [];
  for (const c of data.cameras) {
    const rig = new THREE.Group();
    rig.position.set(c.x, c.y, c.z);
    rig.rotation.z = c.yaw * D2R;
    car.add(rig);
    camRigs.push({ c, rig });
    const tx = Math.tan(c.hfov / 2 * D2R), tz = Math.tan(c.vfov / 2 * D2R);
    const grp = new THREE.Group();
    const corners = (d) => [[d, d * tx, d * tz], [d, -d * tx, d * tz], [d, -d * tx, -d * tz], [d, d * tx, -d * tz]];
    // 이상 범위 (0.3–3 m) 반투명 상자
    const [a0, a1] = c.ideal;
    const n = corners(a0), f = corners(a1);
    const pos = [];
    const quad = (p, q, r, s) => pos.push(...p, ...q, ...r, ...p, ...r, ...s);
    for (let i = 0; i < 4; i++) quad(n[i], f[i], f[(i + 1) % 4], n[(i + 1) % 4]);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    grp.add(new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color: c.yaw === 0 ? 0x2f7de1 : 0x9c36b5, transparent: true,
      opacity: 0.10, side: THREE.DoubleSide, depthWrite: false })));
    // 테두리 선 (렌즈 → far)
    const far = corners(c.far);
    const lp = [];
    for (let i = 0; i < 4; i++) lp.push(0, 0, 0, ...far[i], ...far[i], ...far[(i + 1) % 4], ...f[i], ...f[(i + 1) % 4]);
    const lg = new THREE.BufferGeometry();
    lg.setAttribute('position', new THREE.Float32BufferAttribute(lp, 3));
    grp.add(new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ color: c.yaw === 0 ? 0x2f7de1 : 0x9c36b5,
      transparent: true, opacity: 0.55 })));
    rig.add(grp);
    frusta.push(grp);
  }

  // ---------------------------------------------------------------- 깊이 점 (월드 좌표)
  const NX = 44, NZ = 30;
  const maxPts = NX * NZ * data.cameras.length;
  const ptsGeo = new THREE.BufferGeometry();
  ptsGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(maxPts * 3), 3));
  ptsGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(maxPts * 3), 3));
  const ptsObj = new THREE.Points(ptsGeo, new THREE.PointsMaterial({ size: 0.035, vertexColors: true }));
  ptsObj.frustumCulled = false;
  scene.add(ptsObj);
  const col = new THREE.Color();
  const o = new THREE.Vector3(), dvec = new THREE.Vector3(), q = new THREE.Quaternion(), nW = new THREE.Vector3();
  let stats = { front: 0, rear: 0, near: Infinity, nearFront: Infinity, nearRear: Infinity, gapAhead: Infinity, gapBehind: Infinity };
  // 차가 지나갈 띠 (차 반폭 + 5 cm) 안의 장애물만 — 범퍼에서 잰 거리 (자율주행이 멈출지 정한다)
  const bbx = data.bbox, halfW = Math.max(-bbx.min[1], bbx.max[1]) + 0.05;
  const loc = new THREE.Vector3();
  const feed = { rays: [], points: [], ground: [], fresh: false };   // 자율주행 지도로 넘길 것 (nav.js)
  // 장애물 면인가 — 맞은 면의 법선이 거의 수평 (벽 · 상자 옆 · 턱 옆) 이고 바닥에서 3 cm 위
  function isObstacle(hit) {
    if (hit.point.z <= 0.03 || !hit.face) return false;
    nW.copy(hit.face.normal).transformDirection(hit.object.matrixWorld);
    return Math.abs(nW.z) < 0.7;
  }

  function castDepth() {
    const P = ptsGeo.attributes.position.array, C = ptsGeo.attributes.color.array;
    let k = 0, nearest = Infinity, gapA = Infinity, gapB = Infinity;
    const counts = [], nears = [];
    feed.points = [];
    feed.ground = [];
    for (const { c, rig } of camRigs) {
      let camNear = Infinity;
      rig.getWorldPosition(o);
      rig.getWorldQuaternion(q);
      const tx = Math.tan(c.hfov / 2 * D2R), tz = Math.tan(c.vfov / 2 * D2R);
      let n = 0;
      for (let i = 0; i < NX; i++) {
        for (let j = 0; j < NZ; j++) {
          const u = (i / (NX - 1) * 2 - 1) * tx, w = (j / (NZ - 1) * 2 - 1) * tz;
          dvec.set(1, u, w).normalize().applyQuaternion(q);
          ray.set(o, dvec);
          ray.near = c.near; ray.far = c.far;
          const hit = ray.intersectObjects(course.solids, false)[0];
          if (!hit) continue;
          const dist = hit.distance * Math.cos(Math.atan(Math.hypot(u, w)));   // 깊이 = 광축 방향 거리
          P[3 * k] = hit.point.x; P[3 * k + 1] = hit.point.y; P[3 * k + 2] = hit.point.z;
          const inIdeal = dist >= c.ideal[0] && dist <= c.ideal[1];
          col.setHSL(Math.min(dist / c.far, 1) * 0.66, 0.85, inIdeal ? 0.5 : 0.75);
          C[3 * k] = col.r; C[3 * k + 1] = col.g; C[3 * k + 2] = col.b;
          k++; n++;
          if (!isObstacle(hit)) { if (hit.point.z <= 0.03 || (nW.z > 0.9)) feed.ground.push([hit.point.x, hit.point.y]); }
          else {
            feed.points.push([hit.point.x, hit.point.y]);
            loc.copy(hit.point);
            car.worldToLocal(loc);
            if (Math.abs(loc.y) <= halfW) {
              if (loc.x > bbx.max[0]) gapA = Math.min(gapA, loc.x - bbx.max[0]);
              else if (loc.x < bbx.min[0]) gapB = Math.min(gapB, bbx.min[0] - loc.x);
            }
            if (dist < camNear) camNear = dist;
            if (dist < nearest) nearest = dist;                         // 바닥 말고 장애물 중 가장 가까운 것
          }
        }
      }
      counts.push(n);
      nears.push(camNear);
    }
    ptsGeo.setDrawRange(0, k);
    ptsGeo.attributes.position.needsUpdate = true;
    ptsGeo.attributes.color.needsUpdate = true;
    stats = { front: counts[0] || 0, rear: counts[1] || 0, near: nearest, nearFront: nears[0], nearRear: nears[1], gapAhead: gapA, gapBehind: gapB };
    feed.fresh = true;
  }

  // ---------------------------------------------------------------- 라이다
  let lidar = null;
  if (data.lidar) {
    const L = data.lidar;
    const rig = new THREE.Group();
    rig.position.set(L.x, L.y, L.z);
    car.add(rig);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(L.rays * 3), 3));
    const pts = new THREE.Points(geo, new THREE.PointsMaterial({ color: 0xff2d2d, size: 0.05 }));
    pts.frustumCulled = false;
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(L.rays * 6), 3));
    const lines = new THREE.LineSegments(lineGeo, new THREE.LineBasicMaterial({ color: 0xff6b6b, transparent: true, opacity: 0.18 }));
    lines.frustumCulled = false;
    scene.add(pts, lines);
    lidar = { L, rig, geo, lineGeo, pts, lines, hits: 0 };
  }
  function castLidar() {
    if (!lidar) return;
    const { L, rig, geo, lineGeo } = lidar;
    rig.getWorldPosition(o);
    rig.getWorldQuaternion(q);
    const P = geo.attributes.position.array, Lp = lineGeo.attributes.position.array;
    let k = 0;
    feed.rays = [];
    for (let i = 0; i < L.rays; i++) {
      const a = 2 * Math.PI * i / L.rays;
      dvec.set(Math.cos(a), Math.sin(a), 0).applyQuaternion(q);
      ray.set(o, dvec);
      ray.near = 0.03; ray.far = L.range;
      const hit = ray.intersectObjects(course.solids, false)[0];
      if (!hit) { feed.rays.push([o.x, o.y, o.x + dvec.x * L.range, o.y + dvec.y * L.range, false]); continue; }
      feed.rays.push([o.x, o.y, hit.point.x, hit.point.y, hit.point.z > 0.03]);
      P[3 * k] = hit.point.x; P[3 * k + 1] = hit.point.y; P[3 * k + 2] = hit.point.z;
      Lp.set([o.x, o.y, o.z, hit.point.x, hit.point.y, hit.point.z], 6 * k);
      k++;
    }
    feed.fresh = true;
    geo.setDrawRange(0, k);
    lineGeo.setDrawRange(0, 2 * k);
    geo.attributes.position.needsUpdate = true;
    lineGeo.attributes.position.needsUpdate = true;
    lidar.hits = k;
  }

  // ---------------------------------------------------------------- 앞 카메라 화면 (PiP)
  const front = camRigs.find((r) => r.c.yaw === 0);
  const pipCam = new THREE.PerspectiveCamera(front.c.vfov, Math.tan(front.c.hfov / 2 * D2R) / Math.tan(front.c.vfov / 2 * D2R), 0.05, 40);
  pipCam.up.set(0, 0, 1);
  const look = new THREE.Vector3();
  function renderPip(w, h) {
    if (!show.pip) { document.getElementById('pipLabel').textContent = ''; return; }
    front.rig.getWorldPosition(o);
    front.rig.getWorldQuaternion(q);
    pipCam.position.copy(o);
    look.set(1, 0, 0).applyQuaternion(q).add(o);
    pipCam.up.set(0, 0, 1).applyQuaternion(q);
    pipCam.lookAt(look);
    const pw = Math.round(Math.min(360, w * 0.32)), ph = Math.round(pw / pipCam.aspect);
    const x = w - pw - 10, y = 30;
    const vis = ptsObj.visible, lv = lidar && lidar.pts.visible;
    ptsObj.visible = false;
    if (lidar) { lidar.pts.visible = false; lidar.lines.visible = false; }
    frusta.forEach((f) => { f.userData.v = f.visible; f.visible = false; });
    renderer.setScissorTest(true);
    renderer.setScissor(x, y, pw, ph);
    renderer.setViewport(x, y, pw, ph);
    renderer.render(scene, pipCam);
    renderer.setScissorTest(false);
    ptsObj.visible = vis;
    if (lidar) { lidar.pts.visible = lv; lidar.lines.visible = lv; }
    frusta.forEach((f) => { f.visible = f.userData.v; });
    document.getElementById('pipLabel').textContent = `앞 카메라 (${front.c.model}, 렌즈 높이 ${(front.c.z * 1000).toFixed(0)} mm)`;
    document.getElementById('pipLabel').style.bottom = `${y + ph + 4}px`;
  }

  // ---------------------------------------------------------------- 전조등
  const spots = [];
  for (const h of data.headlights) {
    const s = new THREE.SpotLight(0xfff4e0, 0, 18, h.beam / 2 * D2R, 0.35, 1.2);
    s.position.set(h.x, h.y, h.z);
    s.target.position.set(h.x + 1, h.y, h.z - 0.05);
    car.add(s, s.target);
    spots.push(s);
  }
  const lampMeshes = [], statusMeshes = [];
  car.traverse((m) => {
    if (m.isMesh && m.userData.role === 'headlight') lampMeshes.push(m);
    if (m.isMesh && m.userData.role === 'status') statusMeshes.push(m);
  });
  function setLights(on) {
    spots.forEach((s) => { s.intensity = on ? 9 : 0; });
    lampMeshes.forEach((m) => m.material.emissive.set(on ? 0xfff2d0 : 0x000000));
  }
  const STATUS_COL = { 빨강: 0xff2d2d, 노랑: 0xffb000, 초록: 0x2fd05a };
  function setStatus(mode, blink) {
    for (const m of statusMeshes) {
      const nm = m.userData.name || '';
      const key = Object.keys(STATUS_COL).find((k) => nm.includes(k));
      if (!key) continue;
      const on = (mode === 'estop' && key === '빨강' && blink) || (mode === 'idle' && key === '노랑') || (mode === 'drive' && key === '초록');
      m.material.emissive.set(on ? STATUS_COL[key] : 0x000000);
    }
  }

  // ---------------------------------------------------------------- 갱신
  let lastCast = 0;
  function update(now, force = false) {
    frusta.forEach((f) => { f.visible = show.frustum; });
    ptsObj.visible = show.points;
    if (lidar) { lidar.pts.visible = show.lidar; lidar.lines.visible = show.lidar; }
    if (force || now - lastCast > 120) {
      lastCast = now;
      castDepth();                                                  // 보이기와 상관없이 잰다 (자율주행 지도가 쓴다)
      castLidar();
      const near = isFinite(stats.near) ? `${stats.near.toFixed(2)} m` : '없음';
      document.getElementById('sensNote').textContent =
        `깊이 점 앞 ${stats.front} · 뒤 ${stats.rear} (색 = 거리, 진한 색 = 이상 범위 0.3~3 m) · 가장 가까운 장애물 ${near}`
        + (lidar ? ` · 라이다 반사 ${lidar.hits}/${lidar.L.rays}` : '');
    }
  }
  document.getElementById('lidarNote').textContent = data.lidar ? `(${data.lidar.model}, 스캔 높이 ${(data.lidar.z * 1000).toFixed(0)} mm)` : '(아직 없음)';
  if (!data.lidar) document.getElementById('lidar').disabled = true;

  return {
    toggle: (id, on) => { show[id] = on; },
    update, renderPip, setLights, setStatus,
    stats: () => ({ ...stats, lidarHits: lidar ? lidar.hits : null }),
    takeFeed: () => { if (!feed.fresh) return null; feed.fresh = false; return { rays: feed.rays, points: feed.points, ground: feed.ground }; },
  };
}
