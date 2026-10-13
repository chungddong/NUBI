// ⑤ GPT · Claude 운전 (카메라만) — 앞 카메라 사진에 깊이로 잰 갈 수 있는 방향을 번호 화살표로 그려 모델에 보내고,
// 모델이 고른 행동 (화살표로 가기 · 제자리 돌기 · 멈추기) 을 차가 한다. VLMnav (Goetting 외, CoRL 2024) 방식.
// '지도 · 위치' 를 켜면 위에서 본 지도 그림 + 문 좌표 + **추정** 위치 (오도메트리 드리프트 · 시각 위치 인식 오차를 흉내)
// 를 같이 준다. 실물도 지도는 갖고 가지만 지금 위치는 추정뿐이므로, 진짜 위치는 '정답' 모드 (비교용) 와 채점에만 쓴다.
// 서버 쪽은 ../agent.py (GPT · 모의) · ../agent_claude.py (Claude) — API 키 · 프롬프트는 서버에만.
// observe() · act(결정) 을 따로 부르면 서버 없이 다른 운전자 (사람, 콘솔) 도 같은 사진 · 실행 · 채점으로 몰 수 있다.

// 화살표 (도, + = 왼쪽, 번호 1 = 가장 왼쪽) 와 사진 폭은 앞 카메라에 맞춘다 — Gemini 336 (90°, RGB 1280 급) ·
// Xtion Pro Live (58°, 640 × 480). 실차 experiments/2026-10-13_vlm-drive/depth_view.py 의 ARROWS_XTION 과 같다.
const arrowsFor = (hfov) => (hfov <= 60 ? [24, 16, 8, 0, -8, -16, -24] : [40, 27, 13, 0, -13, -27, -40]);
const BAND0 = 0.65;                               // 사각 판정 띠 시작 (m) — 화살표 방향 ± atan(반폭 / 0.65) 안의 사각이 막는다
const HALF_W = 0.32;                              // 차 반폭 + 여유 (m) — 이 띠 안의 장애물만 화살표를 막는다
const MAX_MOVE = 2.0, MARGIN = 0.3, SHOW = 3.0;   // 한 번에 가는 최대 · 장애물 앞 여유 · 화살표 끝 최대 거리
const GOAL_R = 1.0;                               // 목표 문 앞 1 m 안이면 성공 (채점)
const D2R = Math.PI / 180;
// 지도 · 위치 모드 — 실물에서 바퀴 오도메트리 · IMU · SfM 위치 인식이 할 몫
const POSE = {
  none: '안 줌 (사진만)',
  map: '지도만 (위치 모름)',
  odom: '지도 + 오도메트리 (출발점에서 쌓음, 드리프트)',
  vis: '지도 + 오도메트리 + 시각 위치 인식 (흉내)',
  truth: '지도 + 정답 위치 (비교용 상한)',
};
const ODO_SCALE = 0.03, ODO_STEP = 0.02, IMU_DEG = 0.5;   // 바퀴 축척 오차 (주행마다) · 걸음 잡음 · IMU 방향 잡음 [가정]
const VIS_P = 0.8, VIS_M = 0.15, VIS_DEG = 3;             // 시각 위치 인식: 걸음마다 성공 확률 · 위치 · 방향 오차 [가정]
const MAP_W = 1280;
const gauss = () => Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());

export function makeAgent({ sim, sensors, course, st, place, $, data }) {
  const doors = course.doors;
  const cam = data.cameras.find((c) => c.yaw === 0);
  const ARROWS = arrowsFor(cam.hfov), IMG_W = cam.hfov <= 60 ? 640 : 1280;
  const camText = `${cam.model}, 렌즈 높이 ${(cam.z * 100).toFixed(0)} cm, 수평 시야 ${cam.hfov}°, 세로 시야 ${cam.vfov}°, `
    + `깊이 ${cam.blind ?? cam.near} ~ ${cam.far} m` + (cam.blind ? ` (${cam.blind} m 안쪽은 깊이가 없어 '사각' 으로 친다)` : '');
  const goalSel = $('agGoal');
  if (!doors) {
    goalSel.innerHTML = '<option>복도 환경에서만</option>';
    for (const id of ['agRun', 'agOne', 'agStop', 'agGoal']) $(id).disabled = true;
    $('agStat').textContent = '환경을 복도로 바꾸면 쓸 수 있다.';
    return null;
  }
  const targets = doors.filter((d) => d.label || d.x > 30);
  goalSel.innerHTML = targets.map((d) => `<option value="${d.id}">${d.name}</option>`).join('');
  goalSel.value = targets.find((d) => d.label === '310')?.id ?? targets[0].id;

  let running = false, busy = false, run = null, lastCands = null, est = null, lastPose = null, lastMap = null;
  $('agPose').innerHTML = Object.entries(POSE).map(([k, v]) => `<option value="${k}">${v}</option>`).join('');
  $('agPose').value = 'vis';

  // ---- 지도 그림 — 벽 · 고정 가구 · 문 (호수) · 목표 · 추정 위치. 지도 좌표 = 시뮬레이터 좌표 (x 복도 안쪽, y 왼쪽, m)
  const [BX0, BY0, BX1, BY1] = course.bounds, MS = MAP_W / (BX1 - BX0), MAP_H = Math.round((BY1 - BY0) * MS);
  const px = (x, y) => [(x - BX0) * MS, (BY1 - y) * MS];
  // 문 이름표 — 호수 + 앞 · 뒤 (같은 호수 문이 둘인 방), 호수 없는 라운지 문은 이름
  const tag = (d) => (d.label ? d.label + (d.name.includes('앞문') ? '앞' : d.name.includes('뒷문') ? '뒤' : '')
    : d.name.replace(/ 출입문.*$/, ''));
  function drawMap(goal, pose) {
    const cv = document.createElement('canvas');
    cv.width = MAP_W; cv.height = MAP_H;
    const g = cv.getContext('2d');
    const seg = (a, b) => { g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke(); };
    g.fillStyle = '#fff'; g.fillRect(0, 0, MAP_W, MAP_H);
    g.strokeStyle = '#e3e7eb'; g.lineWidth = 1; g.fillStyle = '#7d868f'; g.font = '13px sans-serif';
    for (let x = Math.ceil(BX0 / 5) * 5; x <= BX1; x += 5) { seg(px(x, BY0), px(x, BY1)); g.fillText(`x=${x}`, px(x, 0)[0] + 2, MAP_H - 4); }
    for (let y = Math.ceil(BY0 / 5) * 5; y <= BY1; y += 5) { seg(px(BX0, y), px(BX1, y)); g.fillText(`y=${y}`, 3, px(0, y)[1] - 3); }
    g.fillStyle = '#d5dade';
    for (const b of course.map.boxes) {
      g.beginPath(); b.corners.forEach(([x, y], i) => (i ? g.lineTo(...px(x, y)) : g.moveTo(...px(x, y)))); g.closePath(); g.fill();
    }
    g.strokeStyle = '#1f2328'; g.lineWidth = 3;
    for (const [ax, ay, bx, by] of course.map.segs) seg(px(ax, ay), px(bx, by));
    g.textAlign = 'center'; g.textBaseline = 'middle';
    for (const d of doors) {
      const isGoal = d.id === goal.id, [u, v] = px(d.x, d.y);
      g.fillStyle = isGoal ? '#e03131' : '#2f7de1';
      g.fillRect(u - 6, v - 6, 12, 12);
      g.font = isGoal ? 'bold 17px sans-serif' : '14px sans-serif';
      g.fillText(tag(d), ...px(d.x - 0.6 * d.nx, d.y - 0.6 * d.ny));          // 벽 바깥쪽 (복도 안 표시와 안 겹치게)
    }
    const [gu, gv] = px(...goal.front);
    g.strokeStyle = '#e03131'; g.lineWidth = 2; g.setLineDash([6, 4]);
    g.beginPath(); g.arc(gu, gv, GOAL_R * MS, 0, 2 * Math.PI); g.stroke(); g.setLineDash([]);
    g.fillStyle = '#e03131'; g.beginPath(); g.arc(gu, gv, 4, 0, 2 * Math.PI); g.fill();
    if (pose) {
      const [u, v] = px(pose.x, pose.y), a = -pose.yaw, r = 0.45 * MS;          // 화면 y 는 아래로
      g.fillStyle = 'rgba(47,125,225,0.15)'; g.beginPath(); g.arc(u, v, Math.max(pose.sigma * MS, 4), 0, 2 * Math.PI); g.fill();
      g.fillStyle = '#1c5bb0'; g.beginPath();
      g.moveTo(u + r * Math.cos(a), v + r * Math.sin(a));
      g.lineTo(u + 0.6 * r * Math.cos(a + 2.5), v + 0.6 * r * Math.sin(a + 2.5));
      g.lineTo(u + 0.6 * r * Math.cos(a - 2.5), v + 0.6 * r * Math.sin(a - 2.5));
      g.closePath(); g.fill();
    }
    return cv.toDataURL('image/png');
  }

  function mapText(goal, pose) {
    const L = ['지도 좌표: x = 복도 방향 (입구 방화문 근처 0, 안쪽으로 +), y = 왼쪽 +, 단위 m. 방향 0° = +x (복도 안쪽), 90° = +y (왼쪽).',
      '두 번째 사진이 위에서 본 지도다 (검은 선 = 벽, 회색 = 고정 가구, 파랑 네모 = 문과 호수, 빨강 = 목표 문 · 문 앞 1 m 원'
        + (pose ? ', 파랑 삼각형 = 추정 위치와 방향, 연한 원 = 추정 오차' : '') + ').',
      `목표: ${goal.name} — 문 (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}), 문 앞 점 (${goal.front[0].toFixed(2)}, ${goal.front[1].toFixed(2)}). 이 점에서 1 m 안에 멈추면 성공.`,
      '복도 문 (x 순서, 앞 · 뒤 = 앞문 · 뒷문): ' + doors.filter((d) => d.x < 30).map((d) => `${tag(d)} ${d.side} x=${d.x.toFixed(1)}`).join(', ')];
    L.push(pose ? `추정 위치: (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}), 방향 ${(pose.yaw / D2R).toFixed(0)}°, 오차 약 ±${pose.sigma.toFixed(2)} m (출처: ${pose.src}). 추정이라 틀릴 수 있다 — 사진과 맞춰 보라.`
      : '추정 위치는 없다 — 사진을 지도와 맞춰 지금 어디인지 추론하라.');
    return L.join('\n');
  }

  // ---- 위치 추정 — 출발점은 안다. 걸음마다 진짜 움직임에 오차를 얹어 쌓고, 'vis' 는 가끔 시각 위치 인식으로 바로잡는다
  function poseInit() {
    est = run.pose === 'odom' || run.pose === 'vis' ? { x: st.x, y: st.y, yaw: st.yaw, sigma: 0.05, src: '출발점' } : null;
    run.odoScale = 1 + ODO_SCALE * gauss();
  }
  function poseUpdate(before) {
    if (!est) return;
    const dx = st.x - before.x, dy = st.y - before.y, herr = wrap(est.yaw - before.yaw);
    const s = run.odoScale * (1 + ODO_STEP * gauss()), c = Math.cos(herr), sn = Math.sin(herr);
    est.x += s * (c * dx - sn * dy);                                    // 진짜 이동을 지금 방향 오차만큼 돌려 쌓는다
    est.y += s * (sn * dx + c * dy);
    est.yaw = wrap(est.yaw + wrap(st.yaw - before.yaw) + IMU_DEG * D2R * gauss());
    est.sigma = Math.min(3, est.sigma + 0.04 * Math.hypot(dx, dy) + 0.01);
    est.src = '오도메트리';
    if (run.pose === 'vis' && Math.random() < VIS_P) {
      est = { x: st.x + VIS_M * gauss(), y: st.y + VIS_M * gauss(), yaw: wrap(st.yaw + VIS_DEG * D2R * gauss()), sigma: VIS_M, src: '시각 위치 인식' };
    }
  }
  function currentPose() {
    if (run.pose === 'truth') return { x: st.x, y: st.y, yaw: st.yaw, sigma: 0, src: '정답 (비교용)' };
    return est ? { ...est } : null;
  }

  function directory() {
    const side = (s) => doors.filter((d) => d.side === s && d.x < 30).map((d) => d.name).join(' → ');
    const lounge = doors.filter((d) => d.x >= 30).map((d) => d.name).join(', ');
    return `복도 입구 (방화문) 에서 안쪽으로 들어가며 볼 때, 앞에서부터 차례로\n`
      + `- 왼쪽 벽: ${side('왼쪽')}\n- 오른쪽 벽: ${side('오른쪽')}\n`
      + `복도 끝 방화문을 지나면 라운지 (${lounge}). 문 위쪽에 검은 번호판, 몇 방은 벽에서 튀어나온 방 이름 표지판이 있다.`;
  }

  // 화살표마다 빈 거리 — 앞 카메라 세로줄의 가장 가까운 장애물을 그 방향 띠 (차 반폭) 에 비춰 본다
  function candidates() {
    const cols = sensors.frontCols();
    return ARROWS.map((deg, k) => {
      const th = deg * D2R;
      let free = cam.far, blind = false;
      for (const c of cols) {
        const along = c.free * Math.cos(c.ang - th), lat = c.free * Math.sin(c.ang - th);
        if (along > 0 && Math.abs(lat) < HALF_W) free = Math.min(free, along);
        if (c.blind && Math.abs(c.ang - th) <= Math.atan(HALF_W / BAND0)) blind = true;
      }
      if (blind) free = Math.min(free, 0.3);
      return { id: k + 1, deg, free: Math.round(free * 10) / 10, blind };
    });
  }

  // 바닥 위 방향 θ, 거리 r 의 점 → 사진 좌표 (광축 수평, 렌즈 높이 cam.z)
  function drawArrows(snap, cands) {
    const g = snap.canvas.getContext('2d');
    const { w, h, f, cam } = snap;
    g.lineCap = 'round';
    for (const c of cands) {
      const th = c.deg * D2R, r = Math.min(Math.max(c.free, 0.4), SHOW);
      const u = w / 2 - f * Math.tan(th), v = h / 2 + f * cam.z / (r * Math.cos(th));
      const color = c.free >= 1 ? '#2fbf4a' : c.free >= 0.5 ? '#f08c00' : '#e03131';
      g.strokeStyle = 'rgba(0,0,0,0.55)'; g.lineWidth = 9;
      g.beginPath(); g.moveTo(w / 2, h - 2); g.lineTo(u, v); g.stroke();
      g.strokeStyle = color; g.lineWidth = 5;
      g.beginPath(); g.moveTo(w / 2, h - 2); g.lineTo(u, v); g.stroke();
      g.fillStyle = color; g.strokeStyle = '#fff'; g.lineWidth = 3;
      g.beginPath(); g.arc(u, v, 20, 0, 2 * Math.PI); g.fill(); g.stroke();
      g.fillStyle = '#fff'; g.font = 'bold 24px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText(String(c.id), u, v + 1);
    }
  }

  const tick = () => new Promise((r) => setTimeout(r, 16));       // 화면이 따라 그리게
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

  async function turnBy(rad) {
    const y0 = st.yaw, target = y0 + rad;
    for (let i = 0; i < 300; i++) {
      const err = wrap(target - st.yaw);
      if (Math.abs(err) < 0.03) break;
      sim.drive(0, Math.sign(err) * Math.min(0.9, Math.max(0.25, 2 * Math.abs(err))), 0.05);
      await tick();
    }
    sim.drive(0, 0, 0.3);
    return wrap(st.yaw - y0);
  }
  async function forward(d) {
    const x0 = st.x, y0 = st.y;
    let blocked = null, still = 0;
    for (let i = 0; i < 400; i++) {
      const done = Math.hypot(st.x - x0, st.y - y0);
      if (d - done < 0.03) break;
      if (sensors.stats().gapAhead < 0.25) { blocked = '깊이로 앞이 막혀 섬'; break; }
      sim.drive(Math.min(0.6, Math.max(0.15, 1.5 * (d - done))), 0, 0.1);
      still = Math.hypot(st.x - x0, st.y - y0) - done < 0.002 ? still + 1 : 0;
      if (i > 5 && still > 3) { blocked = '부딪혀 섬'; break; }
      await tick();
    }
    sim.drive(0, 0, 0.5);
    return { moved: Math.hypot(st.x - x0, st.y - y0), blocked };
  }

  async function execute(d, cands) {
    if (d.action === 'move') {
      const c = cands.find((k) => k.id === d.arrow) ?? cands[3];
      const turned = await turnBy(c.deg * D2R);
      const dist = Math.max(0, Math.min(d.distance_m, MAX_MOVE, c.free - MARGIN));
      const r = await forward(dist);
      return `화살표 ${c.id} (${c.deg}°) 로 돌아 ${(turned / D2R).toFixed(0)}°, ${r.moved.toFixed(2)} m 감`
        + (dist < d.distance_m - 0.05 ? ` (요청 ${d.distance_m.toFixed(1)} m 를 빈 거리에 맞춰 줄임)` : '')
        + (r.blocked ? ` — ${r.blocked}` : '');
    }
    if (d.action === 'turn_left' || d.action === 'turn_right') {
      const deg = Math.min(Math.max(Math.abs(d.turn_deg), 5), 180) * (d.action === 'turn_left' ? 1 : -1);
      const turned = await turnBy(deg * D2R);
      return `제자리 ${turned > 0 ? '왼쪽' : '오른쪽'} ${Math.abs(turned / D2R).toFixed(0)}°`;
    }
    return '멈춤';
  }

  function score() {
    const goal = doors.find((d) => d.id === run.goal);
    const dist = (d) => Math.hypot(st.x - d.front[0], st.y - d.front[1]);
    const nearest = doors.reduce((a, b) => (dist(a) < dist(b) ? a : b));
    const e = dist(goal);
    return { ok: e <= GOAL_R, err: e, nearest: nearest.name, nearestErr: dist(nearest) };
  }

  function setStat() {
    if (!run) { $('agStat').textContent = ''; return; }
    $('agStat').textContent = `걸음 ${run.step} · 토큰 입력 ${run.tin.toLocaleString()} · 출력 ${run.tout.toLocaleString()}`
      + ` · 약 $${run.usd.toFixed(3)} · 마지막 응답 ${(run.lastMs / 1000).toFixed(1)} s`
      + (run.result ? ` — ${run.result}` : running ? ' · 운전 중' : '');
  }
  function log(html) { $('agLog').insertAdjacentHTML('afterbegin', `<div style="border-top:1px solid var(--line); padding:4px 0">${html}</div>`); }
  const esc = (s) => String(s).replace(/[&<>]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[ch]);

  function start(opts = {}) {
    const goal = doors.find((d) => d.id === goalSel.value);
    if ($('agStart').checked) place(course.spots.start);
    document.querySelector('[data-cam="chase"]')?.click();             // 차를 따라가며 보이게
    run = { id: new Date().toISOString().slice(0, 19), goal: goal.id, goalName: goal.name, model: $('agModel').value,
      effort: $('agEffort').value, directory: $('agDir').checked ? directory() : '', step: 0, maxSteps: 40,
      history: [], memory: '', tin: 0, tout: 0, usd: 0, lastMs: 0, result: '', pose: $('agPose').value, ...opts };
    poseInit();
    $('agLog').innerHTML = '';
    log(`<b>시작</b> — 목표 ${esc(goal.name)} · ${esc(run.model)} (${esc(run.effort)}) · 층 안내 ${run.directory ? '줌' : '안 줌'}`
      + ` · 지도 · 위치: ${esc(POSE[run.pose])}`);
  }

  function finish(why) {
    const s = score();
    run.result = `${why}: 목표 문 앞까지 ${s.err.toFixed(2)} m → ${s.ok ? '성공' : '실패'} (가장 가까운 문: ${s.nearest} ${s.nearestErr.toFixed(2)} m)`;
    log(`<b>${s.ok ? '✅' : '❌'} ${esc(run.result)}</b> <span style="color:#999">(채점만 진짜 위치를 씀)</span>`);
    running = false;
    setStat();
  }

  // 사진을 찍고 화살표를 그린다 — 서버로 보내든 다른 운전자가 보든 같은 것. 위치는 돌려주지 않는다.
  function observe() {
    if (!run || run.result) start();
    run.step++;
    sensors.update(performance.now(), true);
    const snap = sensors.snapFront(IMG_W);
    lastCands = candidates();
    drawArrows(snap, lastCands);
    $('agImg').src = snap.canvas.toDataURL('image/jpeg', 0.85);
    $('agImg').style.display = 'block';
    lastPose = currentPose();
    lastMap = null;
    if (run.pose !== 'none') {
      const goal = doors.find((d) => d.id === run.goal);
      if (lastPose) for (const c of lastCands) c.map_deg = Math.round(wrap(lastPose.yaw + c.deg * D2R) / D2R);
      lastMap = { image: drawMap(goal, lastPose), text: mapText(goal, lastPose) };
      $('agMap').src = lastMap.image;
    }
    $('agMap').style.display = lastMap ? 'block' : 'none';
    setStat();
    return { step: run.step, candidates: lastCands, memory: run.memory, history: run.history.slice(-8),
      map_text: lastMap?.text ?? null };
  }

  // 결정 (agent.py SCHEMA 모양) 을 실행하고 기록 · 채점한다
  async function act(d, ms = 0) {
    run.memory = d.memory;
    const what = d.action === 'move' ? `가기 ${d.arrow}번 ${d.distance_m.toFixed(1)} m`
      : d.action === 'stop' ? '멈춤' : `${d.action === 'turn_left' ? '왼쪽' : '오른쪽'} ${d.turn_deg.toFixed(0)}°`;
    const before = { x: st.x, y: st.y, yaw: st.yaw };
    const outcome = await execute(d, lastCands);
    poseUpdate(before);
    run.history.push(`걸음 ${run.step}: ${what} → ${outcome}`);
    const perr = lastPose ? ` · 이번 걸음 추정 위치 오차 ${Math.hypot(lastPose.x - before.x, lastPose.y - before.y).toFixed(2)} m (${lastPose.src})` : '';
    log(`<b>#${run.step} ${esc(what)}</b>${ms ? ` (${(ms / 1000).toFixed(1)} s)` : ''}<br>보임: ${esc(d.observation)}<br>`
      + `까닭: ${esc(d.reason)}<br>메모: ${esc(d.memory)}<br><span style="color:#999">${esc(outcome)}${esc(perr)}</span>`);
    if (d.action === 'stop') finish(`${run.model} 가 멈춤`);
    else if (run.step >= run.maxSteps) finish(`걸음 한계 ${run.maxSteps}`);
    setStat();
    return { outcome, done: !!run.result, result: run.result };
  }

  async function stepOnce() {
    if (busy) return;
    busy = true;
    try {
      observe();
      const req = { run: run.id, step: run.step, max_steps: run.maxSteps, model: run.model, effort: run.effort,
        goal: run.goalName, camera: camText, directory: run.directory, candidates: lastCands, history: run.history.slice(-8), memory: run.memory,
        image: $('agImg').src, map_image: lastMap?.image ?? null, map_text: lastMap?.text ?? null,
        truth: { x: st.x, y: st.y, yaw: st.yaw, est: lastPose, pose_mode: run.pose } };   // truth 는 서버 기록용 — 프롬프트에 안 넣는다
      const res = await fetch('/api/agent/step', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(req) }).then((r) => r.json());
      if (!res.ok) { log(`<b style="color:var(--warn)">오류</b> ${esc(res.error)}`); running = false; run.step--; return; }
      run.tin += res.usage.input; run.tout += res.usage.output; run.usd += res.usage.usd; run.lastMs = res.latency_ms;
      await act(res.decision, res.latency_ms);
    } catch (e) {
      log(`<b style="color:var(--warn)">오류</b> ${esc(e.message)}`);
      running = false;
    } finally {
      busy = false;
    }
  }

  async function loop() {
    while (running) await stepOnce();
    setStat();
  }
  $('agRun').onclick = () => { if (running) return; if (run?.result) run = null; running = true; loop(); };
  $('agOne').onclick = () => { if (!running) stepOnce(); };
  $('agStop').onclick = () => { running = false; setStat(); };
  for (const id of ['agGoal', 'agModel', 'agEffort', 'agDir', 'agStart', 'agPose']) $(id).onchange = () => { if (!running) run = null; };   // 설정을 바꾸면 다음 걸음은 새 주행
  return { start, observe, act, stepOnce, candidates, directory, get run() { return run; }, stop: () => { running = false; } };
}
