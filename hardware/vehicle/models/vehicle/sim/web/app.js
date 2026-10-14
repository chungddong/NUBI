// 차량 시뮬레이터 — 스키드 스티어 주행 · 시험 코스 · 센서 · 전원 추정
//   ① 사각지대 지도 (coverage.js) ② 전복 · 무게중심 (stability.js) ③ 비상정지 방식 비교 ④ 자율주행 (nav.js)
// 좌표: 지면 z=0, 앞 +x, 왼 +y, 위 +z, 단위 m (web_export.py 가 mm → m 로 옮겨 둔다).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { makeCourse } from './course.js';
import { makeSensors } from './sensors.js';
import { makePower } from './power.js';
import { makeCoverage } from './coverage.js';
import { makeStability } from './stability.js';
import { makeNav } from './nav.js';
import { makeHallway } from './hallway_env.js';
import { makeAgent } from './agent.js';
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

// 광선 투사 가속 (BVH) — 복도는 물체가 443 개라 그냥 쏘면 시뮬레이터 1 s 에 실제 2.5 s 가 걸렸다
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;

const ENV = new URLSearchParams(location.search).get('env') === 'hallway' ? 'hallway' : 'course';
// 카메라 — 기본은 차체 설계의 Gemini 336 앞 · 뒤. ?cam=xtion 이면 지금 실차의 Xtion Pro Live 앞 하나 (화각 58° × 45°,
// 0.5 m 안쪽은 깊이 없음 = blind, 4 m 까지). 렌즈 자리는 설계의 앞 카메라 자리를 그대로 쓴다.
const CAM = new URLSearchParams(location.search).get('cam') === 'xtion' ? 'xtion' : 'gemini';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const $ = (id) => document.getElementById(id);

const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
view.prepend(renderer.domElement);

const scene = new THREE.Scene();
const SKY_DAY = new THREE.Color(0xbfd3e6), SKY_NIGHT = new THREE.Color(0x0b0e14);
scene.background = SKY_DAY.clone();
scene.fog = new THREE.Fog(SKY_DAY.clone(), 18, 45);

const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 200);
camera.position.set(-2.2, -2.0, 1.6);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.2);
controls.enableDamping = true;

const hemi = new THREE.HemisphereLight(0xffffff, 0x6b6f62, 0.9);
scene.add(hemi);
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(-6, -4, 10);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 1, far: 30 });
scene.add(sun, sun.target);

// ------------------------------------------------------------------ 데이터 · 차체
const data = await (await fetch('data/vehicle.json')).json();
if (CAM === 'xtion') {
  data.cameras = [{ ...data.cameras[0], name: '앞 깊이 카메라', model: 'ASUS Xtion Pro Live', hfov: 58, vfov: 45,
    near: 0.05, blind: 0.5, ideal: [0.8, 3.5], far: 4.0 }];
}
const bin = new Float32Array(await (await fetch('data/vehicle.bin')).arrayBuffer());
$('loading').remove();
$('ver').innerHTML = `${data.version} — ${data.title}<br>${data.date} · ${data.summary}`;

const car = new THREE.Group();
scene.add(car);
const wheelPivots = [];
const mats = new Map();
for (const w of data.wheels) {
  const p = new THREE.Group();
  p.position.set(w.x, w.y, w.z);
  p.userData = { ...w, angle: 0 };
  car.add(p);
  wheelPivots.push(p);
}
function matFor(color, role) {
  const key = color + role;
  if (!mats.has(key)) mats.set(key, new THREE.MeshStandardMaterial({ color, metalness: role === 'body' ? 0.15 : 0.05, roughness: 0.65 }));
  return mats.get(key);
}
for (const p of data.parts) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(bin.slice(p.start * 9, (p.start + p.count) * 9), 3));
  let parent = car;
  if (p.role === 'wheel') {                                  // 가장 가까운 바퀴 피벗에 붙이고 축 기준으로 옮긴다
    const cx = (p.min[0] + p.max[0]) / 2, cy = (p.min[1] + p.max[1]) / 2;
    let best = wheelPivots[0], bd = 1e9;
    for (const w of wheelPivots) { const d = Math.hypot(w.position.x - cx, w.position.y - cy); if (d < bd) { bd = d; best = w; } }
    g.translate(-best.position.x, -best.position.y, -best.position.z);
    parent = best;
  }
  g.computeVertexNormals();
  const mat = ['headlight', 'status'].includes(p.role) ? new THREE.MeshStandardMaterial({ color: p.color, emissive: 0x000000 }) : matFor(p.color, p.role);
  const mesh = new THREE.Mesh(g, mat);
  mesh.castShadow = mesh.receiveShadow = true;
  mesh.userData = p;
  parent.add(mesh);
}

// ------------------------------------------------------------------ 모듈
const course = ENV === 'hallway' ? await makeHallway(scene) : makeCourse(scene);
if (ENV === 'hallway') { scene.fog.near = 40; scene.fog.far = 120; }
// 움직이지 않는 것은 월드 좌표로 한 메시에 합쳐 BVH 하나로 (광선마다 물체 수백 개를 훑지 않게). 사람만 따로.
{
  const dyn = new Set(course.movers.map((w) => w.m));
  const geos = [];
  for (const m of course.solids) {
    if (dyn.has(m)) continue;
    m.updateMatrixWorld(true);
    const g = m.geometry.index ? m.geometry.toNonIndexed() : m.geometry.clone();
    for (const k of Object.keys(g.attributes)) if (k !== 'position') g.deleteAttribute(k);
    g.applyMatrix4(m.matrixWorld);
    geos.push(g);
  }
  const merged = new THREE.Mesh(mergeGeometries(geos), new THREE.MeshBasicMaterial({ side: THREE.DoubleSide }));
  merged.visible = false;
  merged.updateMatrixWorld(true);
  merged.geometry.computeBoundsTree();
  for (const m of dyn) m.geometry.computeBoundsTree();
  course.solids = [merged, ...dyn];
}
const sensors = makeSensors({ scene, renderer, car, data, course, THREE });
const power = makePower(data);
const cover = makeCoverage({ THREE, car, data });
const stab = makeStability({ THREE, car, data });
const nav = makeNav({ THREE, scene, data, bounds: course.bounds });

const bb = data.bbox;
const foot = { x0: bb.min[0], x1: bb.max[0], y0: bb.min[1], y1: bb.max[1] };
const wr = data.wheel_radius;
const G = 9.81;
// ③ 비상정지 방식 — 상선 단락 제동 계수 K = 4 · Kt² / (r² · R선간) [가정: R선간 0.3 Ω] → 힘 = K · v
const K_SHORT = 4 * data.power.kt ** 2 / (wr * wr * 0.3);
const ESTOP_MODES = {
  cut: '즉시 차단 (지금 D-05) — 컨택터가 바퀴 전원을 바로 끊는다',
  brake_cut: '정지 후 차단 — 보드가 0.5 s 브레이크를 건 뒤 끊는다',
  hold: '정지 유지 — 전원은 두고 보드가 속도 0 을 잡는다 (Panther 식)',
  short: '차단 + 상선 단락 제동 — 끊을 때 릴레이가 모터 세 상을 묶는다',
};

// ------------------------------------------------------------------ 상태
const st = {
  x: 0, y: 0, yaw: 0, z: 0, pitch: 0, roll: 0, v: 0, w: 0, a: 0,
  estop: false, estopAt: 0, estopMode: 'cut', mode: 'idle', imax: 0, t: 0,
  vmax: 1.5, wmax: 1.5, lights: false, night: false, tipped: false,
};
const START = course.spots.start;
function place(spot) {
  Object.assign(st, { x: spot.x, y: spot.y, yaw: spot.yaw, v: 0, w: 0, tipped: false });
  updatePose();
}

// ------------------------------------------------------------------ 입력
const keys = new Set();
addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  const k = e.key.toLowerCase();
  if (k === ' ') { toggleEstop(); e.preventDefault(); return; }
  if (k === 'r') { place(START); return; }
  keys.add(k);
  if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) e.preventDefault();
  if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k) && nav.st.active) nav.stop();   // 손으로 잡으면 자율 끔
});
addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
const ext = { v: null, w: null };                            // __sim.drive · 시험이 넣는 명령

function command(dt, now) {
  if (ext.v !== null) return [ext.v, ext.w];
  if (nav.st.active) return nav.command(st, now, dt, sensors.stats().gapAhead);
  let f = 0, t = 0;
  if (keys.has('w') || keys.has('arrowup')) f += 1;
  if (keys.has('s') || keys.has('arrowdown')) f -= 1;
  if (keys.has('a') || keys.has('arrowleft')) t += 1;
  if (keys.has('d') || keys.has('arrowright')) t -= 1;
  return [f * st.vmax, t * st.wmax];
}

function toggleEstop(force) {
  const on = force === undefined ? !st.estop : force;
  if (on === st.estop) return;
  st.estop = on;
  st.estopAt = st.t;
  $('estop').classList.toggle('on', st.estop);
  $('estop').textContent = st.estop ? '비상정지 풀기' : '비상정지';
  if (on) nav.stop();
  toast(on ? `비상정지 — ${ESTOP_MODES[st.estopMode].split(' — ')[0]}` : '', on ? 'bad' : '');
}
$('estop').onclick = () => toggleEstop();
$('reset').onclick = () => place(START);
$('vmax').oninput = (e) => { st.vmax = +e.target.value; $('vmaxv').textContent = `${st.vmax.toFixed(1)} m/s`; };
$('wmax').oninput = (e) => { st.wmax = +e.target.value; $('wmaxv').textContent = `${st.wmax.toFixed(1)} rad/s`; };
// 환경 · 출발점 — 환경마다 출발점 단추를 만든다
$('env').value = ENV;
const goTo = (env, cam) => { location.search = [env === 'hallway' ? 'env=hallway' : '', cam === 'xtion' ? 'cam=xtion' : ''].filter(Boolean).map((s, i) => (i ? '&' : '?') + s).join(''); };
$('env').onchange = (e) => goTo(e.target.value, CAM);
$('camSel').value = CAM;
$('camSel').onchange = (e) => goTo(ENV, e.target.value);
$('spots').innerHTML = Object.keys(course.spots).map((k) => `<button data-go="${k}">${course.names[k] ?? k}</button>`).join('');
$('envNote').textContent = ENV === 'hallway' ? `${course.title} — ${course.subtitle}. 벽 · 난간 · 고정 가구와 충돌, 사람 2 명이 복도를 오간다. 천장은 내 화면에서만 숨김 (차 카메라 · 깊이에는 있음).`
  : '상자 · 사람 크기 기둥 · 좁은 통로 (폭 0.8 m) · 턱 20 / 40 mm · 경사로 10° / 15° · 옆 경사 20°. 턱은 바퀴 반지름의 40 % 까지만 넘는다고 본다 (어림).';
document.querySelectorAll('[data-go]').forEach((b) => b.onclick = () => place(course.spots[b.dataset.go]));
if (course.ceilLayer) {
  $('ceilRow').hidden = false;
  $('ceil').onchange = (e) => (e.target.checked ? camera.layers.enable(course.ceilLayer) : camera.layers.disable(course.ceilLayer));
}
if (!course.ramps.length) { $('estopTest').disabled = true; $('estopTest').title = '시험 코스 (경사로) 에서만'; }
let camMode = 'orbit';
document.querySelectorAll('[data-cam]').forEach((b) => b.onclick = () => {
  camMode = b.dataset.cam;
  document.querySelectorAll('[data-cam]').forEach((x) => x.classList.toggle('on', x === b));
});
for (const id of ['frustum', 'points', 'pip', 'lidar']) $(id).onchange = (e) => sensors.toggle(id, e.target.checked);
$('lights').onchange = (e) => { st.lights = e.target.checked; sensors.setLights(st.lights); };
$('night').onchange = (e) => setNight(e.target.checked);
function setNight(on) {
  st.night = on;
  scene.background.copy(on ? SKY_NIGHT : SKY_DAY);
  scene.fog.color.copy(on ? SKY_NIGHT : SKY_DAY);
  hemi.intensity = on ? 0.06 : 0.9;
  sun.intensity = on ? 0.0 : 1.6;
}

// ① 사각지대
function refreshCover() {
  const s = cover.compute(+$('covH').value);
  const f = (v) => v === null ? '못 봄' : `${(v * 100).toFixed(0)} cm`;
  $('covStats').innerHTML = `높이 ${(s.H * 100).toFixed(0)} cm 장애물 — 차 둘레 1 m 안 사각 <b>${s.blindNear.toFixed(2)} m²</b> / ${s.nearArea.toFixed(2)} m²<br>`
    + `처음 보이는 거리 (차 윤곽에서): ` + Object.entries(s.first).map(([k, v]) => `${k} ${f(v)}`).join(' · ');
}
$('cover').onchange = (e) => { cover.setVisible(e.target.checked); if (e.target.checked) refreshCover(); };
$('covH').onchange = refreshCover;

// ② 전복 · 무게중심
function refreshStab() {
  const p = { m: +$('pm').value, h: +$('ph').value, x: +$('px').value };
  $('pmv').textContent = `${p.m} kg`; $('phv').textContent = `${(p.h * 1000).toFixed(0)} mm`; $('pxv').textContent = `${(p.x * 1000).toFixed(0)} mm`;
  const r = stab.set(p);
  power.setMass(r.m);
  $('stabStats').innerHTML = `전체 ${r.m.toFixed(1)} kg · 무게중심 x ${(r.com[0] * 1000).toFixed(0)} · z <b>${(r.com[2] * 1000).toFixed(0)}</b> mm<br>`
    + `전복각 옆 <b>${r.side.toFixed(1)}°</b> · 앞 ${r.fwd.toFixed(1)}° · 뒤 ${r.back.toFixed(1)}° — 미끄러짐 ${r.slide.toFixed(1)}° (μ 0.6)`;
}
for (const id of ['pm', 'ph', 'px']) $(id).oninput = refreshStab;
$('stabShow').onchange = (e) => stab.setVisible(e.target.checked);
refreshStab();

// ③ 비상정지 방식
$('estopMode').onchange = (e) => { st.estopMode = e.target.value; $('estopDesc').textContent = ESTOP_MODES[st.estopMode]; };
$('estopDesc').textContent = ESTOP_MODES.cut;
$('estopTest').onclick = () => { $('estopTable').innerHTML = estopTable(runEstopTest()); };

// ④ 자율주행
let picking = false;
$('navPick').onclick = () => { picking = !picking; $('navPick').classList.toggle('on', picking); toast(picking ? '바닥을 눌러 목표를 찍는다' : '', 'info', 2500); };
$('navGo').onclick = () => { if (nav.st.goal) { toggleEstop(false); nav.setGoal(...nav.st.goal, st); } };
$('navStop').onclick = () => nav.stop();
$('navClear').onclick = () => nav.clear();
$('navMap').onchange = (e) => nav.setMapVisible(e.target.checked);
$('movers').onchange = (e) => course.setMovers(e.target.checked);
const picker = new THREE.Raycaster(), ndc = new THREE.Vector2();
let downXY = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downXY = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!picking || !downXY || Math.hypot(e.clientX - downXY[0], e.clientY - downXY[1]) > 5) return;
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  picker.setFromCamera(ndc, camera);
  const hit = picker.intersectObjects(course.solids, false)[0];
  if (!hit) return;
  picking = false;
  $('navPick').classList.remove('on');
  toggleEstop(false);
  nav.setGoal(hit.point.x, hit.point.y, st);
  toast('', '');
});

let toastTimer = 0;
function toast(msg, cls = 'info', ms = 0) {
  const t = $('toast');
  t.textContent = msg;
  t.className = msg ? cls : '';
  clearTimeout(toastTimer);
  if (msg && ms) toastTimer = setTimeout(() => { t.className = ''; }, ms);
}

// ------------------------------------------------------------------ 자세 · 충돌
function wheelWorld(w, x, y, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [x + w.x * c - w.y * s, y + w.x * s + w.y * c];
}
function terrainPose(x, y, yaw) {
  const h = data.wheels.map((w) => course.height(...wheelWorld(w, x, y, yaw)));
  const front = (h[2] + h[3]) / 2, rear = (h[0] + h[1]) / 2;     // 바퀴 순서: 뒤오른 · 뒤왼 · 앞오른 · 앞왼
  const left = (h[1] + h[3]) / 2, right = (h[0] + h[2]) / 2;
  return { z: (front + rear) / 2, pitch: Math.atan2(front - rear, data.wheelbase), roll: Math.atan2(left - right, data.track), h };
}
function footprint(x, y, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [[foot.x0, foot.y0], [foot.x1, foot.y0], [foot.x1, foot.y1], [foot.x0, foot.y1], [0, foot.y0], [0, foot.y1],
          [foot.x1, 0], [foot.x0, 0]].map(([u, v]) => [x + u * c - v * s, y + u * s + v * c]);
}
function blocked(x, y, yaw, prevH) {
  const hit = course.hits(footprint(x, y, yaw));
  if (hit) return hit;
  const pose = terrainPose(x, y, yaw);
  for (let i = 0; i < 4; i++) {
    if (pose.h[i] - prevH[i] > 0.4 * wr) return `턱 ${(1000 * (pose.h[i] - prevH[i])).toFixed(0)} mm — 바퀴 반지름의 40 % (${(400 * wr).toFixed(0)} mm) 넘음`;
  }
  return null;
}

// ------------------------------------------------------------------ 한 스텝
function step(dt, now = performance.now()) {
  course.update(dt, footprint(st.x, st.y, st.yaw).slice(0, 4));
  const feed = sensors.takeFeed();
  if (feed) { nav.addRays(feed.rays); nav.addPoints(feed.points, feed.ground); }
  nav.decay(dt);
  const [vc, wc] = st.estop ? [0, 0] : command(dt, now);
  const pose0 = terrainPose(st.x, st.y, st.yaw);
  let v = st.v, w = st.w, hold = false;
  const gAlong = -G * Math.sin(pose0.pitch);                    // 앞 (+x) 방향 중력 가속
  const coast = (vv, extra = 0) => {                            // 바퀴 전원 없이 굴러감 (구름 저항 · 단락 제동)
    if (Math.abs(vv) < 1e-3 && Math.abs(gAlong) <= G * data.power.f_roll) return 0;
    const a = gAlong - Math.sign(vv || gAlong) * G * data.power.f_roll - extra * vv;
    const nv = vv + a * dt;
    return (Math.sign(nv) !== Math.sign(vv) && Math.abs(gAlong) <= G * data.power.f_roll) ? 0 : nv;
  };
  const brake = (vv) => Math.sign(vv) * Math.max(0, Math.abs(vv) - 2.5 * dt);   // 보드 능동 제동 2.5 m/s² [가정]
  if (st.estop) {
    const te = st.t - st.estopAt;
    w = 0;
    if (st.estopMode === 'cut') v = coast(v);
    else if (st.estopMode === 'brake_cut') v = te < 0.5 ? brake(v) : coast(v);
    else if (st.estopMode === 'hold') { v = brake(v); hold = Math.abs(v) < 1e-3; if (hold) v = 0; }
    else v = coast(v, K_SHORT / stab.get().m);
    st.mode = 'estop';
  } else {
    const A_MAX = 1.0, AL_MAX = 3.0;                            // 가속 한계 [가정]
    v += THREE.MathUtils.clamp(vc - v, -A_MAX * dt, A_MAX * dt);
    w += THREE.MathUtils.clamp(wc - w, -AL_MAX * dt, AL_MAX * dt);
    const moving = Math.abs(vc) + Math.abs(wc) > 0 || Math.abs(v) + Math.abs(w) > 0.01;
    st.mode = nav.st.active ? 'auto' : moving ? 'drive' : 'idle';
  }
  const nyaw = st.yaw + w * dt;
  const nx = st.x + v * Math.cos(st.yaw + w * dt / 2) * dt;
  const ny = st.y + v * Math.sin(st.yaw + w * dt / 2) * dt;
  const why = st.tipped ? '넘어짐' : blocked(nx, ny, nyaw, pose0.h);
  st.a = st.estop || dt <= 0 ? 0 : THREE.MathUtils.clamp((v - st.v) / dt, -1.0, 1.0);
  if (why) {
    v = 0; w = 0; st.a = 0;
    if (!st.tipped) toast(`멈춤 — ${why}`, 'bad', 1500);
  } else {
    st.x = nx; st.y = ny; st.yaw = nyaw;
  }
  st.v = v; st.w = w;
  for (const p of wheelPivots) {                               // 왼 (+y) 바퀴 = v − w·윤거/2, 오른 = v + w·윤거/2
    p.userData.angle += (p.userData.y > 0 ? v - w * data.track / 2 : v + w * data.track / 2) * dt / wr;
    p.rotation.y = p.userData.angle;
  }
  updatePose();
  const chk = stab.check({ roll: st.roll, pitch: st.pitch, v, w });
  if (chk.tipMargin < 0 && !st.tipped) { st.tipped = true; toast(`넘어진다 — 기울기 ${chk.side.toFixed(1)}° 가 전복각 ${stab.get().side.toFixed(1)}° 를 넘었다`, 'bad'); }
  const pw = power.update({ v, w, a: st.a, pitch: st.pitch, estop: st.estop && !hold, lights: st.lights, hold, dt });
  st.imax = Math.max(st.imax, pw.ibat);
  st.t += dt;
  return { pw, chk };
}

function updatePose() {
  const p = terrainPose(st.x, st.y, st.yaw);
  st.z = p.z; st.pitch = p.pitch; st.roll = p.roll;
  car.position.set(st.x, st.y, st.z);
  car.rotation.set(0, 0, 0);
  car.rotateZ(st.yaw);
  car.rotateY(-st.pitch);
  car.rotateX(st.roll);
}

// ③ 비상정지 시험 — 10° 경사에서 0.8 m/s 로 오르다 · 내려가다 비상정지, 4 초 뒤까지 본다
function runEstopTest() {
  const save = { ...st };
  const r10 = course.ramps.find((r) => r.name === 'ramp10');
  const out = {};
  for (const mode of Object.keys(ESTOP_MODES)) {
    out[mode] = {};
    for (const [nm, x0] of [['오르막', r10.x0 + 0.45], ['내리막', r10.x2 + 0.45]]) {
      Object.assign(st, { x: x0, y: (r10.y0 + r10.y1) / 2, yaw: 0, v: 0.8, w: 0, tipped: false, estop: false, estopMode: mode });
      updatePose();
      ext.v = 0.8; ext.w = 0;
      for (let i = 0; i < 18; i++) step(1 / 60);
      st.estop = true; st.estopAt = st.t;
      const xs = st.x;
      let xmax = st.x, xmin = st.x, tStop = null;
      for (let i = 0; i < 240; i++) {
        step(1 / 60);
        xmax = Math.max(xmax, st.x); xmin = Math.min(xmin, st.x);
        if (tStop === null && Math.abs(st.v) < 0.01) tStop = (i + 1) / 60;
      }
      // 앞으로 더 간 거리 · 거기서 뒤로 굴러 내려온 거리 · 4 s 동안 움직인 거리 · 끝 속도
      out[mode][nm] = { forward: xmax - xs, back: xmax - st.x, xEnd: st.x - xs, vEnd: st.v, tStop };
      void xmin;
    }
  }
  ext.v = null;
  Object.assign(st, save, { estopMode: $('estopMode').value });
  updatePose();
  return out;
}
function estopTable(res) {
  const f = (v) => `${(v * 100).toFixed(0)} cm`;
  let h = '<table class="kv small"><tr><td>방식</td><td>오르막: 더 간 · 뒤로 굴러감 (4 s)</td><td>내리막: 4 s 동안 간 거리 · 끝 속도</td></tr>';
  for (const [m, r] of Object.entries(res)) {
    const up = r['오르막'], dn = r['내리막'];
    h += `<tr><td>${ESTOP_MODES[m].split(' — ')[0]}</td><td>${f(up.forward)} · <b>${f(up.back)}</b></td>`
      + `<td><b>${f(dn.xEnd)}</b> · ${Math.abs(dn.vEnd).toFixed(2)} m/s</td></tr>`;
  }
  return h + '</table>';
}

// ------------------------------------------------------------------ 패널
function setBar(id, frac, bad) {
  const b = $(id);
  b.querySelector('i').style.width = `${Math.min(100, frac * 100).toFixed(0)}%`;
  b.classList.toggle('bad', !!bad);
}
const LAMP = { idle: ['', 'a', ''], drive: ['', '', 'g'], auto: ['', '', 'g'], estop: ['r', '', ''] };
const MODE_KO = { idle: '대기 (노랑)', drive: '수동 주행 (초록)', auto: '자율 주행 (초록 깜빡)', estop: '비상정지 (빨강 깜빡 · 부저)' };
function panel({ pw, chk }) {
  const deg = (r) => (r * 180 / Math.PI).toFixed(1);
  $('kv_v').textContent = `${st.v.toFixed(2)} m/s · ${st.w.toFixed(2)} rad/s`;
  $('kv_tilt').textContent = `${deg(st.pitch)}° · ${deg(st.roll)}°`;
  $('kv_ip').textContent = `${pw.iphase.toFixed(1)} A`;
  setBar('bar_ip', pw.iphase / data.power.i_phase_max, pw.iphase > data.power.i_phase_max);
  $('kv_ib').textContent = `${pw.ibat.toFixed(1)} A`;
  setBar('bar_ib', pw.ibat / data.power.bms_max_a, pw.ibat > data.power.bms_max_a);
  $('kv_p').textContent = `${pw.p.toFixed(0)} W (바퀴 ${pw.pmotor.toFixed(0)} · 전자 ${pw.pelec.toFixed(0)})`;
  $('kv_t').textContent = pw.p > 1 ? `${(data.power.wh * 0.8 / pw.p).toFixed(1)} h` : '—';
  $('kv_imax').textContent = `${st.imax.toFixed(1)} A`;
  $('pwrNote').textContent = pw.iphase > data.power.i_phase_max ? '상전류 한도 15 A 를 넘는다 — 실제로는 펌웨어가 토크를 깎아 그만큼 못 낸다' : '';
  $('stabLive').innerHTML = `지금 옆 기울기 ${chk.side.toFixed(1)}° (회전 원심 ${chk.lat.toFixed(1)}° 포함) · 앞뒤 ${chk.fb.toFixed(1)}° — `
    + `전복 여유 <b style="color:${chk.tipMargin < 10 ? '#d9480f' : '#2b8a3e'}">${chk.tipMargin.toFixed(1)}°</b> · 미끄럼 여유 ${chk.slideMargin.toFixed(1)}°`;
  const blink = (performance.now() % 600) < 300;
  const on = LAMP[st.mode];
  const lit = (k) => k && (st.mode === 'estop' || st.mode === 'auto' ? blink : true);
  $('lr').style.background = lit(on[0]) ? '#e03131' : '#3a3f45';
  $('la').style.background = lit(on[1]) ? '#f59f00' : '#3a3f45';
  $('lg').style.background = lit(on[2]) ? '#2f9e44' : '#3a3f45';
  $('mode').textContent = MODE_KO[st.mode];
  sensors.setStatus(st.mode === 'auto' ? (blink ? 'drive' : 'off') : st.mode, blink);
  $('navStatus').textContent = `${nav.st.active ? '자율 주행 중 — ' : ''}${nav.st.status}${nav.st.goal ? ` · 목표 (${nav.st.goal[0].toFixed(1)}, ${nav.st.goal[1].toFixed(1)}) 까지 ${Math.hypot(nav.st.goal[0] - st.x, nav.st.goal[1] - st.y).toFixed(1)} m` : ''}`;
  $('hud').innerHTML = `x ${st.x.toFixed(2)} · y ${st.y.toFixed(2)} m · 방향 ${(st.yaw * 180 / Math.PI).toFixed(0)}° · ${MODE_KO[st.mode]}<br>`
    + `배터리 전류 ${pw.ibat.toFixed(1)} A · 상전류 ${pw.iphase.toFixed(1)} A${st.estop ? ' · <b style="color:#ff6b6b">비상정지</b>' : ''}`;
}
function vehTable() {
  const rows = [
    ['질량 (어림, 추가 모듈 빼고)', `${data.mass.toFixed(1)} kg`],
    ['무게중심 높이', `${(data.com[2] * 1000).toFixed(0)} mm`],
    ['외곽 (바퀴 포함)', `${((bb.max[0] - bb.min[0]) * 1000).toFixed(0)} × ${((bb.max[1] - bb.min[1]) * 1000).toFixed(0)} × ${(bb.max[2] * 1000).toFixed(0)} mm`],
    ['윤거 · 휠베이스', `${(data.track * 1000).toFixed(0)} · ${(data.wheelbase * 1000).toFixed(0)} mm`],
    ['바퀴 지름', `${(2000 * wr).toFixed(0)} mm (6.5")`],
    ['배터리', `36 V ${data.power.wh} Wh · BMS ${data.power.bms_max_a} A`],
  ];
  $('veh').innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');
}
vehTable();

// ------------------------------------------------------------------ 시점 · 렌더
function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

const lastCar = new THREE.Vector3(), tmp = new THREE.Vector3();
function updateCamera() {
  const c = Math.cos(st.yaw), s = Math.sin(st.yaw);
  if (camMode === 'chase') {
    camera.position.set(st.x - 2.2 * c, st.y - 2.2 * s, st.z + 1.3);
    controls.target.set(st.x + 0.5 * c, st.y + 0.5 * s, st.z + 0.25);
  } else if (camMode === 'top') {
    camera.position.set(st.x - 0.01, st.y, 7.5);
    controls.target.set(st.x, st.y, 0);
  } else {
    tmp.set(st.x, st.y, st.z).sub(lastCar);
    camera.position.add(tmp);
    controls.target.add(tmp);
  }
  lastCar.set(st.x, st.y, st.z);
  sun.position.set(st.x - 6, st.y - 4, 10);
  sun.target.position.set(st.x, st.y, 0);
}

let last = performance.now(), lastOut = step(0);
place(START);
lastCar.set(st.x, st.y, st.z);
let lastMapDraw = 0;
function frame(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  lastOut = step(dt, now);
  updateCamera();
  controls.update();
  sensors.update(now);
  if (now - lastMapDraw > 300) { nav.draw(); lastMapDraw = now; }
  panel(lastOut);
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, view.clientWidth, view.clientHeight);
  renderer.render(scene, camera);
  sensors.renderPip(view.clientWidth, view.clientHeight);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

// 검증용 (브라우저 패널이 숨어 rAF 가 멈춰도 돌릴 수 있게)
window.__sim = {
  st, data, course, sensors, power, cover, stab, nav,
  drive(v, w, sec, dt = 1 / 60) {
    ext.v = v; ext.w = w;
    let o;
    for (let t = 0; t < sec; t += dt) o = step(dt);
    ext.v = null;
    sensors.update(performance.now(), true);
    return { x: st.x, y: st.y, yaw: st.yaw, z: st.z, pitch: st.pitch, roll: st.roll, v: st.v, ibat: o.pw.ibat, iphase: o.pw.iphase, imax: st.imax, tipMargin: o.chk.tipMargin };
  },
  run(sec, dt = 1 / 60) {                       // 자율주행 등 지금 명령 그대로 sec 동안 (센서도 120 ms 마다)
    let t0 = performance.now(), o;
    for (let t = 0; t < sec; t += dt) {
      const now = t0 + t * 1000;
      if (Math.round(t / dt) % 7 === 0) sensors.update(now, true);
      o = step(dt, now);
    }
    return { x: st.x, y: st.y, yaw: st.yaw, mode: st.mode, nav: nav.st.status, reached: nav.st.reached };
  },
  place: (name) => place(course.spots[name]),
  estop: (on) => toggleEstop(on),
  estopTest: () => runEstopTest(),
  goal: (x, y) => nav.setGoal(x, y, st),
};
// ⑤ GPT 운전 (카메라만) — agent.js · ../agent.py
window.__sim.agent = makeAgent({ sim: window.__sim, sensors, course, st, place, $, data });

// ⑥ 실차 운전 프로그램과 연결 (?bridge=1) — drive.py --sim 이 가상 ESP32 의 자세를 보내면 차를 그 자리에 두고,
//    부탁한 카메라 (Xtion 컬러 · 깊이) 를 3D 지도에서 그려 돌려준다. 바퀴 명령 · 위치 추정은 왼쪽 아래에 보인다 (serve.py).
if (new URLSearchParams(location.search).get('bridge') === '1') {
  if (course.setMovers) course.setMovers(false);              // 가상 ESP32 는 벽만 안다 — 사람은 끈다
  const box = document.createElement('div');
  box.style.cssText = 'position:fixed;left:12px;bottom:12px;z-index:20;max-width:46vw;padding:8px 12px;border-radius:8px;'
    + 'background:rgba(20,20,20,.78);color:#f2f2f2;font:13px/1.5 system-ui,"Malgun Gothic",sans-serif;white-space:pre-wrap';
  box.textContent = '연결 대기 — drive.py --sim 을 띄우면 여기서 움직인다';
  document.body.appendChild(box);
  const b64 = (u8) => { let t = ''; for (let i = 0; i < u8.length; i += 0x8000) t += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000)); return btoa(t); };
  (async function bridgeLoop() {
    for (;;) {
      let job;
      try {
        job = await (await fetch('/api/bridge/next')).json();
      } catch (e) {
        box.textContent = '시뮬레이터 서버와 연결이 끊김';
        await new Promise((r) => setTimeout(r, 1000));
        continue;
      }
      if (!job.seq) continue;
      if (job.pose) {
        Object.assign(st, { x: job.pose.x, y: job.pose.y, yaw: job.pose.yaw, v: 0, w: 0 });
        updatePose();
      }
      if (job.status) box.textContent = job.status;
      if (job.views && job.views.length) {
        const views = {};
        for (const v of job.views) {
          const r = sensors.renderView(v);
          views[v.name] = { jpg: r.jpg, depth: b64(new Uint8Array(r.depth.buffer)), w: r.w, h: r.h };
        }
        await fetch('/api/bridge/result', { method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ seq: job.seq, views }) });
      }
    }
  })();
}
