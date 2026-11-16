// 교내 공학 페스티벌 전시용 덱 — LoadGoesOn · 누비(NUBI)
// 디자인은 팀 초안 PDF(검정 바탕 · 노랑 · 시안, SYS.0N 머리말)를 따른다.
// 실행: 이 폴더에서  py prep_images.py  →  node build.js
const path = require("path");
const pptxgen = require("pptxgenjs");

const SKILL_THEME = process.env.APPLY_THEME_JS; // 선택: pptx 스킬의 apply_theme.js 경로
const OUT = path.join(__dirname, "LoadGoesOn_공학페스티벌_전시.pptx");
const IMG = (f) => path.join(__dirname, "images", f);

const COL = {
  bg: "0D0F14", panel: "151A23", panel2: "10161C", line: "2A3140",
  yellow: "F5E400", cyan: "22D3EE", red: "FF4D5E", text: "EDEFF3", muted: "9AA3B2", ink: "0D0F14",
};
const FONT = "Malgun Gothic";
const THEME = {
  name: "LoadGoesOn Neon", headFontFace: FONT, bodyFontFace: FONT,
  colors: {
    dk1: COL.bg, lt1: "FFFFFF", dk2: COL.panel, lt2: COL.text,
    accent1: COL.yellow, accent2: COL.cyan, accent3: COL.red, accent4: COL.muted,
    accent5: "3B82F6", accent6: "10B981", hlink: COL.cyan, folHlink: COL.muted,
  },
};
const TOTAL = 14;
const W = 13.333, M = 0.6;

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE";
pres.title = "LoadGoesOn — 자율주행 기반 다목적 모빌리티 플랫폼";
pres.author = "LoadGoesOn (국립목포대학교 컴퓨터공학과)";
pres.theme = { headFontFace: FONT, bodyFontFace: FONT };

// ---------- 레이아웃 ----------
pres.defineSlideMaster({
  title: "COVER",
  background: { color: COL.bg },
  objects: [],
});
pres.defineSlideMaster({
  title: "CONTENT",
  background: { color: COL.bg },
  objects: [
    { line: { x: M, y: 1.8, w: W - 2 * M, h: 0, line: { color: COL.cyan, width: 0.75, transparency: 55 } } },
    { text: { text: "LOADGOESON // AUTONOMOUS MOBILITY PLATFORM",
      options: { x: M, y: 6.95, w: 6, h: 0.3, margin: 0, fontFace: FONT, fontSize: 10, color: COL.cyan, charSpacing: 1 } } },
    { text: { text: "국립목포대학교 컴퓨터공학과",
      options: { x: W - M - 4, y: 6.95, w: 4, h: 0.3, margin: 0, fontFace: FONT, fontSize: 10, color: COL.muted, align: "right" } } },
    { placeholder: { options: { name: "title", type: "title", x: M, y: 0.92, w: W - 2 * M, h: 0.78, margin: 0,
      fontFace: FONT, fontSize: 30, bold: true, color: COL.yellow, valign: "middle", align: "left" }, text: "" } },
  ],
});

// ---------- 공통 조각 ----------
function header(s, n, label) {
  s.addShape(pres.shapes.PARALLELOGRAM, { x: M, y: 0.45, w: 1.1, h: 0.34, fill: { color: COL.yellow },
    line: { color: COL.yellow }, objectName: "SYS tag" });
  s.addText(`SYS.${String(n).padStart(2, "0")}`, { x: M, y: 0.45, w: 1.1, h: 0.34, margin: 0, isTextBox: true,
    fontSize: 13, bold: true, italic: true, color: COL.ink, align: "center", valign: "middle" });
  s.addText(`// ${label}`, { x: M + 1.3, y: 0.45, w: 6, h: 0.34, margin: 0, isTextBox: true,
    fontSize: 12, color: COL.cyan, charSpacing: 2, valign: "middle" });
  s.addText(`NODE ${String(n).padStart(2, "0")} / ${TOTAL}`, { x: W - M - 3, y: 0.45, w: 3, h: 0.34, margin: 0,
    isTextBox: true, fontSize: 11, color: COL.muted, align: "right", valign: "middle", charSpacing: 2 });
}

let node = 1; // 표지가 1
function slide(label, title, section) {
  const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: section });
  header(s, ++node, label);
  s.addText(title, { placeholder: "title", align: "left" });
  return s;
}

function panel(s, x, y, w, h, { border = COL.line, fill = COL.panel, name = "panel" } = {}) {
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { color: fill }, line: { color: border, width: 1 }, objectName: name });
}

// 사진 자리 — 팀 초안과 같은 '[ PHOTO ]' 표시. 사진을 받으면 이 상자를 그림으로 바꾼다.
function photo(s, x, y, w, h, tag, desc) {
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { color: COL.panel2 },
    line: { color: COL.cyan, width: 1.25, dashType: "dash" }, objectName: `PHOTO: ${desc}` });
  s.addText([
    { text: `[ ${tag} ]`, options: { fontSize: 13, bold: true, color: COL.cyan, charSpacing: 2, breakLine: true } },
    { text: desc, options: { fontSize: 13, color: COL.muted } },
  ], { x, y, w, h, isTextBox: true, align: "center", valign: "middle", margin: 0.1 });
}

function picture(s, file, x, y, w, h, alt) {
  s.addImage({ path: IMG(file), x, y, w, h, altText: alt });
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { type: "none" }, line: { color: COL.cyan, width: 1 },
    objectName: "image frame" });
}

// 사진 왼쪽 아래에 붙는 작은 이름표
function chip(s, text, x, y) {
  const w = 0.25 + text.length * 0.16;
  s.addShape(pres.shapes.RECTANGLE, { x, y, w, h: 0.34, fill: { color: COL.bg, transparency: 15 },
    line: { color: COL.bg, transparency: 15 }, objectName: "photo chip" });
  s.addText(text, { x, y, w, h: 0.34, margin: 0, isTextBox: true, fontSize: 12, bold: true, color: COL.text,
    align: "center", valign: "middle" });
}

function label(s, text, x, y, w, color = COL.cyan, size = 11) {
  s.addText(text, { x, y, w, h: 0.28, margin: 0, isTextBox: true, fontSize: size, bold: true, color, charSpacing: 2 });
}

function arrow(s, x1, y1, x2, y2, color = COL.cyan) {
  // 음수 폭 · 높이는 파일을 깨뜨리므로 뒤집기로 방향을 낸다
  s.addShape(pres.shapes.LINE, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1),
    flipH: x2 < x1, flipV: y2 < y1, line: { color, width: 2, endArrowType: "triangle" }, objectName: "arrow" });
}

// ---------- 1. 표지 ----------
pres.addSection({ title: "표지" });
{
  const s = pres.addSlide({ masterName: "COVER", sectionTitle: "표지" });
  s.addShape(pres.shapes.PARALLELOGRAM, { x: 0.75, y: 1.3, w: 2.0, h: 0.36, fill: { color: COL.yellow }, line: { color: COL.yellow } });
  s.addText("ENG.FEST 2026", { x: 0.75, y: 1.3, w: 2.0, h: 0.36, margin: 0, isTextBox: true, fontSize: 13, bold: true,
    italic: true, color: COL.ink, align: "center", valign: "middle" });
  s.addText("// SYSTEM ONLINE", { x: 2.95, y: 1.3, w: 3, h: 0.36, margin: 0, isTextBox: true, fontSize: 12,
    color: COL.cyan, charSpacing: 2, valign: "middle" });
  s.addText("LOADGOESON", { x: 0.75, y: 1.85, w: 6.3, h: 1.1, margin: 0, isTextBox: true, fontSize: 58, bold: true,
    color: COL.yellow, charSpacing: 2, valign: "middle" });
  s.addText("자율주행 기반\n다목적 모빌리티 플랫폼", { x: 0.75, y: 3.0, w: 6.3, h: 1.2, margin: 0, isTextBox: true,
    fontSize: 26, bold: true, color: COL.text, valign: "top", lineSpacingMultiple: 1.1 });
  s.addText("누구나 만들 수 있는 오픈 자율주행 로봇 · 누비 NUBI", { x: 0.75, y: 4.3, w: 6.3, h: 0.4, margin: 0, isTextBox: true,
    fontSize: 16, color: COL.cyan });
  s.addShape(pres.shapes.LINE, { x: 0.75, y: 4.9, w: 3.6, h: 0, line: { color: COL.cyan, width: 1, transparency: 40 } });
  s.addText("팀장 강동혁 · 팀원 서충만 김하민 문지현\n지도교수 정민아 · 국립목포대학교 컴퓨터공학과", {
    x: 0.75, y: 5.05, w: 6.3, h: 0.8, margin: 0, isTextBox: true, fontSize: 14, color: COL.muted, lineSpacingMultiple: 1.2 });
  picture(s, "cover_render.jpg", 6.95, 1.55, 5.8, 3.73, "누비 3D 렌더 — 브러시드 알루미늄 차체, 허브모터 바퀴 4개, 상부 모듈 레일");
  // 아래 경고 띠
  s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 6.72, w: W, h: 0.26, fill: { color: COL.yellow }, line: { color: COL.yellow }, objectName: "hazard band" });
  for (let x = -0.2; x < W; x += 0.42) {
    s.addShape(pres.shapes.PARALLELOGRAM, { x, y: 6.72, w: 0.32, h: 0.26, fill: { color: COL.ink }, line: { color: COL.ink }, objectName: "hazard stripe" });
  }
  s.addNotes("- LoadGoesOn 팀의 모듈형 자율주행 로봇 '누비'\n- Python 코드로 설계한 차체\n- 윗면 레일에 작업 모듈 교체 장착");
}

// ---------- 2. 문제 ----------
pres.addSection({ title: "문제와 접근" });
{
  const s = slide("PROBLEM_DETECTED", "이동 업무는 증가, 로봇은 고가", "문제와 접근");
  const tasks = [
    ["TASK_01", "운반 · 배달", "캠퍼스 · 건물 안 물건 이동"],
    ["TASK_02", "순찰", "정해진 경로 반복 감시"],
    ["TASK_03", "농업", "비닐하우스 고랑 사이 이동"],
  ];
  tasks.forEach(([tag, t, d], i) => {
    const x = M + i * 4.15;
    panel(s, x, 2.05, 3.85, 1.95);
    label(s, tag, x + 0.3, 2.3, 3.2);
    s.addText(t, { x: x + 0.3, y: 2.7, w: 3.3, h: 0.5, margin: 0, isTextBox: true, fontSize: 22, bold: true, color: COL.text });
    s.addText(d, { x: x + 0.3, y: 3.25, w: 3.3, h: 0.5, margin: 0, isTextBox: true, fontSize: 14, color: COL.muted });
  });
  panel(s, M, 4.3, W - 2 * M, 2.35, { border: COL.red, fill: "1A1116", name: "cost warning" });
  label(s, "!! WARNING // COST_OVERFLOW", M + 0.35, 4.55, 6, COL.red, 12);
  s.addText("라이다 배달로봇 한 대", { x: M + 0.35, y: 4.95, w: 5, h: 0.4, margin: 0, isTextBox: true, fontSize: 16, color: COL.text });
  s.addText("약 2,000만 원", { x: M + 0.35, y: 5.35, w: 5.6, h: 0.85, margin: 0, isTextBox: true, fontSize: 44, bold: true, color: COL.yellow });
  s.addText("업계 평균 (2021 보도)", { x: M + 0.35, y: 6.18, w: 5, h: 0.3, margin: 0, isTextBox: true, fontSize: 11, color: COL.muted });
  s.addShape(pres.shapes.LINE, { x: 6.55, y: 4.7, w: 0, h: 1.6, line: { color: COL.line, width: 1 } });
  s.addText("누비 한 대 부품비", { x: 6.95, y: 4.95, w: 5.5, h: 0.4, margin: 0, isTextBox: true, fontSize: 16, color: COL.text });
  s.addText("약 264만 원", { x: 6.95, y: 5.35, w: 5.5, h: 0.85, margin: 0, isTextBox: true, fontSize: 44, bold: true, color: COL.cyan });
  s.addText("약 1/8 가격", { x: 6.95, y: 6.18, w: 5.6, h: 0.3, margin: 0, isTextBox: true, fontSize: 13, bold: true, color: COL.cyan });
  s.addNotes("- 배달 · 순찰 · 농업 등 반복 이동 업무 다수\n- 라이다 배달로봇 한 대 약 2,000만 원 (2021 보도 업계 평균)\n- 누비 부품비 약 264만 원 — 약 1/8 수준");
}

// ---------- 3. 비전 전용 ----------
{
  const s = slide("SENSOR_OVERRIDE", "라이다 대신 카메라 기반 주행", "문제와 접근");
  panel(s, M, 2.05, 5.4, 1.35, { border: COL.line, fill: "12151B", name: "removed module" });
  label(s, "[ MODULE REMOVED ]", M + 0.3, 2.25, 4, COL.muted);
  s.addText([{ text: "3D LIDAR", options: { strike: "sngStrike", bold: true, fontSize: 22, color: COL.muted, breakLine: true } },
    { text: "고가 센서 의존 구조", options: { fontSize: 14, color: COL.muted } }],
    { x: M + 0.3, y: 2.55, w: 4.8, h: 0.8, margin: 0, isTextBox: true, valign: "top" });
  arrow(s, M + 2.7, 3.5, M + 2.7, 3.85, COL.yellow);
  panel(s, M, 3.95, 5.4, 1.5, { border: COL.cyan, name: "active module" });
  label(s, "[ ACTIVE ] VISION_ONLY", M + 0.3, 4.15, 4.5);
  s.addText([{ text: "깊이 카메라 + 온보드 AI", options: { bold: true, fontSize: 22, color: COL.text, breakLine: true } },
    { text: "저가 카메라 영상 처리로 주변을 인식", options: { fontSize: 14, color: COL.muted } }],
    { x: M + 0.3, y: 4.45, w: 4.8, h: 0.9, margin: 0, isTextBox: true, valign: "top" });
  s.addText([{ text: "MODULE_SWAP  ", options: { bold: true, color: COL.yellow } },
    { text: "모듈 레일 3줄로 작업 모듈 교체", options: { color: COL.text } }],
    { x: M, y: 5.7, w: 5.4, h: 0.85, margin: 0, isTextBox: true, fontSize: 14, valign: "top" });
  picture(s, "camera_compare.jpg", 6.35, 2.05, 6.38, 3.24, "깊이 카메라 Xtion과 HP60C의 컬러 영상과 깊이 영상 비교");
  s.addNotes("- 고가 3D 라이다 대신 깊이 카메라 + Jetson으로 주변 인식\n- 오른쪽: 실제 시험한 깊이 카메라 화면 (색 = 거리)\n- 모듈 레일로 적재함 · 센서 교체 → 용도 전환");
}

// ---------- 4. 구조 ----------
pres.addSection({ title: "시스템과 차체" });
{
  const s = slide("ARCHITECTURE", "인지 · 판단 · 구동 · 정지", "시스템과 차체");
  const blocks = [
    ["01 // SENSE", "센서", "깊이 카메라 ×2 (앞 · 뒤)\nIMU · GNSS · 2D 라이다\n바퀴 홀센서"],
    ["02 // THINK", "온보드 AI", "Jetson Orin Nano\n위치추정 · 경로 판단\n주행 녹화 · 상태 화면"],
    ["03 // ACT", "바퀴 제어기", "ESP32 (LOLIN D32)\n바퀴별 믹싱 · 가감속 제한\nWi-Fi 조종 페이지"],
    ["04 // DRIVE", "구동", "호버보드 모터 보드 ×4\n6.5\" 허브모터 ×4\n36 V 배터리 · 스키드 스티어"],
  ];
  const bw = 2.78, gap = 0.34;
  blocks.forEach(([tag, t, d], i) => {
    const x = M + i * (bw + gap);
    panel(s, x, 2.05, bw, 2.75, { border: i === 1 ? COL.yellow : COL.line });
    label(s, tag, x + 0.25, 2.27, bw - 0.4, i === 1 ? COL.yellow : COL.cyan);
    s.addText(t, { x: x + 0.25, y: 2.65, w: bw - 0.4, h: 0.5, margin: 0, isTextBox: true, fontSize: 22, bold: true, color: COL.text });
    s.addText(d, { x: x + 0.25, y: 3.25, w: bw - 0.4, h: 1.4, margin: 0, isTextBox: true, fontSize: 13, color: COL.muted,
      valign: "top", lineSpacingMultiple: 1.25 });
    if (i < 3) arrow(s, x + bw + 0.04, 3.42, x + bw + gap - 0.04, 3.42);
  });
  s.addText("USB 시리얼", { x: M + 2 * (bw + gap) - 0.75, y: 4.88, w: 1.2, h: 0.28, margin: 0, isTextBox: true,
    fontSize: 11, color: COL.cyan, align: "center" });
  s.addText("UART ×2", { x: M + 3 * (bw + gap) - 0.75, y: 4.88, w: 1.2, h: 0.28, margin: 0, isTextBox: true,
    fontSize: 11, color: COL.cyan, align: "center" });
  panel(s, M, 5.35, W - 2 * M, 1.3, { border: COL.red, fill: "1A1116", name: "kill switch" });
  label(s, "KILL_SWITCH", M + 0.35, 5.55, 3, COL.red, 12);
  s.addText("비상정지 버튼 → DC 컨택터가 바퀴 쪽 전원만 차단", { x: M + 0.35, y: 5.85, w: 7.5, h: 0.45, margin: 0,
    isTextBox: true, fontSize: 18, bold: true, color: COL.text });
  s.addText("ESP32 데드맨 · 워치독: 명령 끊김 시 바퀴 정지", { x: 8.15, y: 5.85, w: 4.45, h: 0.45, margin: 0,
    isTextBox: true, fontSize: 13, color: COL.muted, valign: "middle", align: "right" });
  s.addNotes("- 센서 인지 → Jetson 판단 → ESP32가 네 바퀴 개별 구동\n- 비상정지 시 컴퓨터는 유지, 바퀴 전원만 차단\n- 명령 끊김 시 ESP32가 자체 정지");
}

// ---------- 5. 차량 제원 ----------
{
  const s = slide("SPEC_SHEET", "누비 차량 제원", "시스템과 차체");
  picture(s, "v26_module_rails_iso.jpg", M, 2.05, 3.6, 2.32, "누비 3D 설계 모습");
  chip(s, "3D 설계", M + 0.12, 3.93);
  picture(s, "photo_vehicle_xtion.jpg", M, 4.5, 3.6, 2.15, "지금 시제품 — 프레임 위 보드, 배터리, 깊이 카메라, 화면");
  chip(s, "현재 시제품", M + 0.12, 6.21);
  const spec = [
    ["SIZE", "크기 (L × W × H)", "830 × 590 × 534 mm"],
    ["WEIGHT", "무게", "약 34 kg"],
    ["DRIVE", "구동", "6.5\" 허브모터 ×4 · 스키드 스티어"],
    ["SPEED", "최고 속도", "1.5 m/s (제한)"],
    ["BATTERY", "배터리", "36 V 10S 20 Ah"],
    ["COMPUTE", "온보드 컴퓨터", "NVIDIA Jetson Orin Nano"],
    ["CONTROL", "바퀴 제어", "ESP32 + 모터 보드 ×4"],
    ["SENSOR", "센서", "깊이 카메라 ×2 · 2D 라이다 · IMU"],
    ["FRAME", "차체", "알루미늄 프로파일 · 모듈 레일 ×3"],
    ["SAFETY", "안전", "비상정지 · 컨택터 · 워치독"],
    ["SOFTWARE", "소프트웨어", "Python (Jetson) · C++ (ESP32)"],
    ["COST", "부품비", "약 264만 원"],
  ];
  const gx = 4.55, cw = (W - M - gx - 0.25) / 2, rh = 0.76;
  spec.forEach(([tag, k, v], i) => {
    const x = gx + (i % 2) * (cw + 0.25), y = 2.05 + Math.floor(i / 2) * rh;
    s.addText(k, { x, y: y + 0.04, w: cw, h: 0.28, margin: 0, isTextBox: true, fontSize: 12, color: COL.cyan, bold: true });
    s.addText(v, { x, y: y + 0.3, w: cw, h: 0.36, margin: 0, isTextBox: true, fontSize: 15, bold: true, color: COL.text });
    s.addShape(pres.shapes.LINE, { x, y: y + rh - 0.04, w: cw, h: 0, line: { color: COL.line, width: 1 } });
  });
  s.addNotes("- 크기 · 무게는 3D 설계 기준 (무게는 시뮬레이터 추정)\n- 아래 사진: 현재 시험 중인 시제품");
}

// ---------- 6. 움직이는 방식 ----------
{
  const s = slide("DRIVE_MODE", "4륜 스키드 스티어 · 제자리 회전", "시스템과 차체");
  const modes = [
    ["직진", "왼쪽 = 오른쪽", 1, 1, "up"],
    ["커브", "왼쪽 < 오른쪽", 0.35, 1, "curve"],
    ["탱크턴 (제자리 회전)", "왼쪽 = − 오른쪽", -1, 1, "spin"],
  ];
  modes.forEach(([name, rule, L, R, kind], i) => {
    const px = M + i * 4.15, pw = 3.85;
    panel(s, px, 2.05, pw, 4.1, { border: kind === "spin" ? COL.yellow : COL.line });
    const cx = px + pw / 2, cy = 3.45, bw = 1.2, bh = 1.8;
    // 차체 (위에서 본 모습, 위쪽이 앞)
    s.addShape(pres.shapes.RECTANGLE, { x: cx - bw / 2, y: cy - bh / 2, w: bw, h: bh, fill: { color: "2A3140" },
      line: { color: COL.muted, width: 1 }, objectName: "body" });
    s.addShape(pres.shapes.ISOSCELES_TRIANGLE, { x: cx - 0.15, y: cy - bh / 2 + 0.1, w: 0.3, h: 0.22, fill: { color: COL.muted },
      line: { color: COL.muted }, objectName: "front mark" });
    const ww = 0.26, wh = 0.55;
    [[-1, -1], [1, -1], [-1, 1], [1, 1]].forEach(([sx, sy]) => {
      const wx = sx < 0 ? cx - bw / 2 - ww - 0.04 : cx + bw / 2 + 0.04;
      const wy = sy < 0 ? cy - bh / 2 + 0.08 : cy + bh / 2 - wh - 0.08;
      s.addShape(pres.shapes.RECTANGLE, { x: wx, y: wy, w: ww, h: wh, fill: { color: COL.text }, line: { color: COL.text }, objectName: "wheel" });
      const v = sx < 0 ? L : R, len = 0.15 + 0.5 * Math.abs(v);
      const ax = sx < 0 ? wx - 0.22 : wx + ww + 0.22, mid = wy + wh / 2;
      const c = v >= 0 ? COL.cyan : COL.red;
      const y1 = v >= 0 ? mid + len / 2 : mid - len / 2, y2 = v >= 0 ? mid - len / 2 : mid + len / 2;
      s.addShape(pres.shapes.LINE, { x: ax, y: Math.min(y1, y2), w: 0, h: Math.abs(y2 - y1), flipV: v >= 0,
        line: { color: c, width: 3, endArrowType: "triangle" }, objectName: "wheel speed" });
    });
    // 차 전체가 움직이는 방향
    if (kind === "up") arrow(s, cx, cy + 0.45, cx, cy - 0.45, COL.yellow);
    if (kind === "curve") arrow(s, cx + 0.25, cy + 0.45, cx - 0.35, cy - 0.45, COL.yellow);
    if (kind === "spin") s.addShape(pres.shapes.CIRCULAR_ARROW, { x: cx - 0.45, y: cy - 0.45, w: 0.9, h: 0.9,
      fill: { color: COL.yellow }, line: { color: COL.yellow }, flipH: true, objectName: "spin" });
    s.addText(name, { x: px + 0.3, y: 4.95, w: pw - 0.6, h: 0.45, margin: 0, isTextBox: true, fontSize: 20, bold: true,
      color: kind === "spin" ? COL.yellow : COL.text });
    s.addText(rule, { x: px + 0.3, y: 5.45, w: pw - 0.6, h: 0.35, margin: 0, isTextBox: true, fontSize: 14, color: COL.cyan });
  });
  s.addText([{ text: "STEER_BY_SPEED  ", options: { bold: true, color: COL.yellow } },
    { text: "조향 장치 없이 좌우 바퀴 속도 차로 방향 전환 · ESP32가 네 바퀴를 각각 제어", options: { color: COL.text } }],
    { x: M, y: 6.3, w: W - 2 * M, h: 0.4, margin: 0, isTextBox: true, fontSize: 14, valign: "middle" });
  s.addNotes("- 별도 조향 장치 없음\n- 좌우 바퀴 속도 차로 방향 전환\n- 좌우 반대 회전 시 제자리 회전 (탱크턴) → 좁은 복도 U턴 가능");
}

// ---------- 6. 호버보드 개조 ----------
{
  const s = slide("HARDWARE_MOD", "호버보드를 자율주행 차체로", "시스템과 차체");
  picture(s, "photo_teardown.jpg", M, 2.05, 2.85, 4.6, "호버보드를 뜯어 메인보드 두 장과 허브모터가 보이는 모습");
  picture(s, "photo_reflash.jpg", M + 3.05, 2.05, 2.85, 4.6, "떼어 낸 호버보드 메인보드에 USB 장치를 연결해 펌웨어를 다시 올리는 모습");
  chip(s, "호버보드 분해", M + 0.12, 6.19);
  chip(s, "펌웨어 재플래시", M + 3.17, 6.19);
  const rx = 6.85, rw = W - M - rx;
  const mods = [
    ["MOD_01", "모터 · 배터리 재활용", "호버보드 허브모터 4개로 구동부 비용 절감"],
    ["MOD_02", "모터 보드 펌웨어 교체", "오픈소스 펌웨어로 재플래시, 시리얼로 제어"],
    ["MOD_03", "바퀴 제어기 개발", "ESP32 펌웨어 · UART 통신 · 안전장치"],
    ["MOD_04", "무선 조종 · 상태 화면", "Wi-Fi 조종 페이지 · 게임패드 · 웹 상태 화면"],
  ];
  mods.forEach(([tag, t, d], i) => {
    const y = 2.05 + i * 1.17;
    panel(s, rx, y, rw, 1.03);
    label(s, tag, rx + 0.25, y + 0.15, 1.2);
    s.addText(t, { x: rx + 1.45, y: y + 0.1, w: rw - 1.6, h: 0.4, margin: 0, isTextBox: true, fontSize: 17, bold: true, color: COL.text });
    s.addText(d, { x: rx + 1.45, y: y + 0.52, w: rw - 1.6, h: 0.4, margin: 0, isTextBox: true, fontSize: 13, color: COL.muted });
  });
  s.addNotes("- 시판 호버보드 모터 · 모터 보드 재활용\n- 모터 보드에 오픈소스 펌웨어 재플래시\n- ESP32 바퀴 제어 펌웨어 · 무선 조종 화면 직접 개발");
}

// ---------- 7. 제작 일정 ----------
{
  const s = slide("BUILD_LOG", "제작 과정", "시스템과 차체");
  const steps = [
    ["07 ~ 08월", "tl_teardown.jpg", "호버보드 분해\n구동부 확보", COL.cyan],
    ["08월", "tl_frame.jpg", "프레임 · 휠 마운트\n제작", COL.cyan],
    ["09.11", "tl_slam.jpg", "RC카 SLAM\n자율주행 검증", COL.cyan],
    ["09.17", "tl_guard.jpg", "깊이 카메라\n장애물 정지", COL.cyan],
    ["10.13", "tl_wheels.jpg", "ESP32\n네 바퀴 구동 시험", COL.cyan],
    ["10.14", "tl_indoor.jpg", "실차 주행 · 녹화\n위치 찾기", COL.yellow],
  ];
  const cw = 1.88, cg = (W - 2 * M - 6 * cw) / 5;
  s.addShape(pres.shapes.LINE, { x: M, y: 2.62, w: W - 2 * M, h: 0, line: { color: COL.cyan, width: 2, endArrowType: "triangle" } });
  steps.forEach(([date, img, t, c], i) => {
    const x = M + i * (cw + cg);
    s.addText(date, { x, y: 2.05, w: cw, h: 0.35, margin: 0, isTextBox: true, fontSize: 16, bold: true, color: c });
    s.addShape(pres.shapes.OVAL, { x: x + 0.02, y: 2.52, w: 0.2, h: 0.2, fill: { color: c }, line: { color: COL.bg, width: 2 } });
    picture(s, img, x, 2.95, cw, 1.45, t.replace("\n", " "));
    s.addText(t, { x, y: 4.55, w: cw, h: 0.75, margin: 0, isTextBox: true, fontSize: 14, bold: true, color: COL.text, valign: "top" });
  });
  panel(s, M, 5.55, W - 2 * M, 1.1, { border: COL.yellow, name: "support" });
  label(s, "SUPPORTED BY", M + 0.35, 5.75, 3, COL.yellow, 12);
  s.addText("2026 차세대 공학인재 양성사업 지원으로 제작", { x: M + 0.35, y: 6.05, w: W - 2 * M - 0.7, h: 0.45,
    margin: 0, isTextBox: true, fontSize: 17, bold: true, color: COL.text });
  s.addNotes("- 7월 호버보드 분해로 시작\n- 9월 소형 RC카로 SLAM · 자율주행 선검증\n- 10월부터 실제 차량 카메라 주행\n- 2026 차세대 공학인재 양성사업 지원으로 제작");
}

// ---------- 8. 오픈 플랫폼 ----------
{
  const s = slide("OPEN_PLATFORM", "누구나 재현 가능한 오픈 플랫폼", "시스템과 차체");
  picture(s, "v28_inside_iso.jpg", M, 2.05, 5.6, 3.6, "누비 차체 속 — 배터리, 보드, 배선까지 설계 파일에 들어 있음");
  const items = [
    ["OPEN_01", "시판 부품", "호버보드 허브모터 · 알루미늄 프로파일 · 국내 부품표"],
    ["OPEN_02", "열린 코드", "바퀴 제어 펌웨어 · 주행 코드 · 시뮬레이터, MIT 라이선스"],
    ["OPEN_03", "코드로 설계한 차체", "Python 설계 파일로 3D 모델 · STEP 생성"],
    ["OPEN_04", "모든 기록", "첫 설계부터 작업 일지 · 설계 결정 · 실패한 시도까지"],
  ];
  const rx = 6.6, rw = W - M - rx;
  items.forEach(([tag, t, d], i) => {
    const y = 2.05 + i * 0.93;
    panel(s, rx, y, rw, 0.8);
    label(s, tag, rx + 0.25, y + 0.12, 1.2);
    s.addText(t, { x: rx + 1.45, y: y + 0.06, w: rw - 1.6, h: 0.36, margin: 0, isTextBox: true, fontSize: 16, bold: true, color: COL.text });
    s.addText(d, { x: rx + 1.45, y: y + 0.42, w: rw - 1.6, h: 0.32, margin: 0, isTextBox: true, fontSize: 12, color: COL.muted });
  });
  s.addShape(pres.shapes.PARALLELOGRAM, { x: M, y: 5.95, w: W - 2 * M, h: 0.6, fill: { color: COL.yellow }, line: { color: COL.yellow }, objectName: "open bar" });
  s.addText("비싼 전용 로봇 대신, 학생 팀이 시판 부품으로 다시 만들 수 있는 자율주행 플랫폼", { x: M + 0.35, y: 5.95, w: W - 2 * M - 0.7, h: 0.6,
    margin: 0, isTextBox: true, fontSize: 17, bold: true, italic: true, color: COL.ink, valign: "middle" });
  s.addNotes("- 로봇 한 대가 아닌, 다른 팀도 재현 가능한 플랫폼\n- 부품은 호버보드 + 국내 구매 가능 부품만 사용\n- 설계 파일 · 펌웨어 · 주행 코드 · 작업 일지 공개 예정 (MIT)");
}

// ---------- 9. 위치추정 ----------
pres.addSection({ title: "자율주행" });
{
  const s = slide("LOCALIZATION", "세 센서 융합 위치추정", "자율주행");
  picture(s, "robot_map_paths.jpg", M, 2.05, 4.75, 4.5, "3D 복도 지도 위에 웹캠 사진으로 찍은 실제 주행 경로 다섯 번");
  const rx = 5.75, rw = W - M - rx;
  const rows = [["카메라", "지도 사진과 맞춰 내 위치 다시 찾기"], ["IMU", "회전 방향 보정"], ["바퀴 엔코더", "실제 이동 거리 측정"]];
  rows.forEach(([t, d], i) => {
    const y = 2.05 + i * 0.95;
    s.addText(t, { x: rx, y, w: rw - 1.5, h: 0.42, margin: 0, isTextBox: true, fontSize: 20, bold: true, color: COL.text });
    s.addText(d, { x: rx, y: y + 0.42, w: rw - 1.5, h: 0.35, margin: 0, isTextBox: true, fontSize: 14, color: COL.muted });
    s.addText("ONLINE", { x: W - M - 1.4, y: y + 0.2, w: 1.4, h: 0.35, margin: 0, isTextBox: true, fontSize: 13, bold: true,
      color: COL.cyan, align: "right", charSpacing: 2 });
    s.addShape(pres.shapes.LINE, { x: rx, y: y + 0.88, w: rw, h: 0, line: { color: COL.line, width: 1 } });
  });
  s.addShape(pres.shapes.PARALLELOGRAM, { x: rx, y: 5.0, w: rw, h: 0.55, fill: { color: COL.yellow }, line: { color: COL.yellow }, objectName: "fusion bar" });
  s.addText("FUSION → 오차 누적 억제", { x: rx + 0.3, y: 5.0, w: rw - 0.6, h: 0.55, margin: 0, isTextBox: true,
    fontSize: 17, bold: true, italic: true, color: COL.ink, valign: "middle" });
  s.addNotes("- 바퀴 · IMU만으로는 오차 누적\n- 카메라 사진을 3D 복도 지도에 맞춰 위치 재보정\n- 왼쪽: 실제 주행 5회 경로를 지도 위에 복원");
}

// ---------- 11. 실차 ↔ 복도 시뮬레이터 ----------
{
  const s = slide("REAL_TO_SIM", "실제 주행을 복도 시뮬레이터에 그대로", "자율주행");
  const iw = 2.9, ih = 1.63, g1 = 0.1, g2 = 0.33;
  const xs = [M, M + iw + g1, M + 2 * iw + g1 + g2, M + 3 * iw + 2 * g1 + g2];
  [["복도 · 10.14", "t200", 0], ["로비 · 10.14", "t275", 2]].forEach(([place, t, k]) => {
    label(s, place, xs[k], 2.05, 3, COL.yellow, 12);
    picture(s, `real_${t}.jpg`, xs[k], 2.4, iw, ih, `실제 웹캠 화면 — ${place}`);
    picture(s, `sim_${t}.jpg`, xs[k + 1], 2.4, iw, ih, `같은 위치의 복도 시뮬레이터 화면 — ${place}`);
    chip(s, "실제 웹캠", xs[k] + 0.08, 2.4 + ih - 0.42);
    chip(s, "시뮬레이터", xs[k + 1] + 0.08, 2.4 + ih - 0.42);
  });
  picture(s, "replay_map.jpg", M, 4.35, 9.05, 2.29, "복도 지도 위 실제 주행 경로 — 파랑은 카메라와 바퀴를 합친 위치, 주황 점은 카메라로 찾은 위치");
  const tx = M + 9.35;
  label(s, "PIPELINE", tx, 4.4, 3, COL.cyan, 12);
  s.addText([
    { text: "① 웹캠으로 실제 주행 녹화", options: { breakLine: true } },
    { text: "② 3D 복도 지도에서 위치 찾기", options: { breakLine: true } },
    { text: "③ 같은 위치를 시뮬레이터로", options: {} },
  ], { x: tx, y: 4.8, w: W - M - tx, h: 1.8, margin: 0, isTextBox: true, fontSize: 14, bold: true, color: COL.text,
    valign: "top", paraSpaceAfter: 10 });
  s.addNotes("- 차에 웹캠 장착 후 실제 복도 주행 녹화\n- 사진을 3D 복도 지도에 맞춰 위치 추정\n- 같은 위치의 시뮬레이터 화면을 실제 화면과 비교\n- 아래: 지도 위 실제 주행 경로");
}

// ---------- 10. 판단 ----------
{
  const s = slide("DECISION_CORE", "학습은 시뮬레이터에서, 판단은 차 안에서", "자율주행");
  const cards = [
    ["TRAIN // OFFLINE", "시뮬레이터에서 학습", "교내 복도를 옮긴 3D 시뮬레이터,\n장애물 회피 강화학습", COL.line, COL.cyan],
    ["INFER // REALTIME", "차 안에서 판단", "작은 신경망(7,170 파라미터)만\n올려 실시간 판단", COL.line, COL.cyan],
    ["FAILSAFE", "위험 시 정지", "규칙 기반 안전 필터가 마지막 확인\n정면 장애물 감속 · 차가 오면 양보", COL.red, COL.red],
  ];
  const cw = 3.75, cg = 0.4;
  cards.forEach(([tag, t, d, border, tc], i) => {
    const x = M + i * (cw + cg);
    panel(s, x, 2.05, cw, 1.85, { border, fill: i === 2 ? "1A1116" : COL.panel });
    label(s, tag, x + 0.3, 2.22, cw - 0.5, tc);
    s.addText(t, { x: x + 0.3, y: 2.55, w: cw - 0.5, h: 0.5, margin: 0, isTextBox: true, fontSize: 21, bold: true, color: COL.text });
    s.addText(d, { x: x + 0.3, y: 3.08, w: cw - 0.5, h: 0.75, margin: 0, isTextBox: true, fontSize: 14, color: COL.muted, valign: "top" });
    if (i < 2) arrow(s, x + cw + 0.06, 2.97, x + cw + cg - 0.06, 2.97);
  });
  picture(s, "sim_hallway.jpg", M, 4.15, 3.85, 2.48, "교내 복도를 3D로 옮긴 웹 시뮬레이터에서 누비 차체가 자율주행하는 모습");
  chip(s, "교내 복도 시뮬레이터", M + 0.12, 6.17);
  picture(s, "photo_obstacle_guard.jpg", M + 4.15, 4.15, 3.31, 2.48, "깊이 카메라 장애물 가드 화면 — 472 mm 앞 사람을 보고 STOP");
  chip(s, "장애물 가드 STOP", M + 4.27, 6.17);
  const tx = M + 7.75;
  label(s, "FIELD_TEST", tx, 4.2, 4, COL.yellow);
  s.addText([
    { text: "09-14", options: { bold: true, color: COL.cyan, breakLine: true } },
    { text: "시뮬레이터에서 배운 정책으로\n실물 로봇 장애물 회피 실주행", options: { color: COL.text, breakLine: true } },
    { text: "09-17", options: { bold: true, color: COL.red, breakLine: true } },
    { text: "깊이 카메라가 47 cm 앞 사람을 보고 정지", options: { color: COL.text } },
  ], { x: tx, y: 4.55, w: W - M - tx, h: 2.1, margin: 0, isTextBox: true, fontSize: 14, valign: "top", paraSpaceAfter: 4 });
  s.addNotes("- 학습은 시뮬레이터, 차에는 경량 신경망만 탑재\n- 신경망 판단 후 규칙 기반 안전 필터가 최종 확인 (정지 · 양보)\n- 9월: 시뮬레이터 학습 정책으로 소형 실물 로봇 실주행");
}

// ---------- 11. 성과 ----------
pres.addSection({ title: "성과와 다음" });
{
  const s = slide("TELEMETRY", "실차 주행 결과와 연구 성과", "성과와 다음");
  const stats = [
    ["POS_ERROR", "1.1 cm", "지도 기반 위치 재인식 오차", COL.cyan],
    ["FIELD_LOG", "5회", "실차 주행 녹화", COL.yellow],
    ["BUILD_COST", "264만 원", "부품비 (목표 250만 원)", COL.cyan],
  ];
  stats.forEach(([tag, v, l, c], i) => {
    const x = M + i * 4.15;
    panel(s, x, 2.05, 3.85, 2.7);
    label(s, tag, x + 0.3, 2.3, 3.3);
    s.addText(v, { x: x + 0.3, y: 2.8, w: 3.3, h: 1.0, margin: 0, isTextBox: true, fontSize: 48, bold: true, color: c, valign: "middle" });
    s.addText(l, { x: x + 0.3, y: 3.95, w: 3.3, h: 0.4, margin: 0, isTextBox: true, fontSize: 15, bold: true, color: COL.text });
  });
  panel(s, M, 5.1, W - 2 * M, 1.5, { border: COL.yellow, name: "paper" });
  label(s, "PAPER // 2026 한국사물인터넷학회 종합학술대회", M + 0.35, 5.33, 9, COL.yellow, 12);
  s.addText("Jetson Orin Nano 기반 폴리터널 시각 주행거리계:\n격자 균등 특징점 추출기 선정과 CPU · 하이브리드 배치", {
    x: M + 0.35, y: 5.7, w: W - 2 * M - 0.7, h: 0.85, margin: 0, isTextBox: true, fontSize: 16, bold: true, color: COL.text, valign: "top" });
  s.addNotes("- 1.1 cm: 복도 3D 지도에서 사진 한 장으로 위치 재인식한 오차\n- 실차 주행 녹화 5회\n- 부품비 약 264만 원");
}

// ---------- 12. 다음 ----------
{
  const s = slide("NEXT_DEPLOY", "하나의 플랫폼, 여러 현장으로", "성과와 다음");
  const deps = [["DEPLOY_A", "배달", "적재함 모듈로 캠퍼스 · 건물 안 운반"], ["DEPLOY_B", "순찰", "센서 모듈로 정해진 경로 반복 감시"],
    ["DEPLOY_C", "농업", "비닐하우스 고랑 사이 자율 이동"]];
  deps.forEach(([tag, t, d], i) => {
    const x = M + i * 4.15;
    panel(s, x, 2.05, 3.85, 1.8);
    label(s, tag, x + 0.3, 2.3, 3.2);
    s.addText(t, { x: x + 0.3, y: 2.65, w: 3.3, h: 0.5, margin: 0, isTextBox: true, fontSize: 24, bold: true, color: COL.text });
    s.addText(d, { x: x + 0.3, y: 3.2, w: 3.3, h: 0.45, margin: 0, isTextBox: true, fontSize: 13, color: COL.muted });
  });
  panel(s, M, 4.15, 7.95, 2.5, { border: COL.cyan, name: "upgrade" });
  label(s, "UPGRADE // LOADING...", M + 0.35, 4.4, 5);
  s.addText("3차원 공간인지", { x: M + 0.35, y: 4.75, w: 7.2, h: 0.55, margin: 0, isTextBox: true, fontSize: 24, bold: true, color: COL.text });
  s.addText("평면 지도를 넘어 입체 공간 이해로 · 설계와 코드 전체 공개", { x: M + 0.35, y: 5.3, w: 7.2, h: 0.4, margin: 0,
    isTextBox: true, fontSize: 14, color: COL.muted });
  for (let k = 0; k < 12; k++) {
    s.addShape(pres.shapes.RECTANGLE, { x: M + 0.35 + k * 0.6, y: 5.95, w: 0.48, h: 0.3,
      fill: { color: k < 7 ? COL.cyan : COL.line }, line: { color: k < 7 ? COL.cyan : COL.line }, objectName: "progress cell" });
  }
  photo(s, 8.95, 4.15, W - M - 8.95, 2.5, "QR", "SCAN // 주행 영상");
  s.addNotes("- 같은 차체 + 모듈 교체로 배달 · 순찰 · 농업 현장 확장 목표\n- 다음 단계: 평면 지도 → 3차원 공간 인지\n- QR: 주행 영상");
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  if (SKILL_THEME) {
    const { applyTheme } = require(SKILL_THEME);
    await applyTheme(OUT, THEME);
  }
  console.log("wrote", OUT);
})();
