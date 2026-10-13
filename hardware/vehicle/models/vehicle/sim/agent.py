r"""GPT 운전 — 시뮬레이터 앞 카메라 사진 한 장으로 다음 행동을 고른다 (web/serve.py 의 POST /api/agent/step).

브라우저 (web/agent.js) 가 보내는 것: 앞 카메라 사진 (깊이로 잰 갈 수 있는 방향에 번호 화살표를 그린 것), 화살표마다
빈 거리, 목표 문, 층 안내 (켜면), 지난 걸음 기록, GPT 가 남긴 메모. **위치 좌표는 보내지 않는다** — 차는 카메라만 있다.
돌려주는 것: 보이는 것 · 메모 · 행동 (move 화살표 + 거리 / turn_left · turn_right 각도 / stop).

방식은 VLMnav (Goetting 외, CoRL 2024 — 깊이로 갈 수 있는 방향을 화살표로 그려 VLM 이 고르게) 를 따랐다.

API 키: 환경변수 OPENAI_API_KEY, 없으면 이 폴더의 agent.local.json {"openai_api_key": "..."} (git 제외 *.local.*).
기록: agent_logs/YYYY-MM-DD.jsonl (사진 빼고, git 제외).
모델 'mock' 은 API 없이 연결만 시험한다 (목표를 모르고 가장 트인 쪽으로 간다).
"""

import datetime as _dt
import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
KEY_FILE = HERE / "agent.local.json"
LOG_DIR = HERE / "agent_logs"
URL = "https://api.openai.com/v1/responses"

# 100 만 토큰당 달러 (입력, 출력) — developers.openai.com/api/docs/models, 2026-10-13 확인
PRICE = {"gpt-6-astra": (10.0, 50.0), "gpt-6.1-sol": (2.0, 10.0), "gpt-6-luna": (0.10, 0.50)}

SCHEMA = {
    "type": "object",
    "properties": {
        "observation": {"type": "string", "description": "사진에 보이는 것 — 문, 표지판 글자, 복도 모양 (한국어, 짧게)"},
        "memory": {"type": "string", "description": "다음 걸음에 돌려받을 메모 — 지나온 문, 지금 추정 위치 (한국어, 200 자 안)"},
        "reason": {"type": "string", "description": "이 행동을 고른 까닭 (한국어, 한 문장)"},
        "action": {"type": "string", "enum": ["move", "turn_left", "turn_right", "stop"]},
        "arrow": {"type": "integer", "description": "move 일 때 화살표 번호, 아니면 0"},
        "distance_m": {"type": "number", "description": "move 거리 (m), 아니면 0"},
        "turn_deg": {"type": "number", "description": "turn 각도 (도), 아니면 0"},
    },
    "required": ["observation", "memory", "reason", "action", "arrow", "distance_m", "turn_deg"],
    "additionalProperties": False,
}

DEFAULT_CAMERA = "Orbbec Gemini 336, 렌즈 높이 17 cm, 수평 시야 90°, 세로 시야 65°, 깊이 0.1 ~ 6 m"

SYSTEM = """너는 실내 복도를 다니는 작은 바퀴 로봇을 운전한다.
센서는 앞 깊이 카메라 하나뿐이다 (사양은 글의 '카메라' 줄). 진짜 위치 좌표는 모른다 — 지도와 함께 추정 위치가
주어질 수는 있다.

매 걸음 앞 카메라 사진 한 장을 받는다. 사진 위 번호 화살표는 바닥 위로 갈 수 있는 방향이고, 깊이 카메라로 잰
'빈 거리'가 함께 온다 (초록 = 1 m 넘게 트임, 주황 = 0.5–1 m, 빨강 = 막힘). 화살표 끝은 빈 거리 (최대 3 m) 지점이다.
'사각' 이 붙은 화살표는 카메라가 너무 가까운 것을 재지 못해 막힘으로 친 방향이다 — 그쪽으로 가지 말고 돌거나 물러나라.

행동은 하나만 고른다.
- move: 화살표 번호와 거리. 그 방향으로 몸을 돌린 뒤 곧게 간다. 거리는 빈 거리보다 0.3 m 짧게, 한 번에 2 m 까지.
- turn_left / turn_right: 제자리에서 그 각도만큼 돈다 (15–180°). 둘러볼 때도 쓴다.
- stop: 목표 문 앞 복도 (문에서 1 m 안) 에 왔다고 판단할 때만. 멈추면 끝이다.

카메라가 바닥 가까이 있어서 높은 곳은 가까이에서 시야 밖이다. 문 위쪽 번호판 (높이 약 1.8 m) 은 문 바로 앞에서는
안 보이고, 표지판 글자도 거의 읽히지 않는다.

지도 (두 번째 사진과 좌표) 와 추정 위치가 주어지면, 갈 길과 멈출 곳은 지도와 추정 위치로 정하라. 화살표마다 '지도 방향'
이 붙어 있으니 목표 쪽 방향과 맞는 화살표를 고른다. 추정 위치는 오차가 있으니 사진 (문 · 사물함 · 벽 모양, 문 개수) 과
맞지 않으면 사진 쪽을 믿고 조금씩 움직여 확인하라. 지도나 위치가 없으면 문 개수 · 생김새와 층 안내로 지금 어디인지
추론하라. 확실하지 않으면 돌아서 둘러보거나 조금씩 움직여라.

memory 칸에 지나온 문과 지금 추정 위치를 짧게 적어라 — 다음 걸음에 그대로 돌려준다.
observation · memory · reason 은 한국어로 짧게 쓴다."""


def api_key():
    k = os.environ.get("OPENAI_API_KEY")
    if k:
        return k
    if KEY_FILE.exists():
        return json.loads(KEY_FILE.read_text(encoding="utf-8")).get("openai_api_key")
    return None


def user_text(req):
    lines = [f"목표: {req['goal']} 문 앞 복도에 멈추기.", f"카메라: {req.get('camera') or DEFAULT_CAMERA}", ""]
    if req.get("directory"):
        lines += ["층 안내 (방 순서 — 좌표 없음):", req["directory"], ""]
    if req.get("map_text"):
        lines += ["지도:", req["map_text"], ""]
    lines.append(f"걸음 {req['step']} / 최대 {req['max_steps']}.")
    lines.append("화살표 (번호: 방향, 빈 거리" + (", 지도 방향" if any("map_deg" in c for c in req["candidates"]) else "") + "):")
    for c in req["candidates"]:
        side = "정면" if abs(c["deg"]) < 1 else f"{'왼쪽' if c['deg'] > 0 else '오른쪽'} {abs(c['deg']):.0f}°"
        lines.append(f"  {c['id']}: {side}, {c['free']:.1f} m" + (" (사각)" if c.get("blind") else "")
                     + (f", 지도 {c['map_deg']}°" if "map_deg" in c else ""))
    if req.get("history"):
        lines += ["", "지난 걸음 (오래된 것부터):"] + [f"  {h}" for h in req["history"]]
    lines += ["", f"지난 메모: {req.get('memory') or '(없음)'}"]
    return "\n".join(lines)


def call_openai(req):
    key = api_key()
    if not key:
        return {"ok": False, "error": "OpenAI API 키가 없다 — 환경변수 OPENAI_API_KEY 또는 sim/agent.local.json"}
    body = {
        "model": req["model"],
        "input": [
            {"role": "system", "content": SYSTEM},
            {"role": "user", "content": [
                {"type": "input_text", "text": user_text(req)},
                {"type": "input_image", "image_url": req["image"], "detail": "high"},
            ] + ([{"type": "input_image", "image_url": req["map_image"], "detail": "high"}] if req.get("map_image") else [])},
        ],
        "text": {"format": {"type": "json_schema", "name": "drive_action", "schema": SCHEMA, "strict": True}},
        "reasoning": {"effort": req.get("effort", "low")},
    }
    r = urllib.request.Request(URL, data=json.dumps(body).encode("utf-8"), method="POST",
                               headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(r, timeout=180) as f:
            res = json.loads(f.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return {"ok": False, "error": f"HTTP {e.code}: {e.read().decode('utf-8', 'replace')[:600]}"}
    except (urllib.error.URLError, TimeoutError) as e:
        return {"ok": False, "error": f"연결 실패: {e}"}
    text = next((c.get("text") for it in res.get("output", []) if it.get("type") == "message"
                 for c in it.get("content", []) if c.get("type") == "output_text"), None)
    if text is None:
        return {"ok": False, "error": f"응답에 output_text 가 없다 (status {res.get('status')})"}
    u = res.get("usage") or {}
    pin, pout = PRICE.get(req["model"], (0.0, 0.0))
    usage = {"input": u.get("input_tokens", 0), "output": u.get("output_tokens", 0),
             "reasoning": (u.get("output_tokens_details") or {}).get("reasoning_tokens", 0)}
    usage["usd"] = (usage["input"] * pin + usage["output"] * pout) / 1e6
    return {"ok": True, "decision": json.loads(text), "usage": usage}


def mock(req):
    """API 없이 — 가운데 셋 중 가장 트인 화살표로 1.5 m, 다 막히면 왼쪽으로 90°. 목표를 모르니 멈추지 않는다."""
    cs = sorted(req["candidates"], key=lambda c: abs(c["deg"]))[:3]
    best = max(cs, key=lambda c: c["free"])
    if best["free"] < 0.8:
        d = {"action": "turn_left", "arrow": 0, "distance_m": 0, "turn_deg": 90, "reason": "앞이 막혀 돈다"}
    else:
        d = {"action": "move", "arrow": best["id"], "distance_m": min(1.5, best["free"] - 0.3), "turn_deg": 0,
             "reason": "가장 트인 쪽"}
    return {"ok": True, "decision": {"observation": "(모의 — 사진을 보지 않음)", "memory": "", **d},
            "usage": {"input": 0, "output": 0, "reasoning": 0, "usd": 0.0}}


def step(req):
    t0 = time.time()
    res = mock(req) if req.get("model") == "mock" else call_openai(req)
    res["latency_ms"] = round((time.time() - t0) * 1000)
    log_step(req, res)
    return res


def log_step(req, res):
    """한 걸음을 agent_logs/날짜.jsonl 에 (사진 빼고). agent_claude.py 도 쓴다."""
    LOG_DIR.mkdir(exist_ok=True)
    rec = {"t": _dt.datetime.now().isoformat(timespec="seconds"), "run": req.get("run"), "step": req.get("step"),
           "model": req.get("model"), "effort": req.get("effort"), "goal": req.get("goal"),
           "candidates": req.get("candidates"), "history": req.get("history"), "memory": req.get("memory"),
           "truth": req.get("truth"), **{k: v for k, v in res.items() if k != "image"}}
    with open(LOG_DIR / f"{_dt.date.today().isoformat()}.jsonl", "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
