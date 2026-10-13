r"""Claude 운전 — agent.py (GPT 운전) 와 같은 프롬프트 · 출력 형식 · 기록으로 Claude 를 부른다 (Anthropic Python SDK).

web/serve.py 가 모델 이름이 claude- 로 시작하면 여기로 보낸다. 사진 · 화살표 · 실행 · 채점은 GPT 운전과 같아서
두 모델을 같은 조건으로 견줄 수 있다.

인증: SDK 기본 순서 (ANTHROPIC_API_KEY → ANTHROPIC_AUTH_TOKEN → `ant auth login` 프로필). 그게 없으면
agent.local.json 의 "anthropic_api_key" (저장소 공통 *.local.* 로 git 제외).
안전 분류기가 거절하면 (stop_reason "refusal") 서버 쪽 fallbacks "default" 로 추천 모델이 이어 받는다
(Haiku 5.5 는 서버 쪽 대체가 없어 뺀다).
"""

import json
import time

import anthropic

import agent

# 100 만 토큰당 달러 (입력, 출력) — claude-api 스킬 모델 표, 2026-10-06 기준
PRICE = {"claude-opus-5-5": (4.0, 20.0), "claude-sonnet-5-5": (2.0, 10.0), "claude-haiku-5-5": (0.10, 0.50)}
NO_FALLBACK = {"claude-haiku-5-5"}


def client():
    key = None
    if agent.KEY_FILE.exists():
        key = json.loads(agent.KEY_FILE.read_text(encoding="utf-8")).get("anthropic_api_key")
    return anthropic.Anthropic(api_key=key, timeout=180.0) if key else anthropic.Anthropic(timeout=180.0)


def image_block(data_url):
    head, data = data_url.split(",", 1)                         # data:image/jpeg;base64,....
    return {"type": "image", "source": {"type": "base64", "media_type": head.split(":", 1)[1].split(";", 1)[0], "data": data}}


def call_claude(req):
    images = [image_block(req["image"])] + ([image_block(req["map_image"])] if req.get("map_image") else [])
    params = dict(
        model=req["model"],
        max_tokens=16000,
        system=agent.SYSTEM,
        messages=[{"role": "user", "content": images + [{"type": "text", "text": agent.user_text(req)}]}],
        thinking={"type": "adaptive"},
        output_config={"effort": req.get("effort", "medium"),
                       "format": {"type": "json_schema", "schema": agent.SCHEMA}},
    )
    c = client()
    if not (c.api_key or c.auth_token or c.credentials):        # 없으면 SDK 가 요청 전에 TypeError 를 낸다
        return {"ok": False, "error": "Anthropic 키가 없다 — ANTHROPIC_API_KEY, `ant auth login`, 또는 sim/agent.local.json"}
    try:
        if req["model"] in NO_FALLBACK:
            r = c.messages.create(**params)
        else:
            r = c.beta.messages.create(**params, betas=["server-side-fallback-2026-07-01"], fallbacks="default")
    except anthropic.AuthenticationError:
        return {"ok": False, "error": "Anthropic 인증 실패 — 키를 확인 (ANTHROPIC_API_KEY 또는 sim/agent.local.json)"}
    except anthropic.RateLimitError as e:
        return {"ok": False, "error": f"요청 한도 초과 — {e.response.headers.get('retry-after', '?')} s 뒤 다시"}
    except anthropic.APIStatusError as e:
        return {"ok": False, "error": f"HTTP {e.status_code}: {e.message}"}
    except anthropic.APIConnectionError as e:
        return {"ok": False, "error": f"연결 실패: {e}"}
    if r.stop_reason == "refusal":
        sd = r.stop_details
        return {"ok": False, "error": f"거절 ({sd.category if sd else '?'}): {sd.explanation if sd else ''}"}
    if r.stop_reason == "max_tokens":
        return {"ok": False, "error": "max_tokens 에 걸려 답이 잘렸다"}
    text = next((b.text for b in r.content if b.type == "text"), None)
    if text is None:
        return {"ok": False, "error": f"응답에 text 블록이 없다 (stop_reason {r.stop_reason})"}
    pin, pout = PRICE.get(req["model"], (0.0, 0.0))
    usage = {"input": r.usage.input_tokens, "output": r.usage.output_tokens, "reasoning": 0}
    usage["usd"] = (usage["input"] * pin + usage["output"] * pout) / 1e6
    return {"ok": True, "decision": json.loads(text), "usage": usage, "served_by": r.model,
            "request_id": r._request_id}


def step(req):
    t0 = time.time()
    res = call_claude(req)
    res["latency_ms"] = round((time.time() - t0) * 1000)
    agent.log_step(req, res)
    return res
