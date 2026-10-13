#!/usr/bin/env bash
# PC (Git Bash) → Jetson: 실차 운전에 필요한 것만 ~/NUBI 아래 같은 자리로 복사한다 (.venv · 기록 빼고).
#   bash experiments/2026-10-13_vlm-drive/to_jetson.sh [dongki@192.168.127.2]
# 유선 직결 주소는 docs/worklog/2026-10.md 'Jetson 유선 직결'. scene.json (복도 지도) 과 agent.local.json (API 키) 은
# git 밖 자료라 여기서 같이 보낸다 — 키를 Jetson 에 두고 싶지 않으면 그 파일을 빼고 Jetson 에서 환경변수로.
set -euo pipefail
HOST=${1:-dongki@192.168.127.2}
cd "$(dirname "$0")/../.."
SIM=hardware/vehicle/models/vehicle/sim
files=(experiments/2026-10-13_vlm-drive "$SIM/agent.py" "$SIM/agent_claude.py" "$SIM/web/hallway/data/scene.json")
[ -f "$SIM/agent.local.json" ] && files+=("$SIM/agent.local.json")
tar -cz --exclude=.venv --exclude=__pycache__ --exclude=_selftest_logs "${files[@]}" \
  | ssh "$HOST" "mkdir -p ~/NUBI && tar -xz -C ~/NUBI && echo '복사 끝: ~/NUBI' && ls ~/NUBI/experiments/2026-10-13_vlm-drive"
