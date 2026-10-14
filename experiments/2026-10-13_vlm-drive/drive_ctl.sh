#!/usr/bin/env bash
# 실차 운전 (drive.py) 을 Jetson 에서 뒤로 띄운다 — 게임패드로 승인하며 (이더넷을 뽑고 따라가도 계속 돈다).
#   bash drive_ctl.sh start [drive.py 옵션]   기본: --pose vis --confirm pad --model gpt-6-luna --goal 310앞
#   bash drive_ctl.sh stop | status | log      기록: /tmp/drive.log (걸음마다 사진 · 결정은 data/logs/vlm-drive/)
# 손 운전 (pad.py) 이 시리얼 포트를 쥐고 있으면 먼저 끈다. 끄기는 SIGTERM (drive.py 가 바퀴를 세우고 w 0 으로 끝낸다).
cd "$(dirname "$0")"
PAT="python3 -u drive.py"
stop() {
  pkill -TERM -f "$PAT" || return 0
  for _ in 1 2 3 4 5 6 7 8 9 10; do pgrep -f "$PAT" >/dev/null || return 0; sleep 0.5; done
  echo "5 s 안에 안 꺼져 KILL" && pkill -KILL -f "$PAT"
}
case "${1:-status}" in
  stop)
    stop; pgrep -f "$PAT" >/dev/null && echo "아직 돈다" || echo "꺼짐"; tail -3 /tmp/drive.log 2>/dev/null ;;
  start)
    stop
    pgrep -f "pad.py drive" >/dev/null && { [ -f pad_ctl.sh ] && bash pad_ctl.sh stop || pkill -TERM -f "pad.py drive"; sleep 1; }
    pgrep -f "python3 -u record.py" >/dev/null && bash record_ctl.sh stop
    args=("${@:2}")
    [ ${#args[@]} -eq 0 ] && args=(--pose vis --confirm pad --model gpt-6-luna --goal 310앞)
    nohup python3 -u drive.py "${args[@]}" > /tmp/drive.log 2>&1 < /dev/null &
    sleep 12
    cat /tmp/drive.log ;;
  log)
    tail -40 /tmp/drive.log ;;
  *)
    pgrep -af "$PAT" || echo "안 돈다"
    tail -5 /tmp/drive.log 2>/dev/null ;;
esac
