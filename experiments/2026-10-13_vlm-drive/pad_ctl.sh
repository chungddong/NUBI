#!/usr/bin/env bash
# 게임패드 운전 (pad.py drive) 을 Jetson 에서 뒤로 띄우거나 끈다 — PC 에서 ssh 로 부르기 좋게.
#   bash pad_ctl.sh start [--fake] | stop | status        기록: <repo>/data/logs/pad/pad.log (재부팅해도 남는다)
#   bash pad_ctl.sh install | uninstall                  부팅하면 스스로 start (사용자 crontab @reboot — sudo 없이)
# start 는 pad.py 가 끝나면 3 s 뒤 다시 띄우는 고리로 돈다 — 부팅 직후 ESP32 가 아직 안 잡혀 끝나도 다시 시도한다.
# pad.py 는 놓은 상태로 뜨므로 (START 전에는 아무것도 안 보냄) 다시 떠도 바퀴는 움직이지 않는다.
# 패드 권한은 자동 로그인 (gdm, dongki) 뒤 uaccess 로 생긴다 — 그 전에는 pad.py 가 1 s 마다 다시 연다.
# 끄기는 고리를 먼저 끝내고 SIGTERM (pad.py 가 받아 바퀴를 세우고 w 0 으로 끝낸다). 뒤로 띄운 프로세스는 SIGINT 를 무시한다.
# 시리얼은 한 프로그램만 연다 (open_serial 이 잠금) — bench.py 가 돌고 있으면 먼저 bench_ctl.sh stop.
# 기록을 /tmp 에 두면 재부팅 때 지워진다 (2026-10-14 밖에서 몬 기록을 그렇게 잃었다). Jetson 은 시계 배터리가 없어
# 네트워크 없이 켜면 시각이 마지막 저장 시각부터 가므로, 고리를 시작할 때 시계 동기화 여부를 같이 적는다.
cd "$(dirname "$0")"
LOOP="pad_ctl.sh loop"
PAT="python3 -u pad.py"
LOG_DIR="$(cd ../.. && pwd)/data/logs/pad"
LOG="$LOG_DIR/pad.log"
mkdir -p "$LOG_DIR"
stop() {
  pkill -TERM -f "$LOOP"
  pkill -TERM -f "$PAT" || return 0
  for _ in 1 2 3 4 5 6; do pgrep -f "$PAT" >/dev/null || return 0; sleep 0.5; done
  echo "3 s 안에 안 꺼져 KILL" && pkill -KILL -f "$PAT"
}
case "${1:-status}" in
  loop)
    echo "=== $(date '+%F %T') 고리 시작 · 부팅 $(uptime -s) · 시계 동기화 $(timedatectl show -p NTPSynchronized --value 2>/dev/null) ==="
    while true; do
      python3 -u pad.py drive "${@:2}"
      echo "$(date +%T) pad.py 끝남 (코드 $?) — 3 s 뒤 다시"
      sleep 3
    done ;;
  stop)
    stop; pgrep -f "$PAT" >/dev/null && echo "아직 돈다" || echo "꺼짐" ;;
  start)
    stop
    [ -f "$LOG" ] && [ "$(stat -c %s "$LOG")" -gt 20000000 ] && mv -f "$LOG" "$LOG.1"     # 20 MB 넘으면 하나만 남기고 넘김
    nohup bash pad_ctl.sh loop "${@:2}" >> "$LOG" 2>&1 < /dev/null &
    sleep 4
    tail -5 "$LOG" ;;
  install)
    (crontab -l 2>/dev/null | grep -v "pad_ctl.sh"; echo "@reboot bash $PWD/pad_ctl.sh start") | crontab -
    crontab -l | grep pad_ctl ;;
  uninstall)
    crontab -l 2>/dev/null | grep -v "pad_ctl.sh" | crontab -
    echo "부팅 자동 실행 지움" ;;
  log)
    tail -n "${2:-40}" "$LOG" ;;
  *)
    pgrep -af "$PAT" || echo "안 돈다"
    tail -5 "$LOG" 2>/dev/null ;;
esac
