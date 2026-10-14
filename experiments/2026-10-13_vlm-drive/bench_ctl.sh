#!/usr/bin/env bash
# 바퀴 점검판 (bench.py) 을 Jetson 에서 뒤로 띄우거나 끈다 — PC 에서 ssh 로 부르기 좋게.
#   bash bench_ctl.sh start | stop | status        기록: /tmp/bench.log
# 끄기는 SIGTERM (bench.py 가 받아 바퀴를 세우고 w 0 으로 끝낸다). 뒤로 띄운 프로세스는 SIGINT 를 무시하므로 쓰지 않는다.
cd "$(dirname "$0")"
stop() {
  pkill -TERM -f "python3 -u bench.py" || return 0
  for _ in 1 2 3 4 5 6; do pgrep -f "python3 -u bench.py" >/dev/null || return 0; sleep 0.5; done
  echo "3 s 안에 안 꺼져 KILL" && pkill -KILL -f "python3 -u bench.py"
}
case "${1:-status}" in
  stop)
    stop; pgrep -f "python3 -u bench.py" >/dev/null && echo "아직 돈다" || echo "꺼짐" ;;
  start)
    stop
    nohup python3 -u bench.py "${@:2}" > /tmp/bench.log 2>&1 < /dev/null &
    sleep 4
    cat /tmp/bench.log ;;
  *)
    pgrep -af "python3 -u bench.py" || echo "안 돈다"
    tail -3 /tmp/bench.log 2>/dev/null ;;
esac
