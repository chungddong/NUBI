#!/usr/bin/env bash
# 복도 주행 녹화 (record.py) 를 Jetson 에서 뒤로 띄우거나 끈다 — PC 에서 ssh 로 부르고, 이더넷을 뽑고 몰아도 계속 돈다.
#   bash record_ctl.sh start [record.py 옵션] | stop | status        기록: /tmp/record.log
# start 는 웹 화면 (http://192.168.127.2:8093) 을 띄운다 — 카메라를 보고 녹화 버튼을 누른다. 화면 없이 바로 녹화는 --now.
# 바퀴 점검판 (bench.py) 이 시리얼 포트를 쥐고 있으면 먼저 끈다. 끄기는 SIGTERM (record.py 가 파일을 닫고 끝낸다).
cd "$(dirname "$0")"
PAT="python3 -u record.py"
stop() {
  pkill -TERM -f "$PAT" || return 0
  for _ in 1 2 3 4 5 6 7 8 9 10; do pgrep -f "$PAT" >/dev/null || return 0; sleep 0.5; done
  echo "5 s 안에 안 꺼져 KILL" && pkill -KILL -f "$PAT"
}
case "${1:-status}" in
  stop)
    stop; pgrep -f "$PAT" >/dev/null && echo "아직 돈다" || echo "꺼짐"; tail -2 /tmp/record.log 2>/dev/null ;;
  start)
    stop
    pgrep -f "python3 -u bench.py" >/dev/null && bash bench_ctl.sh stop
    nohup python3 -u record.py "${@:2}" > /tmp/record.log 2>&1 < /dev/null &
    sleep 8
    cat /tmp/record.log ;;
  *)
    pgrep -af "$PAT" || echo "안 돈다"
    tail -3 /tmp/record.log 2>/dev/null ;;
esac
