#!/usr/bin/env bash
# 웹캠 HDR 화면 (hdr_view.py) 을 Jetson 에서 뒤로 띄우거나 끈다.  bash hdr_ctl.sh start [옵션] | stop | status   기록: /tmp/hdr.log
# 녹화기 (record.py) 와 같은 웹캠을 쓰므로 둘 중 하나만 띄운다. 끄기는 SIGTERM (카메라 설정을 처음대로 되돌리고 끝낸다).
cd "$(dirname "$0")"
PAT="[p]ython3 -u hdr_view.py"
stop() {
  pkill -TERM -f "$PAT" || return 0
  for _ in 1 2 3 4 5 6; do pgrep -f "$PAT" >/dev/null || return 0; sleep 0.5; done
  pkill -KILL -f "$PAT"
}
case "${1:-status}" in
  stop) stop; tail -1 /tmp/hdr.log ;;
  start)
    stop
    pgrep -f "[p]ython3 -u record.py" >/dev/null && echo "녹화기 (record.py) 가 웹캠을 쓰고 있다 — record_ctl.sh stop 먼저" && exit 1
    nohup python3 -u hdr_view.py "${@:2}" > /tmp/hdr.log 2>&1 < /dev/null &
    sleep 6
    cat /tmp/hdr.log ;;
  *) pgrep -af "$PAT" || echo "안 돈다"; tail -2 /tmp/hdr.log ;;
esac
