#!/usr/bin/env bash
# Xtion Pro Live 의 마이크 (USB 오디오) 를 Jetson 이 쓰지 않게 한다 — sudo 로 한 번 실행.
#   sudo bash xtion_no_audio.sh          설치 + 지금 꽂힌 Xtion 의 오디오도 바로 뗀다
#   sudo bash xtion_no_audio.sh --undo   되돌리기 (규칙 지우기, 다시 꽂으면 마이크가 돌아온다)
#
# 까닭 (2026-10-14): 커널에 'usb 1-2.1: usb_set_interface failed (-110)' 이 5 초마다 찍혔다. 로그인 화면 (gdm) 의 오디오
# 서버가 Xtion 마이크 (ALSA 카드 PrimeSense Device) 를 열다 시간 초과를 내고, 그동안 깊이 카메라 쪽 첫 명령
# ('Getting hardware versions', OpenNI 로그) 이 대답을 못 받아 장치 열기가 끝없이 멈췄다.
# 오디오 인터페이스 (class 01) 만 authorized=0 으로 막는다. 깊이 · 컬러 (class ff, OpenNI) 는 그대로.
set -euo pipefail
RULE=/etc/udev/rules.d/99-xtion-no-audio.rules
if [ "${1:-}" = "--undo" ]; then
  rm -f "$RULE" && udevadm control --reload && echo "지웠다: $RULE (Xtion 을 다시 꽂으면 마이크가 돌아온다)"
  exit 0
fi
cat > "$RULE" <<'EOF'
# ASUS Xtion Pro Live (1d27:0601) 마이크를 쓰지 않는다 — 오디오 서버가 열다 실패하며 깊이 카메라 제어를 막는다 (NUBI 2026-10-14)
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_interface", ATTRS{idVendor}=="1d27", ATTRS{idProduct}=="0601", ATTR{bInterfaceClass}=="01", ATTR{authorized}="0"
EOF
udevadm control --reload
echo "규칙: $RULE"
# 지금 꽂혀 있는 Xtion 의 오디오 인터페이스도 바로 막는다 (다시 꽂지 않아도 되게)
for dev in /sys/bus/usb/devices/*; do
  [ -f "$dev/idVendor" ] && [ "$(cat "$dev/idVendor")" = 1d27 ] && [ "$(cat "$dev/idProduct")" = 0601 ] || continue
  for itf in "$dev"/*:*; do
    if [ "$(cat "$itf/bInterfaceClass")" = 01 ]; then
      echo 0 > "$itf/authorized"
      echo "막음: $(basename "$itf") (오디오)"
    fi
  done
done
echo "끝 — 깊이 카메라가 그래도 안 열리면 Xtion USB 를 한 번 뽑았다 꽂는다"
