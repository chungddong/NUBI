# 공개 전 체크리스트

이 저장소는 아직 비공개입니다(원격 없음). 공개(첫 푸시) 전에 아래를 차례로 합니다. 2026-10-13에 정리한 내용이고,
그날의 경위는 [작업 일지 2026-10-13](../worklog/2026-10.md)에 있습니다.

## 1. 라이선스

- [ ] 바퀴 제어기의 `include/hoverserial.h` · `util.h`는 GPL-3.0 프로젝트(RoboDurden
      Hoverboard-Firmware-Hack-Gen2.x-GD32)에서 그대로 복사한 파일입니다. MIT로 공개하려면 실제로 쓰는 부분
      (명령 · 텔레메트리 구조체, 송수신 함수)을 직접 다시 쓰고 두 파일을 뺍니다. 실기에서 통신을 한 번 확인합니다.
- [ ] 저장소 루트에 `LICENSE`를 넣습니다 (MIT로 할 예정, 2026-10-13 논의).
- [ ] 루트 README의 "출처와 라이선스" 절을 고칩니다.
- [ ] 메인보드 펌웨어(GPL-3.0 업스트림을 고친 것)의 소스 · `.bin`이 `firmware/hoverboard-mainboard/local/`에만 있고
      git에 없는지 확인합니다.

## 2. 비밀값과 개인 정보

- [ ] 바퀴 제어기를 새 Wi-Fi 비밀번호(`include/wifi_config.h`, git에서 제외)로 다시 플래시합니다. 예전 비밀번호는
      바퀴 제어기 커밋 이력에 남아 있어서, 기기 비밀번호를 바꿔야 쓸모가 없어집니다.
- [ ] 전체 이력에서 비밀값 패턴을 다시 찾습니다.

```bash
git grep -I -n -i -E "(api[_-]?key|secret|token|passw|bearer|ghp_|sk-)" $(git rev-list --all) -- .
```

## 3. 이력 정리

재배포 조건을 모르는 외부 파일과 개인 문자열을 **이력 전체에서** 지웁니다. 지금 파일에서는 이미 빠져 있지만
(2026-10-13) 예전 커밋에는 남아 있습니다. 첫 푸시 직전에, 다른 작업이 모두 커밋된 상태에서 합니다.

| 지울 것 | 위치 |
| --- | --- |
| 휠 모터 참조 STEP | `refmodel/Motor, BLDC, Hoverboard Wheel, 6.5in.stp` (통합 전 경로 포함) |
| 휠 모터 형상이 들어간 조립 STEP | `models/vehicle/v28_wiring/output/v28_wiring.step` (통합 전 경로 포함) |
| 제조사 카탈로그 표 그림 | `*_ref_spg_*.png` |
| 로컬 사용자 이름이 든 경로, 학교 이름 | 문자열 치환. 목록은 `docs/guides/publish-replacements.local.txt` (git에서 제외된 로컬 파일) |

```bash
# 0) 백업
git bundle create ../nubi-before-rewrite.bundle --all

# 1) 도구 (임시 venv)
py -3.12 -m venv ../frvenv
../frvenv/Scripts/python.exe -m pip install git-filter-repo

# 2) 새로 복제한 곳에서 실행 (원래 작업 폴더는 그대로 둔다)
git clone --no-local . ../nubi-publish
cp docs/guides/publish-replacements.local.txt ../replacements.txt
cd ../nubi-publish
../frvenv/Scripts/git-filter-repo.exe --invert-paths \
  --path "refmodel/Motor, BLDC, Hoverboard Wheel, 6.5in.stp" \
  --path "hardware/vehicle/refmodel/Motor, BLDC, Hoverboard Wheel, 6.5in.stp" \
  --path "models/vehicle/v28_wiring/output/v28_wiring.step" \
  --path "hardware/vehicle/models/vehicle/v28_wiring/output/v28_wiring.step" \
  --path-glob "*_ref_spg_*.png" \
  --replace-text ../replacements.txt
```

확인할 것:

- [ ] 위 파일들이 이력 어디에도 없다: `git log --all --name-only --format= | sort -u`
- [ ] 치환 대상 문자열이 이력 어디에도 없다 (`git grep` + `$(git rev-list --all)`)
- [ ] 최신 커밋의 트리가 원래 작업 폴더의 것과 같다: `git rev-parse HEAD^{tree}` 비교
- [ ] 바퀴 제어기 커밋(`6e0a3a6`, `cd922a5`, `55f450a` …)의 해시가 그대로다
- [ ] 문서에 적힌 옛 해시를 새 해시로 고친다. 대응표는 `.git/filter-repo/commit-map`. 2026-10-13 시험에서 바뀐 곳은
      루트 README의 `git log 02f205d^2`와 10월 일지의 HoverboardVehicle 커밋 목록이었다 (커밋 메시지 안의 해시는
      도구가 자동으로 고친다)

2026-10-13에 시험으로 돌렸을 때: 바뀐 커밋 12개(차체 쪽 7 + 통합 이후 5), 바퀴 제어기 쪽과 통합 초기 커밋 9개는 해시
유지, 트리 동일, 저장소 76 → 71 MB.

## 4. 올리기

- [ ] GitHub 설정의 "Block command line pushes that expose my email"이 켜져 있으면 커밋 이메일 때문에 푸시가
      막힙니다. 이메일은 그대로 두기로 했으므로(2026-10-13) 설정을 확인합니다.
- [ ] 정리한 복제본(`../nubi-publish`)에 원격을 붙여 푸시합니다.
- [ ] 원래 작업 폴더는 이력이 달라졌으므로, 푸시한 저장소를 새로 클론해 쓰거나 원격 브랜치로 맞춥니다.
- [ ] 문서 링크 검사를 한 번 더 합니다.
