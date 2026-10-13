# 공개 전 체크리스트

이 저장소는 아직 비공개입니다(원격 없음). 공개(첫 푸시) 전에 아래를 확인합니다. 경위는
[작업 일지 2026-10-13](../worklog/2026-10.md)에 있습니다.

## 끝낸 것 (2026-10-13)

- [x] **라이선스**: MIT([LICENSE](../../LICENSE)). 바퀴 제어기의 프레임 코드는 직접 작성한
      `include/hover_protocol.h`로 바꿨고, GPL-3.0 업스트림 예제에서 복사했던 `hoverserial.h` · `util.h`는 뺐습니다
      (로컬 사본은 `firmware/hoverboard-mainboard/local/upstream-examples/`).
- [x] **메인보드 펌웨어**: GPL-3.0 업스트림을 고친 소스 · bin · 기록은 `firmware/hoverboard-mainboard/`에 로컬로만
      둡니다(git 제외). `git ls-files firmware/hoverboard-mainboard`가 비어 있어야 합니다.
- [x] **Wi-Fi 비밀번호**: `include/wifi_config.h`(git 제외)로 뺐고, 바퀴 제어기는 새 비밀번호로 플래시돼 있습니다.
- [x] **이력 정리**: 아래를 이력 전체에서 지웠습니다(`git filter-repo`, 원래 저장소에서 바로 실행).
  - 재배포 조건을 모르는 외부 자료: 휠 모터 참조 STEP, 그 형상이 들어간 `v28_wiring.step`, SPG 카탈로그 표 그림
  - GPL-3.0 파일과 코드: `hoverserial.h` · `util.h`, 일지 속 메인보드 `main.c` 패치 일곱 줄
  - 누비와 관계없는 그림 32장(ball_robot · gring · ib · sphex)
  - 개인 정보: 로컬 사용자 이름이 든 경로, 학교 이름, 옛 Wi-Fi 비밀번호

  지울 문자열 목록은 `docs/guides/publish-replacements.local.txt`(git 제외)에 있습니다. 이력을 다시 쓰기 전 상태는
  `.git/backup/nubi-before-rewrite-2026-10-13.bundle`에 백업했습니다. 이 백업에는 지운 내용이 그대로 있으니 공개할
  곳에 올리지 마세요.

  결과: 커밋 24개 중 23개의 해시가 바뀌었고(문서 속 해시도 새 값으로 고침), 최신 파일은 정리 전과 같습니다(트리 해시
  일치). 이 저장소의 커밋 해시는 이제 원래 개인 저장소(`lolind32-wheel`, `HoverboardVehicle`)의 해시와 다릅니다.

## 올리기 전에 할 것

- [ ] 바퀴 제어기를 새 프레임 코드(`hover_protocol.h`) 빌드로 플래시하고 통신을 확인합니다(텔레메트리 프레임 수가
      늘고, 바퀴가 명령대로 도는지). [ISSUES](../../firmware/wheel-controller/ISSUES.md) 참고.
- [ ] 그 사이에 외부 코드 · 자료나 개인 정보가 새로 커밋되지 않았는지 다시 찾습니다. 들어갔다면 아래 방법으로 이력
      정리를 다시 합니다.

```bash
git grep -I -n -i -E "(api[_-]?key|secret|token|passw|bearer|ghp_|sk-|copyright|gnu general public)" $(git rev-list --all) -- . ':!LICENSE'
```

- [ ] GitHub 설정의 "Block command line pushes that expose my email"이 켜져 있으면 커밋 이메일 때문에 푸시가
      막힙니다. 이메일은 그대로 두기로 했으므로(2026-10-13) 설정을 확인합니다.
- [ ] GitHub에 `nubi` 저장소를 만들고(public), 원격을 붙여 푸시합니다.

```bash
git remote add origin https://github.com/<계정>/nubi.git
git push -u origin main
```

- [ ] 문서 링크를 한 번 더 확인합니다.

## 이력을 다시 정리해야 할 때

```bash
# 백업
git bundle create .git/backup/nubi-before-rewrite-<날짜>.bundle --all
# 도구 (임시 venv)
py -3.12 -m venv ../frvenv
../frvenv/Scripts/python.exe -m pip install git-filter-repo
# 지울 경로는 --invert-paths --path ... / --path-glob ..., 바꿀 문자열은 --replace-text
../frvenv/Scripts/git-filter-repo.exe --force --invert-paths --path "<경로>" --replace-text docs/guides/publish-replacements.local.txt
```

확인할 것: 지운 경로가 어느 커밋의 트리에도 없는지(`git ls-tree -r --name-only <커밋>`, 병합 커밋 때문에 `git log
--name-only`만으로는 부족), 바꾼 문자열이 이력에 없는지, 최신 트리가 정리 전과 같은지(`git rev-parse HEAD^{tree}`),
문서 속 옛 해시를 `.git/filter-repo/commit-map`으로 고쳤는지.
