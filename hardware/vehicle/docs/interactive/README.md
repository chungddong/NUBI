# 인터랙티브 설명 페이지

브라우저에서 파일을 **그대로 열면 된다.** 서버도 인터넷도 필요 없다 —
CSS·JS·SVG 가 전부 파일 안에 들어 있다.

| 파일 | 무엇 |
|---|---|
| [lever.html](lever.html) | 바퀴가 프레임보다 바깥에 있으면 왜 항상 젖히는 힘이 생기나. 슬라이더로 팔 길이를 바꾸면 모멘트·볼트 인발·기울기가 따라온다 |

관련 정지 이미지:

- `../images/63_why_tipping_moment.png` — 3단 설명 (형상 / 젖힘 / 실측)
- `../images/64_tipping_animation.gif` — 레일 고정 상태에서 젖혀지는 움직임

두 이미지는 스크립트로 다시 굽는다:

```
.venv/Scripts/python.exe tools/figures/tipping_moment.py
```

## 숫자를 고칠 때

`lever.html` 의 수치는 **손으로 박아 넣은 값**이다. 형상이나 하중을 바꾸면
같이 고쳐야 한다. 출처는 파일 맨 위 주석에 적어 뒀다.

| 값 | 출처 |
|---|---|
| 팔 길이 53.7 mm | `parameters.AXLE_FLAT_Y0 - AXLE_FLAT_LENGTH/2` |
| 설계하중 800 N | `parameters.DESIGN_W` |
| 레일 폭 30 mm | `parameters.EXTRUSION_W` |
| 너트 한계 1500 N | `parameters._TNUT_PULLOUT[6.0]` — **보수 하한, 미실측** |
| 변위·응력 3종 | `models/hoverboard_mount/output/fea/` → `tools/fea/structural_stress.py` |

## 왜 정적 HTML 인가

뷰어(ocp_vscode)는 서버가 떠 있어야 하고, PNG 는 값을 바꿔 볼 수 없다.
이 페이지는 둘 다 아니어서 나중에 혼자 열어 봐도 그대로 동작한다.
