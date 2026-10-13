# 포트 칸 부품 · 배기 팬 국내 구매처 조사 (2026-10-05)

조건: 국내 판매자 · 국내 재고만, 방수 등급 안 따짐 (D-01, D-08). 상품 페이지를 열어 확인한 값만, 나머지는 [추정].
이 조사로 내린 결정은 [`../DECISIONS.md`](../DECISIONS.md) D-09 · D-10.

## 배기 팬 · 가드

| 항목 | 상품 | 판매처 | 가격 | 치수 · 사양 |
|---|---|---|---|---|
| 팬 ★ | [EVERCOOL EC8025H05B 2P](https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=1047506) | 컴퓨존 (에버쿨 직영몰 8,500) | 8,740 | 80 × 80 × 25 (±0.5), 구멍 4-Ø4.4 간격 71.5, 케이블 300 (2510 2P). **5 V 0.28 A 1.4 W**, 3000 rpm, 36.48 CFM, < 33 dBA, 2 볼. 라벨면 = 배기면 |
| 팬 (조용한 대안) | [EC8025M05CA 3P](http://evercool.co.kr/shop/step1.php?number=6480&b_code=B20200507055324) | 에버쿨 직영몰 | 6,200 | 같은 프레임, 0.18 A, 2400 rpm, 23 CFM, < 28 dBA |
| 가드 ★ | [EVERCOOL EFG-S80](https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=351400) | 컴퓨존 | 380 | 바깥 링 Ø76.5, 가운데 Ø14.2, 링 간격 4.5, 와이어 Ø1.8, 높이 5.5, 고리 Ø4.6 |
| 필터 (선택) | [EVERCOOL EFM-S80](https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=351408) | 컴퓨존 | 1,720 | 구멍 간격 표기 71.2 (팬 71.5 와 0.3 차이), 구멍 Ø4.2 — 안 씀 (배기 쪽이라 효용 작음) |

고정: **M4×20 + 나일론 너트 + 와셔** (가드 1.8 + PC 3 + 팬 플랜지 약 4). PC 에 록타이트 금지 (응력균열).

## 포트 칸 (칸 안 195 × 55, 깊이 35, 안벽 2.5)

| 자리 | 상품 | 판매처 | 가격 | 패널 구멍 | 치수 |
|---|---|---|---|---|---|
| 충전 단자 | [GX16 3P set](https://rmpower.co.kr/shop_view/?idx=63) | 알엠파워 (충전기와 같은 곳) | 1,500 | M16 × 1, Ø16 + D 컷 14.6 (제노몰 HAC5012 도면) | 플랜지 Ø19 × 1.8, 전장 20.1 · 패널 뒤 13.1, 5~7 A 250 V |
| 키 스위치 | [HAS1911 2 단 메탈 키스위치 19 mm](https://www.jenomall.com/goods/goods_view.php?goodsNo=1000009235) | 제노몰 | 6,600 | M19, Ø19 + D 컷 16.5 | 베젤 Ø21.8 × 6.5, 패널 뒤 33.8, 너트 맞변 22, 4 A 28 VDC. 키는 ON · OFF 둘 다 빠짐 |
| 전압계 | [VaneAims 원형 LED DC 4~100 V (FXB16-22DSV)](https://www.eleparts.co.kr/goods/view?no=12829953) | 엘레파츠 | 6,160 | Ø22 [추정] | 앞 Ø29.5, 몸통 Ø20.5, 전장 47.6, 2 선 자체 전원 0.5 W. (사각형 대안 0~99.9 V 4,950 은 48 × 28.8 로 칸에 안 들어가고 3 선식) |
| ESP32 USB-C | [MAXTEK MT723 USB-C 수↔암 판넬용 30 cm](https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=1373361) | 컴퓨존 | 3,200 | 사각 컷아웃 + 귀 나사 2 (간격 17) [컷아웃 크기 추정] | 귀 폭 25, 하우징 26 |
| USB-A ×2 | [Coms HA079 USB 3.0 원형 벌크헤드](https://www.comsmart.co.kr/cmart/shop/item.php?it_id=1763692829) | 컴스마트 | 7,000 | Ø22 [사진 비례] | 전장 33, 앞 Ø28 (같은 계열 MAXTEK 도면). 암-암 — 안쪽 A 수-수 케이블 별도 |
| RJ45 | [Coms HA084 RJ45 CAT6 원형 벌크헤드](https://www.comsmart.co.kr/cmart/shop/item.php?it_id=1763689525) | 컴스마트 | 6,000 | Ø22 [사진 비례] | HA079 와 같은 몸체 |

- 한 줄로는 앞면 폭 합이 약 204 mm 라 195 에 안 들어간다 → **두 줄**: 왼쪽 전압계 · USB-C, 가운데 원형 3, 오른쪽 키 · 충전 (바깥에서 볼 때).
- **충전기 항공단자가 GX16 인지 GX12 인지, 충전기 쪽 성별이 무엇인지** 알엠파워에 물어야 한다. 이 세트는 패널 쪽이 수 (핀) — 배터리 쪽에 쓰면 42 V 핀이 노출된다.
- 원형 벌크헤드의 나사 규격 · 구멍 지름 · 너트 치수는 어디에도 없다 → 실물을 받아 재거나 시험 출력으로 맞춘다.
- 값싼 ESP32 보드는 USB-C CC 저항이 없어 C-C 케이블로 안 잡힐 수 있다 — A-C 로 쓰는 편이 안전.
