# 전조등 국내 구매처 조사 (2026-10-05)

조건: 국내 재고 · 12 V 또는 5 V 직결 · 한 개 2~6 W · 앞판 (PC 3t) 장착 쉬움 · 렌즈/반사경으로 전조등처럼 보일 것.
결정은 [`../DECISIONS.md`](../DECISIONS.md) D-11.

| # | 상품 | 판매처 | 가격 | 치수 | 전기 | 비고 |
|---|---|---|---|---|---|---|
| 1 ★ | [건식 LED MR16 12V 5W 집중형](https://www.11st.co.kr/products/3733952699) | 11번가 오렌지라이팅 (우리조명) | 1,720 | Ø50 × 49 (핀 포함), GU5.3 | 12 V 5 W **0.41 A, 410 lm, 빔 50°**, 6500 K | 소켓 별도 [GU5.3 사기 소켓](https://www.11st.co.kr/products/7292435290) 1,000. 엘레파츠 우리조명 MR16 5W 2,810 |
| 2 | [6W 라이트 + 경적](https://www.e-pm.kr/product/detail.html?product_no=907) | 전동부품몰 | 9,000 | 렌즈 약 Ø55 · 길이 약 50, 금속 스테이 | 12~60 V 6 W | 밖으로 돌출, lm 미기재 |
| 3 | [55mm 빅파워 LED](https://www.e-pm.kr/product/detail.html?product_no=429) | 전동부품몰 | 12,000 | Ø55 × 32, ㄷ자 브래킷 | 12 V 10 W | 프로젝터 + 엔젤링, 2 개 1.7 A |
| 4 | [미니 U7 10W](https://www.e-pm.kr/product/detail.html?product_no=1232) | 전동부품몰 | 12,000 | 90 × 55 × 50 | 12~70 V 10 W | 모드 전환식 — PWM 불가 |
| 5 | [파워LED 23mm 3W](https://www.e-pm.kr/product/detail.html?product_no=390) | 전동부품몰 | 3,500 | Ø23 × 16, 양면테이프 | 12 V 3 W | 작음 |

MOSFET 모듈 (ESP32 3.3 V → 12 V 로우사이드)

| 상품 | 판매처 | 가격 | 사양 |
|---|---|---|---|
| ★ [DFRobot DFR0457 Gravity MOSFET](https://www.mechasolution.com/goods/goods_view.php?goodsNo=539634) | 메카솔루션 | 6,270 (VAT 포함) | VIN 5~36 V, 0~20 A, 로직 3.3~10 V, PWM ≤ 1 kHz, 44 × 32 |
| [PP-A603 15A MOSFET 트리거 모듈](https://www.icbanq.com/P019040075) | 아이씨뱅큐 | 924 | 5~36 V, 트리거 3.3~20 V, PWM ≤ 20 kHz, 치수 미기재 |

- IRF520 모듈은 3.3 V 로 충분히 안 켜져 제외. 쿠팡 · G마켓 · 네이버는 접속 막힘.
