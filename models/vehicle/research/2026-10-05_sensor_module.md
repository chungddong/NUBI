# 센서 모듈 부품 국내 구매처 조사 (2026-10-05)

결정은 [`../DECISIONS.md`](../DECISIONS.md) D-17. 조건: 국내 재고 · 국내 배송.

## 2D 라이다

| 후보 | 판매처 | 가격 | 주변광 | 거리 | 비고 |
|---|---|---|---|---|---|
| ★ [LDROBOT STL-19P (D500 키트)](https://www.eleparts.co.kr/goods/view?no=13532871) | 엘레파츠 | 172,700 (VAT 포함) | **60 klux** | 백 12 m · 흑 8 m | 54 × 46.29 × 35, 스캔 = 바닥 + 27.2, Ø2.5 ×3, 5 V 290 mA, 45 g, UART 230400 + CP2102 USB |
| [RPLIDAR C1](https://m.eduino.kr/product/slamtec-rplidar-c1-360도-12m-거리측정-2d-lidar-센서/9805/) | 에듀이노 | 112,500 | 40 klux | 백 12 m · 흑 6 m | 55.6 × 55.6 × 41.3, 스캔 29.8, M2.5 ×4 (43 □), 110 g |
| RPLIDAR S2 / YDLIDAR T-mini Plus · X4 Pro / A2M12 | — | — | — | — | 국내 재고 없음 (주문 · 해외) |

- [STL-19P 데이터시트](https://files.waveshare.com/wiki/D500-LiDAR-Kit/LDROBOT_STL-19P_Datasheet_EN_v1.0(1).pdf) — ROS2 는 `ldlidar_stl_ros2` 의 LD19 설정.

## GNSS

| 급 | 후보 | 판매처 | 가격 | 비고 |
|---|---|---|---|---|
| 일반 ★ | [아센코리아 GPS630](https://www.eleparts.co.kr/goods/view?no=17686184) | 엘레파츠 | 75,394 | u-blox M8, USB, 안테나 일체 · 자석, 55 × 46 × 14.5, 2.5 m CEP |
| 일반 | [Holybro M10](https://dronewe.com/product/holybro-m10-gps-픽스호크/6198/) | 드론위 | 86,900 | UART (USB 변환기 필요) |
| RTK | [Holybro H-RTK ZED-F9P Rover](https://dronewe.com/product/holybro-h-rtk-zed-f9p-rover-픽스호크/6338/) | 드론위 | 684,200 | Ø78 × 22, IP66, UART |
| RTK | [Holybro H-RTK F9P Ultralight](https://dronewe.com/product/holybro-h-rtk-f9p-ultralight-픽스호크/6208/) | 드론위 | 599,500 | |

## 상태 표시등

| 후보 | 판매처 | 가격 | 비고 |
|---|---|---|---|
| ★ [큐라이트 ST45ML-BZ-3-12-RAG](https://alphaeplus.com/product/%ED%81%90%EB%9D%BC%EC%9D%B4%ED%8A%B8-st45ml-bz-3-12-%EC%8B%9C%EA%B7%B8%EB%84%90-%ED%83%80%EC%9B%8C%EB%9E%A8%ED%94%84-%EB%B6%80%EC%A0%80%EB%82%B4%EC%9E%A5-%EC%A0%90%EB%93%B1%ED%98%95-3%EB%8B%A8-dc12v-%EC%A7%81%EB%B6%80%ED%98%95-%EC%99%B8%EA%B2%BD-45mm-led-%ED%83%80%EC%9B%8C%EB%9E%A8%ED%94%84/6120/) | 알파전기 | 65,670 | Ø45, 장착면 위 188.9 (68 + 40 × 3), 스터드 M4 ×3 PCD 30, 12 V 단당 40 mA + 부저 40 mA, 90 dB |
| 큐라이트 QT50ML-BZ | 굿케이디 | 89,410 | Ø50, 주문생산 |
| 5 V WS2812 링 16 구 | 메카솔루션 | 5,280 | 부저 없음, 확산 캡 출력 |

## 2020 부속 · 판재 ([동진프로파일](http://www.profileok.com/shop/list.php?ca_id=10))

- DF2020 1 m 4,630 (VAT · 절단 포함), 100 mm 800 · 200 mm 1,260. 미스미 HFS5-2020 · 엘레파츠 DF2020 은 더 비쌈.
- M5 너트 (20 계열): 반달 인서트 DBIN20*M5 150 (홈 중간), 스프링너트 200. 유럽형 슬롯 6 이면 미스미 HNTAJ5-5 533.
- 코너 브래킷 DCB2025 (ZN) 340 + 볼트 · 너트 세트 180. 끝 캡 DEC2020 100.
- 판재: 5T AL6061 300 × 260 19,590, 3T AL5052 300 × 260 10,880 (재단만, 구멍은 직접).
