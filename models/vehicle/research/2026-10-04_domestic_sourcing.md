# 국내 구매처 조사 (2026-10-04)

조건: **국내 판매자 · 국내 재고 배송만** (해외직구 · 구매대행 · AliExpress · Amazon 제외). Jetson · ESP32 제외.
가격은 상품 페이지를 열어 확인한 값이고, VAT 포함 여부는 판매처마다 표기가 다르다. 디바이스마트는 이 PC 에서 접속이 안 돼 빼고 봤다.
이 조사로 내린 결정은 [`../DECISIONS.md`](../DECISIONS.md) 에 있다.

| 부품 | 상품 | 판매처 | 단가 (원) | 배송 | 비고 |
|---|---|---|---|---|---|
| 배터리 | [볼턴 36V 20Ah 기본형](https://www.voltonbattery.com/pedelec_36v/?idx=88) | 볼턴 | 480,000 | 제작 2~3 영업일 | **스펙시트 15 A / 500 W** → 30 A 기준 미달 (D-04) |
| 배터리 대안 | [알엠파워 삼성 21700 36V 20Ah 10S4P](https://rmpower.co.kr/shop_view/?idx=514) | 알엠파워 | 548,000 | — | 최대 30 A (연속 미기재) |
| 배터리 대안 | [파워크래프트 36V 15Ah 35A BMS](https://www.powercraft.co.kr/goods/goods_view.php?goodsNo=1616) | 파워크래프트 | 문의 | 3~15 일 | 연속 ≤30 A, 피크 42 A |
| 충전기 | [볼턴 42V 2A](https://www.voltonbattery.com/shop_view/?idx=114) | 볼턴 | 40,000 | 오늘 출발 | KC |
| 모터 드라이버 | [ZLAC8015D](https://smotion.co.kr/product/zltech-zlac8015d-canopenrs-485-%EC%A0%9C%EC%96%B4%EC%9A%A9-2%EC%B6%95-wheel-in-motor%EC%9D%B8%ED%9C%A0%EB%AA%A8%ED%84%B0%EC%9A%A9-%EB%93%9C%EB%9D%BC%EC%9D%B4%EB%B2%84/206/) | 에스모션 | 문의 | — | 다나와 · 11번가는 전부 구매대행 |
| 드라이버 대안 | [엠디로봇 MD400T](https://www.mdrobot.co.kr/BLDCmotordriver-store-dualchannel/?idx=161) | 엠디로봇 | 374,000 | 최대 2 주 | 166 × 127 × 42.5, 프로토콜 다름 |
| 메인 차단기 | [DC 차단기 1P 250VDC 40A](https://mega-tec.co.kr/product/detail.html?product_no=504&cate_no=169&display_group=1) | 메가에너텍 | 7,000 | 1~3 일 | 18 × 80 × 71, DIN 레일. MIDI/ANL 58 V 퓨즈는 국내 재고 없음 |
| 모터 전원 SSR | [운영 WYMS1C140D4](https://wyesmarket.com/product/detail.html?product_no=1292&cate_no=102&display_group=1) | 와이스마켓 | 49,760 | 3~7 일 | DC 100 V 40 A, 116 × 45 × 105. Fotek 정품 국내 확인 불가 · 위조품 문제 → **v20 에서 컨택터로 바꿈 (D-05)** |
| 젯슨 전원 | [MEAN WELL RSD-60L-12](https://meanwell.co.kr/product/detail.html?product_no=799&cate_no=52&display_group=1) | 싸인라이트 | 74,800 | 2~4 일 | 입력 18~72 V → 12 V 5 A (D-07). 민웰몰 65,990 |
| 19 V 대안 | [디오시스템 DKP-53](https://smpskorea.com/product/detail.html?product_no=748&cate_no=190&display_group=1) | 디오시스템 | 149,000 | 2~5 일 | 입력 하한 35~36 V — 배터리가 처지면 꺼질 수 있어 탈락 |
| DC-DC 5V | [디오시스템 DKP-55](https://smpskorea.com/product/detail.html?product_no=741&cate_no=126&display_group=1) | 디오시스템 | 82,000 | 2~5 일 | 입력 8~60 V, 5 V 10 A, 64 × 57 × 22 |
| IMU | [Adafruit BNO085](https://mechasolution.com/goods/goods_view.php?goodsNo=604557) | 메카솔루션 | 40,000 | **품절** | 나머지는 해외 입고 → **iAHRS 로 바꿈 (D-06)** |
| 뎁스 카메라 | [Orbbec Gemini 336](https://www.icbanq.com/P016230044) | 아이씨뱅큐 | 462,000 | 1 주 이내 | 엘레파츠 550,000 (VAT 포함) |
| 카메라 케이블 | [Coms IK045 USB3 A↔C 1m 양쪽 나사](https://www.eleparts.co.kr/goods/view?no=18547167) | 엘레파츠 | 9,800 | 약 3.5 일 | 카메라 포트 나사 규격 확인 필요 |
| 비상정지 | [Autonics SF2ER-E2RB](https://www.eleparts.co.kr/goods/view?no=13038016) | 엘레파츠 | 23,100 | 재고 시 2~3 일 | |
| 비상정지 명판 | [Autonics SF2ER-B3](https://e-punglim.co.kr/product/detail.html?product_no=37361) | 풍림오토테크 | 5,150 | 당일~4 일 | |
| 배기 팬 | [EVERCOOL EC8025H05B 5V](https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=1047506) | 컴퓨존 | 8,740 | 오늘 출발 | 대안 [메카솔루션 FNM-S-8025 5V](https://www.mechasolution.com/goods/goods_view.php?goodsNo=578322) 1,540 |
| 커넥터 | [XT60H 수](https://www.eleparts.co.kr/goods/view?no=13428798) · [암](https://www.eleparts.co.kr/goods/view?no=13428797) | 엘레파츠 | 638 | 24 시간 | AMASS |
