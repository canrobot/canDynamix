# canDynamix
자율주행 자동차 프로젝트 입니다. (ROS기반)



[기본 목록]-------------------------------------------------------------------------------------------------- 

 1. 라즈베리파이  
  - http://mechasolution.com/shop/goods/goods_view.php?goodsno=329559&category=046001

 2. DC 모터    
 - DC 6V 90RPM n20 (encoder)  
   - https://www.aliexpress.com/item/DC-6V-90RPM-N20-Encoder-Motor-Reducer-Gear-Motor-DC-Gear-Motor-Popular/32808696006.html?spm=2114.search0104.3.8.1vNA8M&ws_ab_test=searchweb0_0,searchweb201602_3_10152_10065_10151_10068_10344_10345_10342_10343_10340_10341_10307_10060_10155_10154_10056_10055_10054_10537_10059_10536_10534_10533_10532_100031_10099_10338_10103_10102_5590020_10052_10053_10142_10107_10050_10051_10084_10083_5370020_10080_10082_10081_5610020_10110_10175_10111_10112_10113_10114_10312_10313_10314_10078_10079_10073,searchweb201603_25,ppcSwitch_5&btsid=3eedd6a2-5237-4817-8e98-6c2437aeba6f&algo_expid=de0cc9c4-d0f6-4069-8826-d1833d1c63b7-1&algo_pvid=de0cc9c4-d0f6-4069-8826-d1833d1c63b7
   꼭 6핀인지 확인하세요. 5핀인 경우는 엔코더의 홀센서가 1개 밖이 없어서 정역회전을 구분할 수 없습니다.

 - DFROBOT 100:1  엔코더 모터   
   - http://www.icbanq.com/P007122943
   - http://mechasolution.com/shop/goods/goods_view.php?goodsno=329716&category=131003
    
 3. n20용 브라켓 
 - http://mechasolution.com/shop/goods/goods_view.php?goodsno=539154&category=131015

 4. n20용 wheel 
  - http://mechasolution.com/shop/goods/goods_view.php?goodsno=539532&category=140003

 5. [Pololu] 1/2" 메탈 볼캐스터 (Pololu Ball Caster with 1/2″ Metal Ball) 
  - http://mechasolution.com/shop/goods/goods_view.php?goodsno=57&category=140006

 6. 9V 배터리 홀더 
 - http://mechasolution.com/shop/goods/goods_view.php?goodsno=71506&category=135002

 7. 9V 충전식 배터리 
 - https://www.aliexpress.com/item/2pcs-9V-battery-soshine-Original-9v-rechargeable-650mAh-9v-li-ion-batteries-1pcs-battery-charger-fast/32810384113.html?spm=2114.search0104.3.9.YW1B9u&ws_ab_test=searchweb0_0,searchweb201602_3_10152_10065_10151_10068_10344_10345_10342_10343_10340_10341_10307_10060_10155_10154_10056_10055_10054_10537_10059_10536_10534_10533_10532_100031_10099_10338_10103_10102_5590020_10052_10053_10142_10107_10050_10051_10084_10083_5370020_10080_10082_10081_5610020_10110_10175_10111_10112_10113_10114_10312_10313_10314_10078_10079_10073,searchweb201603_25,ppcSwitch_5&btsid=5ae4daea-331f-435e-b5c7-e1a93c7be23b&algo_expid=69dc78c7-8ebd-44f4-ab04-794529110cb6-1&algo_pvid=69dc78c7-8ebd-44f4-ab04-794529110cb6



[PCB 부품 목록] ------------------------------------------------------------------------------------------------


1. arduino nano(ch340)		http://www.devicemart.co.kr/1342039
2. mpu9250			http://www.devicemart.co.kr/1342892
3. 전원스위치			http://www.devicemart.co.kr/1795
4. 모터드라이버			https://www.aliexpress.com/item/Free-Shipping-New-BA6208-CD6208CS-SIP-9-30PCS/32673832095.html?spm=a2g0s.9042311.0.0.NQuNpX
5. 전원 DC잭			http://www.devicemart.co.kr/1322104
6. PCB서포트용 나사(M3X10)		http://www.devicemart.co.kr/12677
7. PCB서포트 금속 F-3mm		http://www.devicemart.co.kr/4595
8. PCB서포트 금속 M-10mm		http://www.devicemart.co.kr/3160
9. PCB서포트 금속 M-35mm		http://www.devicemart.co.kr/18884
10.칩저항 1608사이즈 J급 270Ω	http://www.devicemart.co.kr/19209
11.칩저항 1608사이즈 J급 470Ω	http://www.devicemart.co.kr/14142
12.칩저항 1608사이즈 J급 2KΩ		http://www.devicemart.co.kr/6079
13.칩저항 1608사이즈 J급 1KΩ		http://www.devicemart.co.kr/6058
14.칩저항 1608사이즈 J급 330Ω	http://www.devicemart.co.kr/19220
15.35155용 클림프			http://www.devicemart.co.kr/36450
16.5051용 클림프 (낱단자)		http://www.devicemart.co.kr/1128414
17.커넥터-53258-02			http://www.devicemart.co.kr/381
18.커넥터-MOLEX 51067-02		http://www.devicemart.co.kr/373
19.테스트[CH254]점퍼용 소켓- single 1x3P http://www.devicemart.co.kr/5490
20.테스트[CH254]점퍼용 소켓- single 1x2P http://www.devicemart.co.kr/5489
21.테스트[CH254]점퍼용 소켓- single 1x1P http://www.devicemart.co.kr/5488
22.테스트[CH254]점퍼용 클림프 (암)	    http://www.devicemart.co.kr/5710
23.핀헤더 Single 1x40Pin Straight(2.54mm) http://www.devicemart.co.kr/2825
24.Single 1x40Pin Rightangle(2.54mm)	http://www.devicemart.co.kr/1077101  <-- 노랑, 빨강 필수 검정 23번을 이용하세요.
25.P2285-LOCK			http://www.devicemart.co.kr/1795
26.LM2596SX-5.0			http://www.devicemart.co.kr/10037
27.SMD E/C 16V 470uF(85℃)/8Ø x10mm http://www.devicemart.co.kr/34601
28.SMD E/C 6.3V 220uF (85℃)/6.3Ø x6mm http://www.devicemart.co.kr/4252
29.SC1054D-33uH [SMD 파워 인덕터]	http://www.devicemart.co.kr/14012
30.부저-BTG-47			http://www.devicemart.co.kr/32828
31.스위치-IMMS-12V			http://www.devicemart.co.kr/2647
32.다이오드-SS14			http://www.devicemart.co.kr/11911
33.핀헤더소켓 Single 1x20 Straight(2.54mm) http://www.devicemart.co.kr/3587
34.핀헤더소켓 Dual 2x20 Straight(2.54mm) http://www.devicemart.co.kr/2828
