# canDynamix
자율주행 자동차 프로젝트 입니다. (ROS기반)



기본 목록 
===

### 1. 라즈베리파이  
  - http://mechasolution.com/shop/goods/goods_view.php?goodsno=329559&category=046001

### 2. DC 모터    
 - DC 6V 90RPM n20 (encoder)  
   - https://www.aliexpress.com/item/DC-6V-90RPM-N20-Encoder-Motor-Reducer-Gear-Motor-DC-Gear-Motor-Popular/32808696006.html?spm=2114.search0104.3.8.1vNA8M&ws_ab_test=searchweb0_0,searchweb201602_3_10152_10065_10151_10068_10344_10345_10342_10343_10340_10341_10307_10060_10155_10154_10056_10055_10054_10537_10059_10536_10534_10533_10532_100031_10099_10338_10103_10102_5590020_10052_10053_10142_10107_10050_10051_10084_10083_5370020_10080_10082_10081_5610020_10110_10175_10111_10112_10113_10114_10312_10313_10314_10078_10079_10073,searchweb201603_25,ppcSwitch_5&btsid=3eedd6a2-5237-4817-8e98-6c2437aeba6f&algo_expid=de0cc9c4-d0f6-4069-8826-d1833d1c63b7-1&algo_pvid=de0cc9c4-d0f6-4069-8826-d1833d1c63b7
   꼭 6핀인지 확인하세요. 5핀인 경우는 엔코더의 홀센서가 1개 밖이 없어서 정역회전을 구분할 수 없습니다.

 - DFROBOT 100:1  엔코더 모터   
   - http://www.icbanq.com/P007122943
   - http://mechasolution.com/shop/goods/goods_view.php?goodsno=329716&category=131003
    
### 3. n20용 브라켓 
 - http://mechasolution.com/shop/goods/goods_view.php?goodsno=539154&category=131015

### 4. n20용 wheel 
  - http://mechasolution.com/shop/goods/goods_view.php?goodsno=539532&category=140003

### 5. [Pololu] 1/2" 메탈 볼캐스터 (Pololu Ball Caster with 1/2″ Metal Ball) 
  - http://mechasolution.com/shop/goods/goods_view.php?goodsno=57&category=140006

### 6.  서포트 
 - 서포트 (M3 35mm) :  http://www.devicemart.co.kr/18884
 - 서포트 (M3 10mm) :  http://www.devicemart.co.kr/3160
 - 볼트 (M3 PCB서포트용 ) : http://www.devicemart.co.kr/12677
 - 너트 (M3 PCB서포트용 ) : http://www.devicemart.co.kr/4595
   
   주의 : 2.5파이를 구매하여야 Raspberry pi의 홀에 적당합니다. 그런데 2.5파이 35mm를 구하기가 힘듭니다. 따라서 위의 링크는 전부 3파이 서포트 들입니다. Raspberry pi호를 넓혀서 사용하셔야합니다.

### 7. 9V 배터리 홀더 
 - http://mechasolution.com/shop/goods/goods_view.php?goodsno=71506&category=135002

### 8. 9V 충전식 배터리 
 - https://www.aliexpress.com/item/2pcs-9V-battery-soshine-Original-9v-rechargeable-650mAh-9v-li-ion-batteries-1pcs-battery-charger-fast/32810384113.html?spm=2114.search0104.3.9.YW1B9u&ws_ab_test=searchweb0_0,searchweb201602_3_10152_10065_10151_10068_10344_10345_10342_10343_10340_10341_10307_10060_10155_10154_10056_10055_10054_10537_10059_10536_10534_10533_10532_100031_10099_10338_10103_10102_5590020_10052_10053_10142_10107_10050_10051_10084_10083_5370020_10080_10082_10081_5610020_10110_10175_10111_10112_10113_10114_10312_10313_10314_10078_10079_10073,searchweb201603_25,ppcSwitch_5&btsid=5ae4daea-331f-435e-b5c7-e1a93c7be23b&algo_expid=69dc78c7-8ebd-44f4-ab04-794529110cb6-1&algo_pvid=69dc78c7-8ebd-44f4-ab04-794529110cb6



PCB 부품 목록 
===

1. arduino nano  http://www.devicemart.co.kr/1342039
2. mpu9250       http://www.devicemart.co.kr/1342892
3. 전원스위치        http://www.devicemart.co.kr/1795
4. 모터드라이버      https://www.aliexpress.com/item/Free-Shipping-New-BA6208-CD6208CS-SIP-9-30PCS/32673832095.html?spm=a2g0s.9042311.0.0.NQuNpX
5. 전원 DC잭       http://www.devicemart.co.kr/1322104
