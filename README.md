# Project Name  
STM32F013RB를 이용한 센서 데이터 수집 저장

## Feature  
1.1초마다 센서에서 데이터를 수집  
2.10분마다 데이터의 평균값을 계산  
3.평균값과 RTC로 측정한 현재 시각을 ILI9341로 디스플레이  
4.평균값과 현재 시각을 SD카드에 저장  
5.모뎀과의 연결 확인 후 서버로 데이터 전송  
5-1. 데이터 송신이 안됐을 경우 원인 파악 후 오류 파일에 원인과 끊긴 시간대를 SD카드에 저장 후 재 연결

## Use Library link  
[ILI9341] : https://github.com/eziya/STM32_HAL_ILI9341  
[DHT11] : https://github.com/mesutkilic/DHT11-STM32-Library/blob/master/mk_dht11.h  
[SD module] : https://github.com/eziya/STM32_SPI_SDCARD
