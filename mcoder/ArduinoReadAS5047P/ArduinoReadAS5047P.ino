#include <SPI.h>

#define CSPin 10
#define ANGLECOM 0x3FFF

/************************************************
Arduino UNO 读取AS5047P/AS5048的角度(SPI)，
原创代码，已经过测试
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120431638
          《SimpleFOC移植STM32（三）—— 角度读取》
创建日期：20210909
作    者：loop222 @郑州
************************************************/
/*******************************************************/
void setup() {
  pinMode(CSPin,OUTPUT);
  digitalWrite(CSPin, HIGH);
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  Serial.println("AS5047P Ready!");
  delay(1000);
}

void loop() {
  //Serial.println(((float)(readRegister(0xFFFf)&0x3FFF)*360)/16384);
  Serial.println(readRegister(0xFFFC));
  delay(1000);
}
/*******************************************************/
/*******************************************************/
uint16_t readRegister(uint16_t address){
  uint16_t receivedData;
  
  digitalWrite(CSPin, LOW);
  SPI.transfer16(address);
  digitalWrite(CSPin, HIGH);
  delayMicroseconds(1); 
  
  digitalWrite(CSPin, LOW);
  receivedData = SPI.transfer16(0x0000);
  digitalWrite(CSPin, HIGH);
  delayMicroseconds(1);
  
  return receivedData;
}
/*******************************************************/
void writeRegister(uint16_t address, uint16_t value){
  
  digitalWrite(CSPin, LOW);
  SPI.transfer16(address);
  digitalWrite(CSPin, HIGH);
  delayMicroseconds(1);
  
  digitalWrite(CSPin, LOW);
  SPI.transfer16(value);
  digitalWrite(CSPin, HIGH);
  delayMicroseconds(1);
}
/*******************************************************/
