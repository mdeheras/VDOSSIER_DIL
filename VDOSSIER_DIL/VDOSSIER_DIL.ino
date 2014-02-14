#include "DIL.h"
#include <SoftwareSerial.h>
#include <Wire.h>

DIL DIL;
void setup(){
  DIL.begin();
}
void loop(){
    DIL.checkButton();
//  DIL.readEncoder();
//  DIL.refreshADXL();
//    Serial.print(DIL.getAxisX());
//    Serial.print(" ");
//    Serial.print(DIL.getAxisY());
//    Serial.print(" ");
//    Serial.println(DIL.getAxisZ());
//  if (DIL.availableIR()) Serial.println(DIL.readIR(), HEX);
//  Serial.println(DIL.readBattery());
//  delay(1000);
    DIL.writeDisplay(DIL.readEncoder());

  
}
