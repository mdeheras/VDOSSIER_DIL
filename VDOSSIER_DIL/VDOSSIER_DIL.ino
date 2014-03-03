#include "DIL.h"
#include <SoftwareSerial.h>
#include <Wire.h>

DIL DIL;
void setup(){
  DIL.begin();
  
}
void loop(){
    DIL.checkOSC();
    DIL.checkButton();
    DIL.refreshADXL();
    DIL.checkIR();
////  if (DIL.availableIR()) Serial.println(DIL.readIR(), HEX);
////  Serial.println(DIL.readBattery());
////  delay(1000);
//    DIL.writeDisplay(DIL.readEncoder());
  
//  {
//    for (int i=0; i<20; i++){ delay(100); while (!DIL.ini_mic()); DIL.volpos_mic();};
//    for (int i=0; i<20; i++){ delay(100); while (!DIL.ini_mic()); DIL.volneg_mic();};
//  }
}
