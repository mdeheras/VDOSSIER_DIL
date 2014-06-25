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
    DIL.checkMic();    
}
