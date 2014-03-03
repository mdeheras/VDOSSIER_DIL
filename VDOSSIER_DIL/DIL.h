#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "WiFlyHQ.h"
#include "ArdOSCForWiFlyHQ.h"


class DIL {
  public:  
    void begin();
    void ledRGB(byte led, byte red, byte green, byte blue);
    void writeDisplay(byte character);
    boolean readButton(byte button);
    char readEncoder();
    byte readBattery();
    void checkButton();
    void checkIR();
    boolean ini_mic();
    void volpos_mic();
    void volneg_mic();
    void refreshADXL();
    void checkOSC();
//    void checkButtons();
//    void refresh();
//    void startup();
//    void checkADC();
  private:
    void writeADXL(byte address, byte val);
    void readADXL(byte address, int num, byte buff[]);
};
