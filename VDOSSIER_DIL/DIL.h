#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "WiFlyHQ.h"
#include "ArdOSCForWiFlyHQ.h"


class DIL {
  public:  
    void begin();
    void writeDisplay(byte character);
    boolean readButton(byte button);
    char readEncoder();
    void checkButton();
    void checkIR();
    boolean checkMic();
    void refreshADXL();
    void checkOSC();
    void checkBattery();
    void checkAUDIO();
    void writeGAIN(long value);
    float readGAIN();
    uint16_t readMCP(int deviceaddress, uint16_t address );
    void writeMCP(byte deviceaddress, byte address, int data );
  private:
    void writeADXL(byte address, byte val);
    void readADXL(byte address, int num, byte buff[]);
    void ledRGB(byte led, byte red, byte green, byte blue);

    float readRGAIN(byte device);
    void writeRGAIN(byte device, long resistor);
};
