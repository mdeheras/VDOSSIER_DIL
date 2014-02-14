#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>

class DIL {
  public:  
    void begin();
    void ledRGB(byte led, byte red, byte green, byte blue);
    void writeDisplay(byte character);
    boolean readButton(byte button);
    byte readEncoder();
    byte readBattery();
    void checkButton();
    void refreshADXL();
    int getAxisX(); 
    int getAxisY(); 
    int getAxisZ(); 
    boolean availableIR();
    unsigned long readIR();
//    void checkButtons();
//    void refresh();
//    void startup();
//    void checkADC();
  private:
    void writeADXL(byte address, byte val);
    void readADXL(byte address, int num, byte buff[]);
//    void on_press(byte r, byte c);
//    void on_release(byte r, byte c);
//    void timer1Initialize();
//    void timer3Initialize();
};
