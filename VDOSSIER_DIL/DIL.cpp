#include "DIL.h"

#include "DynamixelSerial1.h"
#include "Adafruit_NeoPixel.h"
#include "IRremote.h"
#include "WiFlyHQ.h"
#include "ArdOSCForWiFlyHQ.h"
#include "Constants.h"


Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN_WS2812, NEO_GRB + NEO_KHZ800);
SoftwareSerial mic(10, 11); // RX, TX

// inicializa la libreria de recepcion y envio de datos por el infrarrojo
IRrecv irrecv(PIN_IR); //Solo para el pin digital 3!!!
decode_results results;
IRsend irsend; //Solo para el pin digital 9!!!  

//Configuracion del wifi
WiFly wifly;
OSCClient client(&wifly);
OSCServer server(&wifly);

int accel_x=0;
int accel_y=0;
int accel_z=0;
  
void DIL::begin()
  {
    //Inicializacion pulsadores
    for (int i = 64; i<=69; i++)
    {
      pinMode(i, INPUT);
      digitalWrite(i, HIGH);
    }
    
    //Inicializacion pines del encoder
    for (int i = 0; i<4; i++)
    {
      pinMode(enc[i], INPUT);
      digitalWrite(enc[i], HIGH);
    }
    
    //Inicializacion pines del display
    for (int i = 0; i<15; i++)
    {
      pinMode(dis[i], OUTPUT);
      digitalWrite(dis[i], HIGH);
    }
    digitalWrite(dis[14], LOW); //Indicador de encendido
    
    Wire.begin();       //Inicializo bus I2C
    Serial.begin(9600); //USB inicializado a 9600
    Serial2.begin(115200); //WIFI inicializado a 9600
    Serial3.begin(9600); //LANC inicializado a 9600
    mic.begin(2400);  //Control del microfono inicializado a 2400
    
    strip.begin(); //Inicializacion de leds RGB
    strip.show();  //Visualiza leds RGB
    Dynamixel.begin(1000000,PIN_DYNAMIXEL);  // Inicializa el servo a 1Mbps en el Pin Control 4
    irrecv.enableIRIn(); // Start the receiver IR
    writeADXL(0x2D, 0x08);
    //  writeADXL(0x31, 0x00); //2g
    //  writeADXL(0x31, 0x01); //4g
    writeADXL(0x31, 0x02); //8g
    //  writeADXL(0x31, 0x03); //16g
    
   
    
//    wifly.setupForUDP<HardwareSerial>(
//      &Serial2,   //the serial you want to use (this can also be a software serial)
//      115200, // if you use a hardware serial, I would recommend the full 115200
//      true,	// should we try some other baudrates if the currently selected one fails?
//      mySSID,  //Your Wifi Name (SSID)
//      myPassword, //Your Wifi Password 
//      "DI&L",                 // Device name for identification in the network
//      0,         // IP Adress of the Wifly. if 0 (without quotes), it will use dhcp to get an ip
//      localPort,                    // WiFly receive port
//      IP,       // Where to send outgoing Osc messages. "255.255.255.255" will send to all hosts in the subnet
//      outPort,                     // outgoing port
//      false	// show debug information on Serial
//    ); 
//    wifly.printStatusInfo(); //print some debug information 


//    server.addCallback("/m1",&setMotor1);
//    server.addCallback("/m2",&setMotor2);
//    server.addCallback("/m3",&setMotor3);
//    server.addCallback("/v1",&setVel1);
//    server.addCallback("/v2",&setVel2);
//    server.addCallback("/v3",&setVel3);
//    server.addCallback("/r", &sReset);
//    server.addCallback("/s",&setStop);
//    server.addCallback("/t",&test);
    delay(1000);
  }
  
void DIL::ledRGB(byte led, byte red, byte green, byte blue)
  {  
     strip.setPixelColor(led, red, green, blue);
     strip.show();  //Visualiza leds RGB
  }
  
void DIL::writeDisplay(byte character)
  {  
    if (character>15) character = 16;
    for (int i = 0; i<14; i++)
    {
      digitalWrite(dis[i], !code[character][i]);
    }         
  }
  
boolean DIL::readButton(byte button)
  {
    if (button<6) return !digitalRead(69 - button);
    else return false;
  }

byte DIL::readEncoder()
  {
    int value = 0;
    for (int i = 0; i<4; i++)
    {
      value = value + !digitalRead(enc[i])*encval[i];
    }
    return value;
  }

byte DIL::readBattery()
  {
   return(map(VCC/1023.*analogRead(PIN_BAT), 3000, 4200, 0, 100)); 
  }

void DIL::checkButton()
  {
    for (int i = 0; i<6; i++)
    {
     if (readButton(i)) ledRGB(i, 0, 0, 255);
     else ledRGB(i, 0, 0, 0);
    }
  }
  
void DIL::writeADXL(byte address, byte val)
  {
   Wire.beginTransmission(ADXL345); //start transmission to device 
   Wire.write(address);        // write register address
   Wire.write(val);        // write value to write
   Wire.endTransmission(); //end transmission
  }

//reads num bytes starting from address register on device in to buff array
void DIL::readADXL(byte address, int num, byte buff[]) 
  {
    Wire.beginTransmission(ADXL345); //start transmission to device 
    Wire.write(address);        //writes address to read from
    Wire.endTransmission(); //end transmission
    
    Wire.beginTransmission(ADXL345); //start transmission to device
    Wire.requestFrom(ADXL345, num);    // request 6 bytes from device
    
    int i = 0;
    unsigned long time = millis();
    while (!Wire.available()) 
    {
      if ((millis() - time)>500) 
      {
        for(int i=0; i<num; i++) buff[i]=0x00;
        break;
      }
    }
    while(Wire.available())    //device may write less than requested (abnormal)
    { 
      buff[i] = Wire.read(); // read a byte
      i++;
    }
    Wire.endTransmission(); //end transmission
  }

void DIL::refreshADXL()
  {
    #define lim 512
    int temp_x=0;
    int temp_y=0;
    int temp_z=0;
    int lecturas=10;
    byte buffADXL[6] ;    //6 bytes buffer for saving data read from the device
    accel_x=0;
    accel_y=0;
    accel_z=0;
    
    for(int i=0; i<lecturas; i++)
    {
      readADXL(0x32, 6, buffADXL); //read the acceleration data from the ADXL345
      temp_x = (((int)buffADXL[1]) << 8) | buffADXL[0]; 
      temp_x = map(temp_x,-lim,lim,0,1023);  
      temp_y = (((int)buffADXL[3])<< 8) | buffADXL[2];
      temp_y = map(temp_y,-lim,lim,0,1023); 
      temp_z = (((int)buffADXL[5]) << 8) | buffADXL[4];
      temp_z = map(temp_z,-lim,lim,0,1023); 
      accel_x = (int)(temp_x + accel_x);
      accel_y = (int)(temp_y + accel_y);
      accel_z = (int)(temp_z + accel_z);
    }
    accel_x = (int)(accel_x / lecturas);
    accel_y = (int)(accel_y / lecturas);
    accel_z = (int)(accel_z / lecturas);
  }

int DIL::getAxisX()
  {
    return accel_x;
  }

int DIL::getAxisY()
  {
    return accel_y;
  }

int DIL::getAxisZ()
  {
    return accel_z;
  }
  
boolean DIL::availableIR()
{
  return (irrecv.decode(&results));
}

unsigned long DIL::readIR()
{
        irrecv.resume(); // Receive the next value
        return(results.value);
}
