#include "DIL.h"

#include "DynamixelSerial1.h"
#include "Adafruit_NeoPixel.h"
#include "IRremote.h"

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

void setDisplay(OSCMessage *_mes){
  uint8_t Value=(byte)(_mes->getArgInt32(0));
  if (Value>15) Value = 15;
    for (int i = 0; i<14; i++)
    {
      digitalWrite(dis[i], !code[Value][i]);
    }  
}

void setMic(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  uint8_t Value_Send1 = (uint8_t)(Value>>8);
  uint8_t Value_Send2 = (uint8_t)(Value);
  mic.write(Value_Send1);
  mic.write(Value_Send2);
  mic.write((byte)0x80);
  mic.write((byte)0x00); 
} 

void setDYNAMIXEL(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  uint8_t ID = (uint8_t)(Value>>20);
  uint8_t Position = (uint8_t)(Value>>10);
  uint8_t Speed = (uint8_t)(Value);
  Dynamixel.moveSpeed(ID, Position, Speed);  
  //ID – numero de identificación del servomotor
  //Position – posición del servo de 0 a 1023 (0 a 300 grados)
  //Speed – velocidad a la que se moverá el servo 0 a 1023
} 

void setLanc(OSCMessage *_mes){
  uint16_t Value_Send=_mes->getArgInt32(0);
  Serial3.println(Value_Send, HEX);
  Serial3.println();
  Serial3.println();
} 

void ledRGB(byte led, byte red, byte green, byte blue)
  {  
     strip.setPixelColor(led, red, green, blue);
     strip.show();  //Visualiza leds RGB
  }
  
void setLed0(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  ledRGB(0, Value>>16, Value>>8, Value);
} 

void setLed1(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  ledRGB(1, Value>>16, Value>>8, Value);
} 
void setLed2(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  ledRGB(2, Value>>16, Value>>8, Value);
} 
void setLed3(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  ledRGB(3, Value>>16, Value>>8, Value);
} 
void setLed4(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  ledRGB(4, Value>>16, Value>>8, Value);
} 
void setLed5(OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  ledRGB(5, Value>>16, Value>>8, Value);
} 

void DIL::begin()
  {
    Serial.begin(9600); //USB inicializado a 9600
    Serial2.begin(9600); //WIFI inicializado a 9600
    Serial3.begin(9600); //LANC inicializado a 9600
    mic.begin(2400);  //Control del microfono inicializado a 2400
    Serial.println("Puertos serie inicializados.");
    
    //Inicializacion pulsadores
    for (int i = 64; i<=69; i++)
    {
      pinMode(i, INPUT);
      digitalWrite(i, HIGH);
    }
    Serial.println("Pulsadores inicializados.");
    
    pinMode(PIN_POWER_MIC, INPUT);
    digitalWrite(PIN_POWER_MIC, LOW);
    Serial.println("Puerto del microfono en espera.");
    
    //Inicializacion pines del encoder
    for (int i = 0; i<4; i++)
    {
      pinMode(enc[i], INPUT);
      digitalWrite(enc[i], HIGH);
    }
    Serial.println("Encoder inicializado.");
    
    //Inicializacion pines del display
    for (int i = 0; i<15; i++)
    {
      pinMode(dis[i], OUTPUT);
      digitalWrite(dis[i], HIGH);
    }
    digitalWrite(dis[14], LOW); //Indicador de encendido
    Serial.println("Display inicializado.");
    
    Wire.begin();       //Inicializo bus I2C
    Serial.println("Bus I2c inicializado.");
    
    strip.begin(); //Inicializacion de leds RGB
    strip.show();  //Visualiza leds RGB
    Serial.println("Leds inicializados.");
    
    Dynamixel.begin(1000000,PIN_DYNAMIXEL);  // Inicializa el servo a 1Mbps el Serial1 y con el Pin Control 4
    Serial.println("Motores inicializados.");
    
    irrecv.enableIRIn(); // Start the receiver IR
    Serial.println("Infrarrojo inicializado.");
    
    writeADXL(0x2D, 0x08);
    //  writeADXL(0x31, 0x00); //2g
    //  writeADXL(0x31, 0x01); //4g
    writeADXL(0x31, 0x02); //8g
    //  writeADXL(0x31, 0x03); //16g
    Serial.println("Acelerometro inicializado.");
   
    
    wifly.setupForUDP<HardwareSerial>(
      &Serial2,   //the serial you want to use (this can also be a software serial)
      115200, // if you use a hardware serial, I would recommend the full 115200
      true,	// should we try some other baudrates if the currently selected one fails?
      mySSID,  //Your Wifi Name (SSID)
      myPassword, //Your Wifi Password 
      "DI&L",                 // Device name for identification in the network
      0,         // IP Adress of the Wifly. if 0 (without quotes), it will use dhcp to get an ip
      localPort,                    // WiFly receive port
      IP,       // Where to send outgoing Osc messages. "255.255.255.255" will send to all hosts in the subnet
      outPort,                     // outgoing port
      false	// show debug information on Serial
    ); 
    wifly.printStatusInfo(); //print some debug information 
    
    static char STRING_DISPLAY[14] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'D', 'I', 'S', 'P', 'L', 'A' , 'Y' , 0x00
          };
    server.addCallback(STRING_DISPLAY,&setDisplay);
    
    static char STRING_MIC[13] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'M', 'I', 'C', 'C', 'M', 'D' , 0x00
          };
    server.addCallback(STRING_MIC,&setMic);
        
    static char STRING_LANC[14] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'L', 'A', 'N', 'C', 'C', 'M' , 'D', 0x00
          };
    server.addCallback(STRING_LANC,&setLanc);
    
    static char STRING_LED0[11] = {
            '/', 'D', 'I', 'L', readEncoder(), '/',
            'L', 'E', 'D', '0', 0x00
          };
    server.addCallback(STRING_LED0,&setLed0);
    
    static char STRING_LED1[11] = {
            '/', 'D', 'I', 'L', readEncoder() , '/',
            'L', 'E', 'D', '1', 0x00
          };
    server.addCallback(STRING_LED1,&setLed1);
    
    static char STRING_LED2[11] = {
            '/', 'D', 'I', 'L', readEncoder() , '/',
            'L', 'E', 'D', '2', 0x00
          };
    server.addCallback(STRING_LED2,&setLed2);
    
    static char STRING_LED3[11] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'L', 'E', 'D', '3', 0x00
          };
    server.addCallback(STRING_LED3,&setLed3);
    
    static char STRING_LED4[11] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/', 
            'L', 'E', 'D', '4', 0x00
          };
    server.addCallback(STRING_LED4,&setLed4);
    
    static char STRING_LED5[11] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'L', 'E', 'D', '5', 0x00
          };
    server.addCallback(STRING_LED5,&setLed5);
    
    static char STRING_DYNAMIXEL[11] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'D', 'Y', 'N', 'A', 0x00
          };
          
    server.addCallback(STRING_DYNAMIXEL,&setDYNAMIXEL);
    
    delay(1000);
  }
  
void DIL::writeDisplay(byte character)
{
  if (character>15) character = 15;
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

char DIL::readEncoder()
  {
    char value = 0;
    for (int i = 0; i<4; i++)
    {
      value = value + !digitalRead(enc[i])*encval[i];
    }
    if (value<10) value=value+48;
    else value=value+55;
    return value;
  }

byte DIL::readBattery()
  {
   return(map(VCC/1023.*analogRead(PIN_BAT), 3000, 4200, 0, 100)); 
  }

void DIL::checkOSC()
  {
    server.availableCheck(2);
  }
  
boolean state[6] = {0,0,0,0,0,0};

void DIL::checkButton()
  {
    for (int i = 0; i<6; i++)
    {
       if ((readButton(i))!=(state[i])) 
       {
         char STRING_BUTTON[13] = { // Message template
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'B', 'U', 'T', 'T', 'O', 'N' , B0 
          };
         state[i] = readButton(i);
         client.sendInt((i<<4)|state[i], STRING_BUTTON);
       }
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

int accel_ant[3] ={0,0,0};
int accel[3] ={0,0,0};

void DIL::refreshADXL()
  {
    #define lim 512
    #define RES 5
    int temp[3] ={0,0,0};
    int lecturas=10;
    byte buffADXL[6] ;    //6 bytes buffer for saving data read from the device
    for (int i=0; i<3; i++) accel[i] = 0;
    for(int i=0; i<lecturas; i++)
    {
      readADXL(0x32, 6, buffADXL); //read the acceleration data from the ADXL345
      temp[0] = (((int)buffADXL[1]) << 8) | buffADXL[0]; 
      temp[0] = map(temp[0],-lim,lim,0,1023);  
      temp[1]= (((int)buffADXL[3])<< 8) | buffADXL[2];
      temp[1] = map(temp[1],-lim,lim,0,1023); 
      temp[2] = (((int)buffADXL[5]) << 8) | buffADXL[4];
      temp[2] = map(temp[2],-lim,lim,0,1023); 
      for (int j=0; j<3; j++) accel[j] = (int)(temp[j] + accel[j]);
    }
    for (int i=0; i<3; i++) 
    {
      accel[i] = (int)(accel[i] / lecturas);
      if ((accel[i]>=accel_ant[i] + RES)||(accel[i]<=accel_ant[i] - RES)) 
         {
           char STRING_ACCEL[13] = { // Message template
              '/', 'D', 'I', 'L' , readEncoder() , '/',
              'A', 'C', 'C', 'E', 'L', i + 'X' , B0
            };
           accel_ant[i] = accel[i];
           client.sendInt(accel[i], STRING_ACCEL);
         }
    }
  }
  
void DIL::checkIR()
{
  if (irrecv.decode(&results))
   {
     char STRING_IR[10] = { // Message template
              '/', 'D', 'I', 'L' , readEncoder() , '/',
              'I', 'R', B0 , B0
      };
     if (results.value!=0xFFFFFFFF)
      {
        client.sendInt(results.value, STRING_IR);
//        Serial.println(results.value, HEX);
      }
      irrecv.resume(); // Receive the next value
   }
}

boolean connect_mic = false;

boolean DIL::checkMic()
{ 
    int cont=50;
    if ((digitalRead(PIN_POWER_MIC))&&(!connect_mic))
    {  
        while(cont>0) //Repetimos el proceso 50 veces para asegurarnos que la sesion se inicia
         {
          mic.write(byte(0x00)); //El mando mandas 0s hasta que la gravadora responde con 0x80
          //delay(100);
          if (mic.available())
          {
           if(mic.read()==0x80)
            {
              mic.write(byte(0xA1));// Sersion iniciada
              connect_mic = true;
              return connect_mic;
            }
           else 
           {
             connect_mic = false;
           }
          }
         cont--;
         }
    }
    else if(!digitalRead(PIN_POWER_MIC))
    {
      connect_mic = false;
    }
    return connect_mic;
}
