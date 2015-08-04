#include "DIL.h"

#include "DynamixelSerial1.h"
#include "Adafruit_NeoPixel.h"
#include "IRremote.h"


#include "Constants.h"

#define WIFI_ENABLE true
#define ENCODER_ENABLE false
#define VALUE_ENCODER '0'

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN_WS2812, NEO_GRB + NEO_KHZ800);
SoftwareSerial mic(10, 11); // RX, TX

// inicializa la libreria de recepcion y envio de datos por el infrarrojo
IRrecv irrecv(PIN_IR); //Solo para el pin digital 3!!!
decode_results results;
IRsend irsend; //Solo para el pin digital 5!!!  
DIL DIL_;
//Configuracion del wifi
WiFly wifly;
OSCClient client(&wifly);
OSCServer server(&wifly);

void setDisplay(OSCMessage *_mes){
  uint8_t Value=(byte)(_mes->getArgInt32(0));
  if (Value>15) Value = 15;
    for (int i = 0; i<14; i++)
    {
      digitalWrite(dis[i], !codeDisplay[Value][i]);
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

void setIRSend (OSCMessage *_mes){
  uint32_t Value=_mes->getArgInt32(0);
  uint32_t Value1=_mes->getArgInt32(1);
  irsend.sendNEC((Value<<16)+Value1, 32);
} 

void setCode (OSCMessage *_mes){
  uint32_t code=_mes->getArgInt32(0);
  uint32_t vol=_mes->getArgInt32(1);
  if (vol>RES_MCP) vol = RES_MCP;
  DIL_.writeMCP(MCP1, 0x00, vol);
  tone(PIN_AUD, melody[code], 100);
} 

void setDYNAMIXEL(OSCMessage *_mes){
  uint32_t ID=_mes->getArgInt32(0);
  uint32_t Position=_mes->getArgInt32(1);
  uint32_t Speed=_mes->getArgInt32(2);
  Dynamixel.moveSpeed(ID, Position, Speed);  
  //ID – numero de identificación del servomotor
  //Position – posición del servo de 0 a 1023 (0 a 300 grados)
  //Speed – velocidad a la que se moverá el servo 0 a 1023
} 

void lowWait(int pin) {
  byte in;
  unsigned long start = millis();
  do {
    in = digitalRead(pin);
  } 
  while (in && millis() < start + MAX_WAIT_MILLISEC );
}

void frameStartBitWait() {
  // finds the start of a telegram/frame
  unsigned long usec = 0;
  unsigned long start = millis();
  do {
    usec = pulseIn(LANC_X_PIN,HIGH);
  } 
  while (usec < 1450  && millis() < start + MAX_WAIT_MILLISEC); //  Frame Lengths are 1200-1400 dep. on device
}

void writeByte(int pin, byte value, unsigned uSec /* bit width */) {
  delayMicroseconds(uSec); // wait for stop bit
  pinMode(LANC_X_PIN,OUTPUT);
  for (int i = 0; i < 8; i++) {
    boolean bit = value & 0x1;
    digitalWrite(pin,!bit); // NOT (!) pin because all data is inverted in LANC
    value >>= 1;
    delayMicroseconds(uSec);
  }

  //digitalWrite(pin,HIGH);
  //delayMicroseconds(uSec);

  pinMode(LANC_X_PIN,INPUT);    
}

void SendCode(byte type,byte code) {
  /* Okay i really don't know why i have to send this twice biut it works:
     Sonys need 2 cannons 3
  */
    frameStartBitWait();
    writeByte(LANC_X_PIN,type,bitMicroSeconds);
    lowWait(LANC_X_PIN);
    writeByte(LANC_X_PIN,code,bitMicroSeconds);
    
    frameStartBitWait();
    writeByte(LANC_X_PIN,type,bitMicroSeconds);
    lowWait(LANC_X_PIN);
    writeByte(LANC_X_PIN,code,bitMicroSeconds);
    
    frameStartBitWait();
    writeByte(LANC_X_PIN,type,bitMicroSeconds);
    lowWait(LANC_X_PIN);
    writeByte(LANC_X_PIN,code,bitMicroSeconds);
}

byte readByte(int pin,unsigned long uSec /* bit width*/ ) {
  byte result = 0;
  delayMicroseconds(uSec * 1.5); // skips the Start Bit and Land in the midlle of the first byte

  for (int i = 0; i < 8; i++) {
    if (digitalRead(pin) == LOW) { // == *LOW* because bits inverted in LANC 
      result++;
    }
    result <<= 1;
    delayMicroseconds(uSec);
  }
  delayMicroseconds(0.5*uSec);  
  return result; // return happens at end of last (8ths) bit
}

void setLanc(OSCMessage *_mes){
  TIMER_DISABLE_INTR;
  uint16_t Value_Send=_mes->getArgInt32(0);
  uint16_t repeat =_mes->getArgInt32(1);
  
  for (int i=0; i<abs(repeat); i++)
    {
      SendCode(Value_Send>>8, Value_Send&0x00FF);
      delay(10);
    }
  TIMER_ENABLE_INTR;
} 

void ledRGB(byte led, byte red, byte green, byte blue)
  {  
     strip.setPixelColor(led, red, green, blue);
     strip.show();  //Visualiza leds RGB
  }
  
void setLed(OSCMessage *_mes){
  ledRGB(_mes->getArgInt32(0), _mes->getArgInt32(1), _mes->getArgInt32(2), _mes->getArgInt32(3));
} 

boolean rec_all = false;

byte color_led[6][3] = {{ 0,0,0 },
                        { 0,0,0 },
                        { 0,0,0 },
                        { 0,0,0 },
                        { 0,0,0 },
                        { 0,0,0 }};  
                        
void RECALL(byte led)
  {
    if (!rec_all)
    {
      color_led[led][0] = 0;
      color_led[led][1] = 0;
      color_led[led][2] = 255;
      strip.setPixelColor(led, color_led[led][0], color_led[led][1], color_led[led][2]);
      strip.show();  //Visualiza leds RGB
      TIMER_DISABLE_INTR;
      SendCode(0x18, 0x33);
      delay(10);
      mic.write((byte)0x81);
      mic.write((byte)0x00);
      mic.write((byte)0x80);
      mic.write((byte)0x00); 
      delay(100);
      mic.write((byte)0x81);
      mic.write((byte)0x00);
      mic.write((byte)0x80);
      mic.write((byte)0x00); 
      rec_all = true;  
      TIMER_ENABLE_INTR;
      color_led[led][0] = 255;
      color_led[led][1] = 0;
      color_led[led][2] = 0;
      strip.setPixelColor(led, color_led[led][0], color_led[led][1], color_led[led][2]);
      strip.show();  //Visualiza leds RGB
    }
  }
  
void setREC(OSCMessage *_mes){
    RECALL(0);
} 

void STOPALL(byte led)
  {
    if (rec_all)
    {
      color_led[led][0] = 0;
      color_led[led][1] = 0;
      color_led[led][2] = 255;
      strip.setPixelColor(led, color_led[led][0], color_led[led][1], color_led[led][2]);
      strip.show();  //Visualiza leds RGB
      TIMER_DISABLE_INTR;
      SendCode(0x18, 0x33);
      delay(10);
      mic.write((byte)0x84);
      mic.write((byte)0x00);
      mic.write((byte)0x80);
      mic.write((byte)0x00); 
      rec_all = false;  
      TIMER_ENABLE_INTR;
      for (int i = 0; i<6; i++)
        {
          color_led[i][0] = 0;
          color_led[i][1] = 0;
          color_led[i][2] = 0;
          strip.setPixelColor(i, color_led[i][0], color_led[i][1], color_led[i][2]);
        }
      strip.show();  //Visualiza leds RGB
    }
  }
  
void setSTOP(OSCMessage *_mes){
   STOPALL(0);
} 

void DIL::begin()
  {
    Serial.begin(9600); //USB inicializado a 9600
    Serial2.begin(9600); //WIFI inicializado a 9600
    mic.begin(2400);  //Control del microfono inicializado a 2400
    Serial.println("Puertos serie inicializados.");
    
    //Inicializacion pulsadores
    for (int i = 64; i<=69; i++)
    {
      pinMode(i, INPUT);
      digitalWrite(i, HIGH);
    }
    Serial.println("Pulsadores inicializados.");
    
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
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;  
    Serial.println("Bus I2c inicializado.");
    
    pinMode(PIN_POWER_MIC, INPUT);
    digitalWrite(PIN_POWER_MIC, LOW);
    writeGAIN(10000);
    Serial.println("Puerto del microfono en espera.");
    
    pinMode(PIN_AUD, OUTPUT);
    digitalWrite(PIN_AUD, LOW);
    
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
    
    pinMode(LANC_X_PIN, INPUT);
    Serial.println("LANC inicializado.");
    
    #if WIFI_ENABLE
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
    #endif
    
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
    server.addCallback("/DILX/MICCMD",&setMic);
        
    static char STRING_LANC[14] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'L', 'A', 'N', 'C', 'C', 'M' , 'D', 0x00
          };
    server.addCallback(STRING_LANC,&setLanc);
    server.addCallback("/DILX/LANCCMD",&setLanc);

    server.addCallback("/DILX/RECALL",&setREC);
    server.addCallback("/DILX/STOPALL",&setSTOP);
    
    static char STRING_IRSEND[13] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'I', 'R', 'S', 'E', 'N', 'D', 0x00
          };
    server.addCallback(STRING_IRSEND,&setIRSend);
    
    static char STRING_LED[10] = {
            '/', 'D', 'I', 'L', readEncoder(), '/',
            'L', 'E', 'D', 0x00
          };
          
    server.addCallback(STRING_LED,&setLed);
    server.addCallback("/DILX/LED",&setLed);
        
    static char STRING_DYNAMIXEL[11] = {
            '/', 'D', 'I', 'L' , readEncoder() , '/',
            'D', 'Y', 'N', 'A', 0x00
          };
    server.addCallback(STRING_DYNAMIXEL,&setDYNAMIXEL);
    
//    static char STRING_CODE[11] = {
//            '/', 'D', 'I', 'L' , readEncoder() , '/',
//            'C', 'O', 'D', 'E', 0x00
//          }; 
//    server.addCallback(STRING_CODE,&setCode);

    server.addCallback("/DILX/CODE",&setCode);
   
  }
  
void DIL::writeDisplay(byte character)
{
  if (character>15) character = 15;
    for (int i = 0; i<14; i++)
    {
      digitalWrite(dis[i], !codeDisplay[character][i]);
    }    
}
  
boolean DIL::readButton(byte button)
  {
    if (button<6) return !digitalRead(69 - button);
    else return false;
  }

char DIL::readEncoder()
  {
    #if ENCODER_ENABLE
      char value = 0;
      for (int i = 0; i<4; i++)
      {
        value = value + !digitalRead(enc[i])*encval[i];
      }
      if (value<10) value=value+48;
      else value=value+55;
      return value;
    #else 
      return VALUE_ENCODER;
    #endif
  }

void DIL::checkOSC()
  {
    #if WIFI_ENABLE
      server.availableCheck(2);
    #endif
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
         #if WIFI_ENABLE
          OSCMessage loacal_mes;
          loacal_mes.beginMessage(STRING_BUTTON);
          loacal_mes.addArgInt32(i);
          loacal_mes.addArgInt32(state[i]);
          client.send(&loacal_mes);
          if (state[i])
            {
              strip.setPixelColor(i, 0, 0, 255);
              strip.show();  //Visualiza leds RGB
              if (i==0)
                {
                  if (!rec_all)
                    {
                      delay(100);
                      loacal_mes.beginMessage("/DILX/RECALL");
                      client.send(&loacal_mes);
                      RECALL(i);
                    }
                  else
                    {
                      delay(100);
                      loacal_mes.beginMessage("/DILX/STOPALL");
                      client.send(&loacal_mes);
                      STOPALL(i);
                    }
                }
               else if (i==2)
                {
                  delay(100);
                  loacal_mes.beginMessage("/DILX/CODE");
                  loacal_mes.addArgInt32(88);
                  loacal_mes.addArgInt32(255);
                  client.send(&loacal_mes);
                  DIL_.writeMCP(MCP1, 0x00, 255);
                  tone(PIN_AUD, melody[88], 100);
                }
               else if (i==5)
                {
                  if (!rec_all) RECALL(i);
                  else STOPALL(i);
                }
            }
           else 
             {
                strip.setPixelColor(i, color_led[i][0], color_led[i][1], color_led[i][2]);
                strip.show();  //Visualiza leds RGB
             }
         #endif
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
    #define RES_ADXL 5
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
    boolean send_osc = false;
    for (int i=0; i<3; i++) 
    {
      accel[i] = (int)(accel[i] / lecturas);
      if ((accel[i]>=accel_ant[i] + RES_ADXL)||(accel[i]<=accel_ant[i] - RES_ADXL)) 
         {

           accel_ant[i] = accel[i];
           send_osc = true;
//           client.sendInt(accel[i], STRING_ACCEL);
         }
    }
    if (send_osc)
      {
        char STRING_ACCEL[12] = { // Message template
          '/', 'D', 'I', 'L' , readEncoder() , '/',
          'A', 'C', 'C', 'E', 'L', B0
        };
        #if WIFI_ENABLE
          OSCMessage loacal_mes;
          loacal_mes.beginMessage(STRING_ACCEL);
          loacal_mes.addArgInt32(accel[0]);
          loacal_mes.addArgInt32(accel[1]);
          loacal_mes.addArgInt32(accel[2]);
          client.send(&loacal_mes);
        #endif
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
        #if WIFI_ENABLE
          OSCMessage loacal_mes;
          loacal_mes.beginMessage(STRING_IR);
          loacal_mes.addArgInt32(results.value>>16);
          loacal_mes.addArgInt32(results.value&0x0000FFFF);
          client.send(&loacal_mes);
        #endif
      }
      irrecv.resume(); // Receive the next value
   }
}

int bat_ant = 0;
void DIL::checkBattery()
{
     int bat = analogRead(PIN_BAT)*(VCC/1023.); 
     if ((bat>(bat_ant + 20))||(bat<(bat_ant - 20)))
       {
         char STRING_BAT[10] = { // Message template
                  '/', 'D', 'I', 'L' , readEncoder() , '/',
                  'B', 'A', 'T', B0
          };
          #if WIFI_ENABLE
            client.sendInt(bat, STRING_BAT);
            bat_ant = bat;
          #endif
          Serial.print("Bat: ");
          Serial.println(bat);
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



void DIL::writeMCP(byte deviceaddress, byte address, int data ) {
  if (data>RES_MCP) data=RES_MCP;
  byte cmd_byte=((address<<4)&B11110000)|bitRead(data, 8);
  Wire.beginTransmission(deviceaddress);
  Wire.write(cmd_byte);
  Wire.write(lowByte(data));
  Wire.endTransmission();
  Wire.flush();
  delay(4);
}

uint16_t DIL::readMCP(int deviceaddress, uint16_t address ) {
  byte rdata = 0xFF;
  uint16_t  data = 0x0000;
  byte cmd_byte =(address<<4)|B00001100;
  Wire.beginTransmission(deviceaddress);
  Wire.write(cmd_byte);
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,2);
  Wire.endTransmission();                 // stop transmitting
  unsigned long time = millis();
  while (!Wire.available()) if ((millis() - time)>500) return 0x00;
  rdata = Wire.read(); 
  data=rdata<<8;
  while (!Wire.available()); 
  rdata = Wire.read(); 
  Wire.flush();
  data=data|rdata;
  return data;
}

float kr1= ((float)P1*1000)/RES_MCP;    //  Resistance conversion Constant for the digital pot.

void DIL::writeRGAIN(byte device, long resistor) {
  int data=0x00;
  data = (int)(resistor/kr1);
  writeMCP(MCP2, device, data);
}

float DIL::readRGAIN(byte device)
{
    return (kr1*readMCP(MCP2, device));    // Returns Resistance (Ohms)
}

void DIL::writeGAIN(long value)
{
  if (value == 100)
  {
    writeRGAIN(0x00, 10000);
    writeRGAIN(0x01, 10000);
  }
  else if (value == 1000)
  {
    writeRGAIN(0x00, 10000);
    writeRGAIN(0x01, 100000);
  }
  else if (value == 10000)
        {
           writeRGAIN(0x00, 100000);
           writeRGAIN(0x01, 100000);
        }
  delay(100);
}

float DIL::readGAIN()
{
  return (readRGAIN(0x00)/1000)*(readRGAIN(0x01)/1000);
} 

int vol_ant = 0;
void DIL::checkAUDIO()
{
     int vol = analogRead(PIN_VOL)*(VCC/1023.); 
     if ((vol>(vol_ant + 20))||(vol<(vol_ant - 20)))
       {
         char STRING_AUDIO[12] = { // Message template
                  '/', 'D', 'I', 'L' , readEncoder() , '/',
                  'A', 'U', 'D', 'I', 'O',B0
          };
         #if WIFI_ENABLE
            client.sendInt(vol, STRING_AUDIO);
            vol_ant = vol;
         #endif
       }  
}



