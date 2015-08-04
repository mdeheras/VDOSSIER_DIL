#include "pitches.h"

//const char mySSID[] = "DI&L";  
//const char myPassword[] = "vdossier";
//const char *IP = "192.168.1.255";

const char mySSID[] = "hangar_lab";  
const char myPassword[] = "labinteractius";
const char *IP = "172.26.255.255";

const uint16_t outPort = 8000;
const uint16_t localPort = 9000;   

#define PIN_WS2812 43
#define PIN_DYNAMIXEL 4
#define PIN_BAT A0
#define PIN_VOL A1
#define PIN_IR 3
#define PIN_POWER_MIC 40
#define PIN_AUD 42

#define LANC_X_PIN 15         // Receive Status from LANC Device (unimplemented)
#define bitMicroSeconds 103 //  // 102 bad sony / good cannon, 103, good (canon & sony), 104 (spec) good sony bad cannon
#define MAX_WAIT_MILLISEC 20 // 17 measures from sony

#define TIMER_ENABLE_INTR    (TIMSK3 = _BV(OCIE3A))
#define TIMER_DISABLE_INTR   (TIMSK3 = 0)
#define TWI_FREQ 400000L //Frecuencia bus I2C

int enc[4] = {8, 13, 12, 9}; //Pines del encoder
int dis[15] = {32, 34, 37, 30, 28, 26, 27, 35, 25, 24, 33, 38, 31, 29, 36}; //Pines del display

//Visualizacion del Display
                       //A, B, C, D, E, F, G1,G2,H, J, K, L, M, N
uint8_t codeDisplay[17][14] = 
                       {{1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1},//0
                        {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//1
                        {1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0},//2
                        {1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},//3
                        {0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},//4
                        {1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},//5
                        {1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},//6
                        {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//7
                        {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},//8
                        {1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},//9
                        {1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},//A
                        {1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0},//B
                        {1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},//C
                        {1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0},//D
                        {1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},//E
                        {1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},//F
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};//NULL

const uint16_t melody[89] = {NOTE_B0, NOTE_C1, NOTE_CS1, NOTE_D1, NOTE_DS1, NOTE_E1, NOTE_F1, NOTE_FS1, NOTE_G1,
                             NOTE_GS1, NOTE_A1, NOTE_AS1, NOTE_B1, NOTE_C2, NOTE_CS2, NOTE_D2, NOTE_DS2, NOTE_E2,
                             NOTE_F2, NOTE_FS2, NOTE_G2, NOTE_GS2, NOTE_A2, NOTE_AS2, NOTE_B2, NOTE_C3, NOTE_CS3,
                             NOTE_D3, NOTE_DS3, NOTE_E3, NOTE_F3, NOTE_FS3, NOTE_G3, NOTE_GS3, NOTE_A3, NOTE_AS3,
                             NOTE_B3, NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4,
                             NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4, NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5,
                             NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5, NOTE_A5, NOTE_AS5, NOTE_B5, NOTE_C6, NOTE_CS6,
                             NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6, NOTE_A6, NOTE_AS6,
                             NOTE_B6, NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_F7, NOTE_FS7, NOTE_G7,
                             NOTE_GS7, NOTE_A7, NOTE_AS7, NOTE_B7, NOTE_C8, NOTE_CS8, NOTE_D8, NOTE_DS8};
                            
#define NUM_LEDS 6
#define VCC 5000
#define P1  100   //Kohm 
#define RES_MCP 256

int encval[4] = {1, 2, 4, 8};

#define ADXL345 0x53    //ADXL345 device address
#define MCP1               0x2E    // Direction of the mcp1 audio out
#define MCP2               0x2F    // Direction of the mcp2 amplificador
