//const char mySSID[] = "DI&L";  
//const char myPassword[] = "vdossier";
//const char *IP = "192.168.1.255";

const char mySSID[] = "hangar_lab";  
const char myPassword[] = "labinteractius";
const char *IP = "172.26.20.253";

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

int bot[4] = {8, 13, 12, 9}; //Pines del encoder
int enc[4] = {8, 13, 12, 9}; //Pines del encoder
int dis[15] = {32, 34, 37, 30, 28, 26, 27, 35, 25, 24, 33, 38, 31, 29, 36}; //Pines del encoder

//Visualizacion del Display
                       //A, B, C, D, E, F, G1,G2,H, J, K, L, M, N
uint8_t codeDisplay[17][14] = {{1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1},//0
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

const uint16_t MORSE[28] = {0xB000,0xD500,0x1AD0,0x6DB0,0x3500,0x1000,0xAD00,
                            0x6D00,0x5500,0x5000,0x2DB0,0x6B00,0xB500,0x1B00,
                            0xD000,0x1B5B,0xDB00,0x16D0,0x36B0,0x2D00,0x1500,
                            0x3000,0x2B00,0xAB00,0x5B00,0x1AB0,0x35B0,0x1B50};
                            
#define NUM_LEDS 6
#define VCC 5000
#define P1  100   //Kohm 
#define RES_MCP 256

int encval[4] = {1, 2, 4, 8};

#define ADXL345 0x53    //ADXL345 device address
#define MCP1               0x2E    // Direction of the mcp1 audio out
#define MCP2               0x2F    // Direction of the mcp2 amplificador
