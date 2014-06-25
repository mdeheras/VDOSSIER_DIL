const char mySSID[] = "DI&L";  
const char myPassword[] = "vdossier";
const char *IP = "172.26.255.255";

const uint16_t outPort = 8000;
const uint16_t localPort = 9000;   

#define PIN_WS2812 43
#define PIN_DYNAMIXEL 4
#define PIN_BAT A0
#define PIN_IR 3
#define PIN_POWER_MIC 40

int bot[4] = {8, 13, 12, 9}; //Pines del encoder
int enc[4] = {8, 13, 12, 9}; //Pines del encoder
int dis[15] = {32, 34, 37, 30, 28, 26, 27, 35, 25, 24, 33, 38, 31, 29, 36}; //Pines del encoder

//Visualizacion del Display
                       //A, B, C, D, E, F, G1,G2,H, J, K, L, M, N
uint8_t code[17][14] = {{1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1},//0
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
                        
#define NUM_LEDS 6
#define VCC 5000
int encval[4] = {1, 2, 4, 8};

#define ADXL345 0x53    //ADXL345 device address
