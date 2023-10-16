// Defines Relys
#define rel1 PD1
#define rel2 PD2
#define relo2 PD0
#define relo1 PB15
// Define Errors pin
#define LEDerror PB12
#define MCUbuzz PB14
// Mode and Uart
#define Modes1 PA7
#define Modes2 PA8
#define S1tx PA9
#define S1rx PA10
// Power Line control
#define Line1 PB0
#define Line2 PB1
#define Line3 PB2
#define Line4 PB3
#define Line5 PB4
#define Line6 PB5
#define Line7 PB6
#define Line8 PB7
#define Line9 PB8
#define Line10 PB9
#define Line11 PB10
#define Line12 PB11
// Buttons
#define But1 PA11
#define But2 PA12
#define But3 PA13
#define But4 PD3
#define But5 PA15
// Analog Selection
#define Sela PA4
#define Selb PA5
#define Selc PA6
// Analog channels
#define Analog1 PA0
#define Analog2 PA1
#define Analog3 PA2
#define Analog4 PA3
// Battery Charges
#define Batcharges PC15
#define ChangeVolt PC14
// Card Selector
#define CS1 PA7
#define CS2 PA8
// Led pin number
#define ledf1 8
#define lede1 9
#define ledf2 10
#define lede2 11
#define ledf3 12
#define lede3 13
#define ledf4 15
#define lede4 14
#define ledf5 16
#define lede5 17
#define ledf6 18
#define lede6 19
#define ledf7 20
#define lede7 21
#define ledf8 23
#define lede8 22
#define ledf9 24
#define lede9 25
#define ledf10 26
#define lede10 27
#define ledf11 28
#define lede11 29
#define ledf12 31
#define lede12 30
#define generalfault 0
#define ledeearth 1
#define ledebuz 2
#define ledesounder 3
#define ledepower 35
#define ledebat 36
#define ledemainpower 37
#define ledefiremode 38
#define panelon 39

// Variables
float opentreshold = 0.09;
float normaltreshold = 0.24;
float firetreshold = 1.1;
float sctreshold = 0.4;
float lowerthresholdout = 0.1;
float uperthresholdout = 0.49;
bool batcheking = false;
bool card1 = false;
bool card2 = false;
bool relycontroll = false;
bool faultflag = false;
bool fireflag = false;
bool batlowvolt = false;
byte cardpresenterror = 0;
float vpo = 1;
float mux1[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux2[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux3[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux4[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float lcurrent[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float lvoltage[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte limittimesc = 3;
byte muxpos = 0;
byte cardsit = 0;
const byte linecontrol[12] = { PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11 };
const byte lederrors[12] = { 9, 11, 13, 14, 17, 19, 21, 22, 25, 27, 29, 30 };
const byte ledfire[12] = { 8, 10, 12, 15, 16, 18, 20, 23, 24, 26, 28, 31 };
byte linesituation[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };      // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
byte lastlinesituation[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
byte firstsence[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte scdetected[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long time1 = 0;
unsigned long ledblink = 0;
unsigned long buttontime = 0;
unsigned long sctime = 0;
unsigned long buzzready = 0;
unsigned long batscan = 0;
unsigned long fsencetimer = 0;
unsigned long fcounter = 10;
bool blinkerl = true;
bool blinkerl2 = true;
bool buzcont = false;
bool sounderled = false;
bool supplyfault = false;
bool batfail = false;
bool powerfail = false;
bool eartfail = false;
bool genfault = false;
bool firetrac = false;
bool freadanalogs = false;
bool buzzerz = false;
bool beeper = false;
bool relo = false;
bool relycustomon = false;
bool batchargesflag = false;
bool relysit = false;
bool relycharge = false;



