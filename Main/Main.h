#ifndef MAIN_H
#define  MAIN_H

#include <IWatchdog.h>
#include <ShiftRegister74HC595.h>
#include <SoftwareSerial.h>
#include <Arduino.h>




#define MAXIMUM_TIME_FIER_DETECT_FOR_MAIN_LINES 38 
#define MINIMUM_REPEA_FIER_DETECT_FOR_MAIN_LINES 8
#define LIMIT_REPEAT_FOR_FIER_DETECT_MAIN_LINES 9

#define MAXIMUM_TIME_FIER_DETECT_FOR_EXTERA_LINES 30
#define MINIMUM_REPEAT_FIER_DETECT_FOR_EXTERA_LINES 1
#define LIMIT_REPEAT_FOR_FIER_DETECT_EXTERA_LINES 2

int MINIMUM_REPEAT_FIER_DETECT = MINIMUM_REPEA_FIER_DETECT_FOR_MAIN_LINES;
int LIMIT_REPEAT_FOR_FIER_DETECT = LIMIT_REPEAT_FOR_FIER_DETECT_MAIN_LINES;
int MAXIMUM_TIME_FIER_DETECT= MAXIMUM_TIME_FIER_DETECT_FOR_MAIN_LINES;


// #define FIER_DEBUG
// #define SHORT_CIRCUIT_DEBUG
// #define OPEN_CIRCUIT_DEBUG
#define CHECK_BATTERY_DEBUG

#define lineOFF(numberLine) digitalWrite(lineControlPins[numberLine], LOW);
#define lineON(numberLine)  digitalWrite(lineControlPins[numberLine], HIGH);
// Threshold values
#define OPEN_THRESHOLD  0.09
#define NORMAL_THRESHOLD  0.24
//#define FIRE_THRESHOLD  1.1
#define FIRE_THRESHOLD  0.8
//#define SHORT_CIRCUIT_THRESHOLD  0.4
#define SHORT_CIRCUIT_THRESHOLD  0.95
#define LOWER_THRESHOLD_OUT 0.1
#define UPPER_THRESHOLD_OUT  0.49


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
#define JUMPER PA13
#define But3 PB13
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






// Flags
bool batteryChecking = false;
bool card1Present = false;
bool card2Present = false;
bool relayControl = false;
bool faultFlag = false;
bool fireFlag = false;
bool batteryLowVoltage = false;
bool supplyFault = false;
bool batteryFail = false;
bool powerFail = false;
bool earthFail = false;
bool generalFault = false;
bool fireTrace = false;
bool readAnalogs = false;
bool buzzerEnabled = false;
bool beeperEnabled = false;
bool relayOn = false;
bool relayCustomOn = false;
bool relayStatus = false;
bool relayCharging = false;
bool batteryChargesFlag = false;
bool ledBlinker1 = true;
bool ledBlinker2 = true;
bool buzzerControl = false;
bool sounderLedStatus = false;
 int fierLouckBit = 0;

//bool  stateUpdateMUX =false;

// Data arrays
float mux1Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux2Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux3Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux4Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float lineCurrent[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float lineVoltage[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


const char lineControlPins[12] = {Line1, Line2, Line3, Line4, Line5, Line6, Line7, Line8, Line9, Line10, Line11, Line12 };

typedef enum {
  NON_STATUS,
  OPEN_CIRCUIT,
  NORMAL,
  FIER,
  SHORT_CIRCUIT
} status;
status lineStatus[12] = { NON_STATUS };

#define SUPPLY_VOLTAGE_IS_16_V digitalWrite(ChangeVolt, LOW);
#define SUPPLY_VOLTAGE_IS_24_V digitalWrite(ChangeVolt, HIGH);

#define POWER_RELAY_ON  digitalWrite(Batcharges,HIGH);
#define POWER_RELAY_OFF  digitalWrite(Batcharges,LOW);
 byte limitLowPower =18;
typedef enum {
  STOP,
  START

} STATE;

class timerMS {
  public:
  struct
  {
    STATE status;
    unsigned long value = 0;
  } timer;

  void update();
};
 void timerMS::update() {
    if (timer.status == START) {
        timer.value++;
    } else if (timer.status == STOP) {
        timer.value = 0;
    }
} 



//timer.status = START;
unsigned long currentTime = 0;
unsigned long ledBlinkTime = 0;
unsigned long buttonPressTime = 0;
unsigned long shortCircuitTime = 0;
unsigned long buzzerReadyTime = 0;
unsigned long batteryScanTime = 0;

unsigned long fultSencetimer = 0;
unsigned long fultCounter = 10;

unsigned long learningProcessCounter = 10;

int j = 0;
char i = 0;


char CardPresentError = 0;
char limitTimeSC = 3;
char muxPosition = 0;
char cardSituation = 0;

const char ledErrorsPins[12] = { 9, 11, 13, 14, 17, 19, 21, 22, 25, 27, 29, 30 };
const char ledFirePins[12] = { 8, 10, 12, 15, 16, 18, 20, 23, 24, 26, 28, 31 };

char firstSence[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
char shortCircuitDetected[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


#endif

