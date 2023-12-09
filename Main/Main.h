


#ifndef MAIN_H
#define  MAIN_H

#include <Arduino.h>

#include <IWatchdog.h>
#include <ShiftRegister74HC595.h>
#include <SoftwareSerial.h>











#define DEBUG_ON  mySerial.print
#define DEBUG_OFF 

 //#define POWER_CHECK_DEBUG 
// #define LINE_STATUS_DEBUG  
//  #define LINE_FIER_DEBUG
// // #define LINE_CS_DEBUG






#ifdef  POWER_CHECK_DEBUG 
  #define POWER_CHECK_DEBUG   DEBUG_ON
#else
  #define POWER_CHECK_DEBUG   DEBUG_OFF
#endif


#ifdef  LINE_STATUS_DEBUG
  #define LINE_FIER_DEBUG
  #define LINE_STATUS_DEBUG   DEBUG_ON
#else
  #define LINE_STATUS_DEBUG   DEBUG_OFF 
#endif


#ifdef  LINE_FIER_DEBUG
  #define LINE_FIER_DEBUG   DEBUG_ON
#else
  #define LINE_FIER_DEBUG   DEBUG_OFF 
#endif

#ifdef  LINE_CS_DEBUG 
  #define LINE_CS_DEBUG   DEBUG_ON
#else
  #define LINE_CS_DEBUG   DEBUG_OFF
#endif



#define lineOFF(numberLine) digitalWrite(lineControlPins[numberLine], LOW);
#define lineON(numberLine)  digitalWrite(lineControlPins[numberLine], HIGH);
// Threshold values
#define OPEN_THRESHOLD  9
#define NORMAL_THRESHOLD  24
//#define FIRE_THRESHOLD  1.1
#define FIRE_THRESHOLD 80
//#define SHORT_CIRCUIT_THRESHOLD  0.4
#define SHORT_CIRCUIT_THRESHOLD 95
#define LOWER_THRESHOLD_OUT 10
#define UPPER_THRESHOLD_OUT  49


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
bool fierCheckLock =false;
//bool  stateUpdateMUX =false;

// Data arrays
float mux1Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux2Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux3Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux4Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float lineCurrent[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float lineVoltage[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


float voltage;
float batteryVoltage;
float powerSupplyVoltage;


const char lineControlPins[12] = {Line1, Line2, Line3, Line4, Line5, Line6, Line7, Line8, Line9, Line10, Line11, Line12 };


typedef enum {
  NON_STATUS,
  OPEN_CIRCUIT,
  NORMAL,
  FIER,
  SHORT_CIRCUIT,
  DAMAGED

} status;
status lineStatus[12] = { NON_STATUS };

#define SUPPLY_VOLTAGE_IS_16_V digitalWrite(ChangeVolt, HIGH);
#define SUPPLY_VOLTAGE_IS_24_V digitalWrite(ChangeVolt, LOW);

#define POWER_RELAY_ON   digitalWrite(Batcharges,HIGH);
#define POWER_RELAY_OFF  digitalWrite(Batcharges,LOW);


typedef struct 
{
  float minimumOpenCircuit =0;
  float maximumOpenCircuit =0;

  float minimumNormal=0;
  float maximumNormal=0;

  float minimumFier=0;
  float maximumFier=0;
  
  float minimumCurrentShortCircuit =0;
  float maximumCurrentShortCircuit =0;

  float minimumVoltageShortCircuit =90;
  float maximumVoltageShortCircuit =0;
  
}limit;



typedef enum {
  STOP,
  START,
  PAUSE,

} STATE;



class timerMS {
  public:
  
    STATE status;
    unsigned long value = 0;
    void update();
};
 void timerMS::update() {
    if (status == START) {
         value++;
    } else if (status == STOP) {
        value = 0;
    } else if (status == PAUSE ) {

    }

}

class flowDelay: public timerMS {
  public:

  bool Delay(long unsigned time_ms ){
    
    status= START;

     if (value>= time_ms){
        status= STOP;

       return true;
     }
     else{
      return false;
     }
  }

};

  



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

