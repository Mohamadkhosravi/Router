
#ifndef MAIN_H
#define  MAIN_H

#include <Arduino.h>
#include <IWatchdog.h>
#include <ShiftRegister74HC595.h>
#include <SoftwareSerial.h>
#include <thread>
#include <chrono>

/***** ***** ***** ***** *****/

/***** ***** ***** ***** *****/ 
// #define POWER_CHECK_DEBUG 
// #define LINE_STATUS_DEBUG 
// #define LINE_STATE_DEBUG 
// #define LINE_FIER_DEBUG
// #define LINE_SC_DEBUG
// #define EVENT_DEBUG
/***** ***** ***** ***** *****/ 
 //#define BUZZER_SUND_ON
/***** ***** ***** ***** *****/ 
#define DEBUG_ON  mySerial.print
#define DEBUG_OFF 

#ifdef  POWER_CHECK_DEBUG 
  #define POWER_CHECK_DEBUG   DEBUG_ON
#else
  #define POWER_CHECK_DEBUG   DEBUG_OFF
#endif

#ifdef  LINE_STATE_DEBUG
  #define LINE_STATE_DEBUG   DEBUG_ON
#else
  #define LINE_STATE_DEBUG   DEBUG_OFF 
#endif

#ifdef  LINE_STATUS_DEBUG
  #define LINE_STATUS_DEBUG   DEBUG_ON
#else
  #define LINE_STATUS_DEBUG   DEBUG_OFF 
#endif

#ifdef  LINE_FIER_DEBUG
  #define LINE_FIER_DEBUG   DEBUG_ON
#else
  #define LINE_FIER_DEBUG   DEBUG_OFF 
#endif

#ifdef  LINE_SC_DEBUG 
  #define LINE_SC_DEBUG   DEBUG_ON
#else
  #define LINE_SC_DEBUG   DEBUG_OFF
#endif

#ifdef  EVENT_DEBUG 
  #define EVENT_DEBUG   DEBUG_ON
#else
  #define EVENT_DEBUG   DEBUG_OFF
#endif

// Defines Relys
#define rel1 PD1
#define rel2 PD2
#define relo2 PD0
#define relo1 PB15
// Define Errors pin
#define LEDerror PB12
#define BUZZER_PIN PB14
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
#define analogSelectionA PA4
#define analogSelectionB PA5
#define analogSelectionC PA6
// Analog channels
#define Analog1 PA0
#define Analog2 PA1
#define Analog3 PA2
#define Analog4 PA3
// Battery Charges
#define BATTERY_CHARGER_PIN PC15
#define CHANGE_VOLTAGE PC14
// Card Selector
#define CS1 PA7
#define CS2 PA8

#define MUX_POWER_SUPPLY_VOLTAGE mux.Values4[0]
#define MUX_MAIN_VOLTAGE mux.Values4[2]
#define MUX_BATTERY_VOLTAGE mux.Values4[3]
#define MUX_VOLTAGE_ALART_1 mux.Values4[4]
#define MUX_VOLTAGE_ALART_2 mux.Values4[5]
#define MUX_EARTH mux.Values4[7]

#define SUPPLY_VOLTAGE_IS_16_V digitalWrite(CHANGE_VOLTAGE, HIGH);
#define SUPPLY_VOLTAGE_IS_24_V digitalWrite(CHANGE_VOLTAGE, LOW);

#define POWER_RELAY_ON   digitalWrite(BATTERY_CHARGER_PIN,HIGH);
#define POWER_RELAY_OFF  digitalWrite(BATTERY_CHARGER_PIN,LOW);


#define BUZZER_ON digitalWrite(BUZZER_PIN, HIGH);
#define BUZZER_OFF digitalWrite(BUZZER_PIN, LOW);


#define LINE_OFF(numberLine) digitalWrite(lineControlPins[numberLine], LOW);
#define LINE_ON(numberLine)  digitalWrite(lineControlPins[numberLine], HIGH);

ShiftRegister74HC595<5> shiftRegister(PC6, PC7, PC13);
#define LED_ON(numerPin)shiftRegister.set(numerPin,LOW);
#define LED_OFF(numerPin)shiftRegister.set(numerPin,HIGH);

#define MAXIMUM_REPEAD_TURN_ON 12





int firstRepeat=0;
int lineNumber=0;
char CardPresentError = 0;
char muxPosition = 0;
char cardSituation = 0;
// Flags
bool card1Present = false;
bool card2Present = false;
//?
bool faultFlag = false;
bool fireTrace = false;

// Data arrays
float lineCurrent[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float lineVoltage[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const char lineControlPins[12] = {Line1, Line2, Line3, Line4, Line5, Line6, Line7, Line8, Line9, Line10, Line11, Line12 };

struct LEDs
  {
   const char GENERAL=0;
   const char MANITOR_ALARM=1;
   const char BUZZER=2;
   const char ALARM=3;
   const char MAIN_VOLTAGE=35;
   const char BATTERY=36;
   const char POWER=37;
   const char FIER_OUTBREAK=38;
   const char ALL_CONDITION=39;
   const char SYSTEM=28;//This pin number is GPIO PIN ==> PB12
    char WARNING[12] = { 9, 11, 13, 14, 17, 19, 21, 22, 25, 27, 29, 30 };
    char FIER[12] = { 8, 10, 12, 15, 16, 18, 20, 23, 24, 26, 28, 31 };
    char SINGEL_LEDS[9]={0,1,2,3,35,36,37,38,39};
 };

  // Define the structure for Mux
struct Mux {
    float Values1[8]={0};
    float Values2[8]={0};
    float Values3[8]={0};
    float Values4[8]={0};
};
Mux mux;


typedef enum {
  NON_STATUS,
  OPEN_CIRCUIT,
  NORMAL,
  FIER,
  SHORT_CIRCUIT,
  DAMAGED,
  CHECK

} status;

typedef enum
{
  NORMAL_POWER,
  BATTERY,
  LOW_BATTERY,
  POWER_SUPPLY,
  BATTERY_BROKEN,
  POWER_OFF

} powerState;

 enum stateEvents{
  Happened,
  HappenedAgain,
  NormalEvent,
  line1Open
  };



struct event {
  status LineStatus[12]; 
  powerState PowerState;
  bool MainVoltageState=false;
  bool OutputAlartState=false;
};

 enum typeOfEvent{
     NotHappening,
     HappeningOnLine,
     HappeningOnPower,
     HappeningOnMain,
     HappeningOnOutput
  };

 struct valueOfEvent {
    status StateLine;
    char numberLine=0;
    powerState StatePower;
    // bool MainState=false;
    // bool OutputState=false;
  };

 struct eventStatus{
  stateEvents State;
  typeOfEvent TypeOfEvent;
  valueOfEvent ValueOfEvent;
  bool FierTrack=false;
};


typedef struct
{
  bool BUZZER= false;
  bool LED_CHECK=false;
  bool RESET=false;
  bool ALARM_RELAY=false;
 
} ButtonState;


status lineStatus[12] = { NON_STATUS };
powerState powerStatus;


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
flowDelay(){

}
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


class Buzzer{
    public:
    flowDelay buzzerFlow;
    flowDelay buzzerRepeadFlow;
    unsigned int buzzerTimeON=0;
    unsigned int buzzerTimeOFF=0;
    unsigned int buzzerRepedTimeOFF=0;
    unsigned int buzzerRepedTimeON=0;
    int *numberRepead;
    bool AlarmActive =false;
    void Begin (bool ActivityState);
    void TurnOn(bool ActivityState);
    void TurnOff(void);
    void localBib(void);
    void SingelOn(unsigned int timeON,unsigned int timeOFF);
    void Repead(int *NumberRepead,unsigned int timeON,unsigned int timeOFF);

};

void Buzzer::Begin (bool ActivityState)
{
  static bool flag=0;
  #define CONDITION_SINGLE_ON (buzzerFlow.value>1)&&(buzzerFlow.value< buzzerTimeON) 
  #define CONDITION_SINGLE_OFF (buzzerFlow.value>= buzzerTimeON)&&(buzzerFlow.value<= buzzerTimeOFF) 
  #define CONDITION_REPEAD_ON  (buzzerRepeadFlow.value>1) && (buzzerRepeadFlow.value<buzzerRepedTimeON) && (*numberRepead>0 )
  #define CONDITION_REPEAD_OFF (buzzerRepeadFlow.value>= buzzerRepedTimeON) && (buzzerRepeadFlow.value<= buzzerRepedTimeOFF)    
    if(ActivityState)
    {   
        if(CONDITION_SINGLE_ON)BUZZER_ON
        else if((CONDITION_SINGLE_OFF)&&(AlarmActive == false))
        {
          BUZZER_OFF 
        }
        else if(AlarmActive==false){
          BUZZER_OFF 
          buzzerRepeadFlow.status=STOP;
        }
        if(AlarmActive==true)BUZZER_ON    
    }
    else{
        BUZZER_OFF 
        buzzerRepeadFlow.status=STOP;
  
    }  
}


void Buzzer::Repead(int *NumberRepead,unsigned int timeON,unsigned int timeOFF)
{
  buzzerRepedTimeON =timeON;
  buzzerRepedTimeOFF=timeOFF+timeON;
  buzzerRepeadFlow.Delay(timeON+timeOFF);
  numberRepead=NumberRepead;
}

void Buzzer::TurnOn(bool ActivityState){
  
  if((ActivityState))
  {
    BUZZER_ON
    AlarmActive=true;
  }
  else AlarmActive=false; 
}



void Buzzer::TurnOff(void){
  BUZZER_OFF
}
void Buzzer::SingelOn(unsigned int timeON,unsigned int timeOFF){
     buzzerTimeON =timeON;
      buzzerTimeOFF=timeOFF+timeON;
      buzzerFlow.Delay(timeON+timeOFF);
}
void Buzzer::localBib(void)
{
  BUZZER_ON
  delay(50);
  BUZZER_OFF
};



flowDelay timer;
flowDelay ledBlinkerFlow[12];

class LED{
 public:

 LED( unsigned char ID){
  id=ID;
 }
 unsigned char id=0;
 bool ActivityState;


  enum class Behavior {
        Constant,
        Blinking,
        CustomBlinking,
        Custom
    };
  void LEDBegin(void){
  
  }
  void turnOn(char numerPin); 
  void turnOff(char numerPin);
  void blink(char numerPin,unsigned int timeBlinking); 
  void blinkCustum(char numerPin,unsigned int timeOn,unsigned int timeOff); 
  void turnOnArry(char *arry, unsigned int length); 
  void turnOffArry(char *arry, unsigned int length); 
  void blinkCustumArry(char *arry, unsigned int length, unsigned int timeOn, unsigned int timeOff); 
  void blinkArry(char *arry, unsigned int length, unsigned int timeBlinking);
   
};
void LED::turnOn(char numerPin) {
  LED_ON(numerPin); 
}
void LED::turnOff(char numerPin) {
  LED_OFF(numerPin);
}

void LED::blink(char numerPin,unsigned int timeBlinking) { 
  static unsigned int turnOnTime[12]={0};
  turnOnTime[id]=timeBlinking;
  ledBlinkerFlow[id].Delay(turnOnTime[id]*2);
  if(ledBlinkerFlow[id].value<=turnOnTime[id]) LED_ON(numerPin);
  if((ledBlinkerFlow[id].value>turnOnTime[id])&&(ledBlinkerFlow[id].value<(turnOnTime[id]*2))) LED_OFF(numerPin);
}
void LED::blinkCustum(char numerPin,unsigned int timeOn,unsigned int timeOff) { 
  static unsigned int turnOnTime[12]={0};
  static unsigned int turnOffTime[12]={0};
  turnOnTime[id]=timeOn;
  turnOffTime[id]=timeOff;
  ledBlinkerFlow[id].Delay(timeOn+timeOff);
  if(ledBlinkerFlow[id].value<=timeOn) LED_ON(numerPin);
  if((ledBlinkerFlow[id].value>timeOn)&&(ledBlinkerFlow[id].value<timeOn+timeOff)){ LED_OFF(numerPin);}
  if((ledBlinkerFlow[id].value>timeOn))LED_OFF(numerPin);
}


void LED::turnOnArry(char *arry, unsigned int length) {
  for (unsigned int i = 0; i < length; ++i) {
      LED_ON(arry[i]);
  }
}
void LED::turnOffArry(char *arry, unsigned int length) {
for (unsigned int i = 0; i < length; ++i) {
      LED_OFF(arry[i]);
  }
  
}

void LED::blinkCustumArry(char *arry, unsigned int length, unsigned int timeOn, unsigned int timeOff) {
  static unsigned int turnOnTime[12]={0};
  static unsigned int turnOffTime[12]={0};
  turnOnTime[id]=timeOn;
  turnOffTime[id]=timeOff;
  for (unsigned int i = 0; i < length; ++i) {
      ledBlinkerFlow[id].Delay( turnOnTime[id] + turnOffTime[id]);
      if (ledBlinkerFlow[id].value<= turnOnTime[id])
          LED_ON(arry[i]);
      if ((ledBlinkerFlow[id].value>turnOnTime[id]) && (ledBlinkerFlow[id].value< turnOnTime[id] + turnOffTime[id]))
          LED_OFF(arry[i]);
  }
}

void LED::blinkArry(char *arry, unsigned int length, unsigned int timeBlinking) {
  static unsigned int turnOnTime[12]={0};
  turnOnTime[id]=timeBlinking;
  for (unsigned int i = 0; i < length; ++i) {
      ledBlinkerFlow[id].Delay(turnOnTime[id]*2);
      if (ledBlinkerFlow[id].value <= turnOnTime[id])
          LED_ON(arry[i]);
      if ((ledBlinkerFlow[id].value > turnOnTime[id]) && (ledBlinkerFlow[id].value <( turnOnTime[id]*2 )) )
          LED_OFF(arry[i]);
  }
}


void(* resetFunc) (void) = 0;//declare reset function at address 0


SoftwareSerial mySerial(S1rx, S1tx);  // RX, TX
timerMS batteryCheckTime;
flowDelay shortCircuitFlow[12];
flowDelay fierFlow;
flowDelay buttonFlow;
flowDelay eventFlow;
Buzzer buzzer;
LED LEDWarning(0);
LED LEDFier(1);
LED LEDPower(2);


namespace Output {


 
void LEDManagement(status lineStatus[12],powerState powerStatus,ButtonState *buttonStatus, bool mainVoltageState,bool outputAlart,bool existenceEarth);
void RelayManagement(ButtonState *buttonStatus,eventStatus *newEven);
void BuzzerManagement(ButtonState  *buttonStatus,eventStatus *newEven);

}




#endif

