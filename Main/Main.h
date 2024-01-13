
#ifndef MAIN_H
#define  MAIN_H

#include <Arduino.h>
#include <IWatchdog.h>
#include <ShiftRegister74HC595.h>
#include <SoftwareSerial.h>
#include <thread>
#include <chrono>
#define DEBUG_ON  
#define DEBUG_OFF 

//  #define POWER_CHECK_DEBUG 
//  #define LINE_STATUS_DEBUG 
// #define LINE_STATE_DEBUG 
//  #define LINE_FIER_DEBUG
// #define LINE_SC_DEBUG

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



#define lineOFF(numberLine) digitalWrite(lineControlPins[numberLine], LOW);
#define lineON(numberLine)  digitalWrite(lineControlPins[numberLine], HIGH);



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


int firstRepeat=0;
  


unsigned long learningProcessCounter = 10;

int i = 0;


char CardPresentError = 0;
char muxPosition = 0;
char cardSituation = 0;

 char ledErrorsPins[12] = { 9, 11, 13, 14, 17, 19, 21, 22, 25, 27, 29, 30 };
 char ledFirePins[12] = { 8, 10, 12, 15, 16, 18, 20, 23, 24, 26, 28, 31 };

char firstSence[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
char shortCircuitDetected[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
 




// Flags

bool card1Present = false;
bool card2Present = false;
//?
bool faultFlag = false;

bool fireTrace = false;

// Data arrays

float lineCurrent[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float lineVoltage[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  // Define the structure for Mux
struct Mux {
    float Values1[8]={0};
    float Values2[8]={0};
    float Values3[8]={0};
    float Values4[8]={0};
};
Mux mux;
const char lineControlPins[12] = {Line1, Line2, Line3, Line4, Line5, Line6, Line7, Line8, Line9, Line10, Line11, Line12 };


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

typedef struct
{
  bool BUZZER= false;
  bool LED_CHECK=false;
  bool RESET=false;
  bool ALARM_RELAY=false;
 
} ButtonState;

ButtonState buttonStatus;


status lineStatus[12] = { NON_STATUS };
powerState powerStatus;

#define SUPPLY_VOLTAGE_IS_16_V digitalWrite(ChangeVolt, HIGH);
#define SUPPLY_VOLTAGE_IS_24_V digitalWrite(ChangeVolt, LOW);

#define POWER_RELAY_ON   digitalWrite(Batcharges,HIGH);
#define POWER_RELAY_OFF  digitalWrite(Batcharges,LOW);


#define BUZZER_ON digitalWrite(MCUbuzz, HIGH);
#define BUZZER_OFF digitalWrite(MCUbuzz, LOW);

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
  void SingelOn(unsigned int timeON,unsigned int timeOFF);
  void Repead(int *NumberRepead,unsigned int timeON,unsigned int timeOFF);

};

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


ShiftRegister74HC595<5> shiftRegister(PC6, PC7, PC13);
#define LED_ON(numerPin)shiftRegister.set(numerPin,LOW);
#define LED_OFF(numerPin)shiftRegister.set(numerPin,HIGH);
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
      // void createObjects(int value) {
      //         arrayOfClassA[index++] = new flowDelay();
      //     }

    void turnOn(char numerPin) {
      LED_ON(numerPin);
        
    }
    void turnOff(char numerPin) {
      
       LED_OFF(numerPin);
    }

    void blink(char numerPin,unsigned int timeBlinking) { 
     static unsigned int turnOnTime[12]={0};
      turnOnTime[id]=timeBlinking;
      ledBlinkerFlow[id].Delay(turnOnTime[id]*2);
      if(ledBlinkerFlow[id].value<=turnOnTime[id]) LED_ON(numerPin);
      if((ledBlinkerFlow[id].value>turnOnTime[id])&&(ledBlinkerFlow[id].value<(turnOnTime[id]*2))) LED_OFF(numerPin);
    }
    void blinkCustum(char numerPin,unsigned int timeOn,unsigned int timeOff) { 
        static unsigned int turnOnTime[12]={0};
        static unsigned int turnOffTime[12]={0};
        turnOnTime[id]=timeOn;
        turnOffTime[id]=timeOff;
        ledBlinkerFlow[id].Delay(timeOn+timeOff);
      if(ledBlinkerFlow[id].value<=timeOn) LED_ON(numerPin);
      if((ledBlinkerFlow[id].value>timeOn)&&(ledBlinkerFlow[id].value<timeOn+timeOff)) LED_OFF(numerPin);
    }
    void turnOnArry(char *arry, unsigned int length) {
        for (unsigned int i = 0; i < length; ++i) {
            LED_ON(arry[i]);
        }
    }

    void turnOffArry(char *arry, unsigned int length) {
      for (unsigned int i = 0; i < length; ++i) {
            LED_OFF(arry[i]);
        }
        
    }

   void blinkCustumArry(char *arry, unsigned int length, unsigned int timeOn, unsigned int timeOff) {
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

    void blinkArry(char *arry, unsigned int length, unsigned int timeBlinking) {

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

   private:

    // flowDelay* arrayOfflowDelay[10]; 
    // int index = 0; 

    void rotateArrayLeft(char *arry, unsigned int length) {
        char temp = arry[0];
        for (unsigned int i = 0; i < length - 1; ++i) {
            arry[i] = arry[i + 1];
        }
        arry[length - 1] = temp;
    }

    void rotateArrayRight(char *arry, unsigned int length) {
        char temp = arry[length - 1];
        for (int i = length - 1; i > 0; --i) {
            arry[i] = arry[i - 1];
        }
        arry[0] = temp;
    }


};

// class LEDManager {
// public:
//     std::vector<LED> leds;

//     void addLED() {

//         LED newLED;
//         leds.push_back(newLED);
//     }

//     void updateLEDs() {
//         /
//         for (size_t i = 0; i < leds.size(); ++i) {
//             leds[i].timer.update();
//             leds[i].blinkCustom('A', 500, 500); 
//         }
//     }
// };






// Define SoftwareSerial for communication
SoftwareSerial mySerial(S1rx, S1tx);  // RX, TX
// Shiftregister setting
// ShiftRegister74HC595<5> sr(PC6, PC7, PC13);

timerMS batteryCheckTime;
flowDelay shortCircuitFlow[12];
flowDelay fierFlow;
flowDelay buttonFlow;
Buzzer buzzer;
 LED LEDWarning(0);
 LED LEDFier(1);
 LED LEDPower(2);




#endif

