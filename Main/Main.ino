#include <Main.h>

#include <IWatchdog.h>
#include <ShiftRegister74HC595.h>
#include <SoftwareSerial.h>
#include <Arduino.h>



// Threshold values
const float OPEN_THRESHOLD = 0.09;
const float NORMAL_THRESHOLD = 0.24;
const float FIRE_THRESHOLD = 1.1;
const float SHORT_CIRCUIT_THRESHOLD = 0.4;
const float LOWER_THRESHOLD_OUT = 0.1;
const float UPPER_THRESHOLD_OUT = 0.49;


// Flags
bool batteryChecking = false;
bool card1Present = false;
bool card2Present= false;
bool relayControl = false;
bool faultFlag = false;
bool fireFlag = false;
bool batteryLowVoltage = false;
bool supplyFault = false;
bool batteryFail = false;
bool powerFail = false;
bool earthFail  = false;
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
bool ledBlinker1  = true;
bool ledBlinker2 = true;
bool buzzerControl  = false;
bool sounderLedStatus  = false;

byte CardPresentError  = 0;
// Data arrays
float mux1Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux2Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux3Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float mux4Values[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float lineCurrent[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float lineVoltage[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte limitTimeSC  = 3;
byte muxPosition  = 0;
byte cardSituation  = 0;
const byte lineControlPins[12] = { PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11 };
const byte ledErrorsPins[12] = { 9, 11, 13, 14, 17, 19, 21, 22, 25, 27, 29, 30 };
const byte ledFirePins[12] = { 8, 10, 12, 15, 16, 18, 20, 23, 24, 26, 28, 31 };

byte lastLineSituations[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
byte firstSence[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte shortCircuitDetected[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


class Filter {
  public:
    Filter(int numSamples) : numSamples_(numSamples) {
        samples_.reserve(numSamples_);
    }

    // Low-Pass Filter Method
    double LowPassFilter(double inputValue) {
        // Store the input value in the samples vector
        samples_.push_back(inputValue);

        // If the number of samples exceeds the specified limit, remove the oldest sample
        if (samples_.size() > numSamples_) {
            samples_.erase(samples_.begin());
        }

        double sum = 0.0;
        // Calculate the sum of all samples in the window
        for (double sample : samples_) {
            sum += sample;
        }

        // Calculate the average of the samples, which is the low-pass filtered value
        return sum / samples_.size();
    }

    // High-Pass Filter Method
    double HighPassFilter(double inputValue) {
        // Store the input value in the samples vector
        samples_.push_back(inputValue);

        // If the number of samples exceeds the specified limit, remove the oldest sample
        if (samples_.size() > numSamples_) {
            samples_.erase(samples_.begin());
        }

        double sum = 0.0;
        // Calculate the sum of all samples in the window
        for (double sample : samples_) {
            sum += sample;
        }

        // Calculate the high-pass filtered value by subtracting the low-pass filtered value
        return inputValue - (sum / samples_.size());
    }

    // Mid-Pass Filter Method
    double MidPassFilter(double inputValue) {
        // Apply both low-pass and high-pass filters in sequence to achieve a mid-pass effect
        return HighPassFilter(LowPassFilter(inputValue));
    }

private:
    int numSamples_;
    std::vector<double> samples_;
};




typedef enum 
{
  NON_STATUS,
  OPEN_CIRCUIT,
  NORMAL,
  FIER,
  SHORT_CIRCUIT
}status;
status lineStatus[12]={NON_STATUS};


 typedef enum{
    STOP,
    START
  
  }STATE;
  

 struct
{
  STATE status;
  unsigned long value=0;
}timer;
//Timer timer;

//timer.status = START;
unsigned long currentTime  = 0;
unsigned long ledBlinkTime  = 0;
unsigned long buttonPressTime  = 0;
unsigned long shortCircuitTime  = 0;
unsigned long buzzerReadyTime  = 0;
unsigned long batteryScanTime =0;

unsigned long fultSencetimer = 0;
unsigned long fultCounter = 10;

unsigned long learningProcessCounter = 10;

int j = 0;
char i = 0;

SoftwareSerial mySerial(S1rx, S1tx);  // RX, TX
// Shiftregister setting
ShiftRegister74HC595<5> sr(PC6, PC7, PC13);
void Update_IT_callback(void);
// Hardware Settings

void setup() {

  GPIOInit();
  mySerial.begin(9600);
  sr.setAllHigh();
  
// Timers configuration
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(10, HERTZ_FORMAT);  // 10 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();

  // Check if Card 1 is present
  if (digitalRead(CS1) == 0)
    card1Present = true;

  // Check if Card 2 is present
  if (digitalRead(CS2) == 0)
    card2Present = true;
    
  if (card1Present & card2Present) {
    cardSituation  = 2;
  
  } else if (card1Present) {
    cardSituation  = 1;
    digitalWrite(Line1, HIGH);
    digitalWrite(Line2, HIGH);
    digitalWrite(Line3, HIGH);
    digitalWrite(Line4, HIGH);
    digitalWrite(Line5, HIGH);
    digitalWrite(Line6, HIGH);
    digitalWrite(Line7, HIGH);
    digitalWrite(Line8, HIGH);
  } else {
    cardSituation  = 0;
    digitalWrite(Line1, HIGH);
    digitalWrite(Line2, HIGH);
    digitalWrite(Line3, HIGH);
    digitalWrite(Line4, HIGH);
  }


  IWatchdog.begin(5000000);  // 5000ms
  shortCircuitTime  = currentTime ;
  
  digitalWrite(LEDerror, HIGH);
  digitalWrite(Sela, LOW);
  digitalWrite(Selb, LOW);
  digitalWrite(Selc, LOW);
  delay(25);
  mux4Values[0] = (3.3 / 1023.00) * analogRead(Analog4);
  delay(15);
  digitalWrite(Sela, HIGH);
  digitalWrite(Selb, HIGH);
  digitalWrite(Selc, LOW);
  delay(25);
  mux4Values[3] = (3.3 / 1023.00) * analogRead(Analog4);
}
    

void loop() {
    Filter filter(5);
    buttonchek();
    digitalWrite(LEDerror, HIGH);
    batpowerchek1();
    Muxread(muxPosition );
    Linechek();
    buttonchek();
   /* buttonchek();
  
      for ( i = 0; i <5; i++  ) 
      {
        Muxread(muxPosition );
        Linechek();
          for(j = 0; j < ((cardSituation  + 1) * 4); j++)
         {
          lineCurrent[j]=filter.LowPassFilter(lineCurrent[j]);
          buttonchek();
         }

      }
    
 buttonchek();*/
        
   
   

    for ( i = 0; i < ((cardSituation  + 1) * 4); i++) {

    

      if(learningProcessCounter==0){

        if ((lineVoltage[i] < SHORT_CIRCUIT_THRESHOLD) && (currentTime  - shortCircuitTime  > 1)) {
          printVoltageAlert(i, lineVoltage[i]);
          digitalWrite(lineControlPins[i], LOW);
          shortCircuitDetected[i]= shortCircuitDetected[i]+2;

        } 
        else {
          
          lineStatus[i] = processCurrentConditions(i);
            mySerial.print(lineStatus[i]);
        }
        if( (shortCircuitDetected[i]>0)&&(currentTime  - shortCircuitTime  >limitTimeSC ) )
        {
          shortCircuitTime=currentTime;
          digitalWrite(lineControlPins[i], HIGH);
          shortCircuitDetected[i]=0;
          limitTimeSC++;
          if (limitTimeSC > 55) limitTimeSC = 4;
        }

      }

    }

    if(learningProcessCounter>0)learningProcessCounter--;
    
    handleThresholdFaults();
    handleSupplyAndpowerFailures();
    handleCardPresentErrors();
  
    if (timeForLedBlink()) {
      toggleLedState();
      Ledcontrol(lineStatus);
    }
    Relaycont();
    IWatchdog.reload();
    updateMuxPosition();
    checkAndEnableBeeper();

    }



// Set all lines low
void SetAllLinesLow() {
  for (int i = 0; i < 12; i++) {
    digitalWrite(lineControlPins[i], LOW);
  }
}

// Set all lines high
void SetAllLinesHigh() {
  for (int i = 0; i < 12; i++) {
    digitalWrite(lineControlPins[i], HIGH);
  }
}

// Set a specific line high
void SetLineHigh(int line) {
  digitalWrite(lineControlPins[line - 1], HIGH);
}

// Set a specific line low
void SetLineLow(int line) {
  digitalWrite(lineControlPins[line - 1], LOW);
}

// Function to check if a threshold fault is detected
bool thresholdFaultDetected() {
  return ((mux4Values[4] > UPPER_THRESHOLD_OUT) || (mux4Values[4] < LOWER_THRESHOLD_OUT) || (mux4Values[5] > UPPER_THRESHOLD_OUT) || (mux4Values[5] < LOWER_THRESHOLD_OUT));
}

// Function to handle threshold faults
void handleThresholdFaults() {
  if (thresholdFaultDetected() && !fireTrace && !relayControl) {

    if (generalFault) generalFault = false;
    relayOn = true;
    buzzerControl  = true;
  } else {
    relayOn = false;
  }
}
// Function to check if a supply failure is detected
bool supplyFailureDetected() {
  return (0.55 < mux4Values[0]);
}
// Function to check if a power failure is detected

bool powerFailureDetected() {
  return (mux4Values[0] < 0.1);
}

// Function to handle supply and power failures
void handleSupplyAndpowerFailures() {
  if (supplyFailureDetected()) {
    supplyFault = false;
    buzzerControl  = true;
    batteryChecking = false;
  } else {
    if (!batteryChecking)
      supplyFault = true;
    else
      batteryChecking = false;
  }

  if (powerFailureDetected()) {
    powerFail = true;
    buzzerControl  = true;
  } else {
    powerFail = false;
  }
}
// Function to check if it's time to toggle the LED
bool timeForLedBlink() {
  return (currentTime  - ledBlinkTime ) > 6;
}

void batpowerchek1() {
  batteryChecking = true;
  float vp[4] = { 0, 0, 0, 0 };
  byte cont = 0;
  if (!relayCharging) {
    if ((mux4Values[3] * 24) > 18.6) {
      relayCharging = true;
      digitalWrite(Batcharges, relayCharging);
    }
  }
  if (relayCharging) {
    if (currentTime  - batteryScanTime   > 50) {
      if ((mux4Values[0] * 41) > 16) {
        digitalWrite(Sela, LOW);
        digitalWrite(Selb, LOW);
        digitalWrite(Selc, LOW);
        batteryScanTime   = currentTime ;
        digitalWrite(ChangeVolt, HIGH);
        delay(55);
        do {
          mux4Values[0] = (3.3 / 1023.00) * analogRead(Analog4);
          vp[cont] = mux4Values[0];
          cont++;
          delay(13);
          if (cont > 2) {
            if (((vp[0] - vp[1]) < 0.02) && ((vp[1] - vp[2]) < 0.02))
              break;
            cont = 0;
          }
        } while ((currentTime  - batteryScanTime  ) > 21);
        digitalWrite(Sela, HIGH);
        digitalWrite(Selb, HIGH);
        digitalWrite(Selc, LOW);
        delay(21);
        mux4Values[3] = (3.3 / 1023.00) * analogRead(Analog4);
        batteryFail = false;
        delay(13);
        if ((mux4Values[3] * 24) < 18.6) {
          relayCharging = false;
          batteryFail = true;
          digitalWrite(Batcharges, relayCharging);
        }
        digitalWrite(ChangeVolt, LOW);
      }
    }
    if ((mux4Values[0] * 41) < 15.6) {
      if ((mux4Values[3] * 24) < 15.6) {
        relayCharging = false;
        batteryFail = true;
        digitalWrite(Batcharges, relayCharging);
      } else if ((mux4Values[3] * 24) < 16.6) {
        batteryLowVoltage = true;
        digitalWrite(MCUbuzz, HIGH);
      } else {
        batteryLowVoltage = false;
      }
    }
  }
  delay(10);
  batteryFail = !relayCharging;
  powerFail = false;
}
// Function to print voltage alert
void printVoltageAlert(byte line, float voltage) {
  mySerial.print("line");
  mySerial.print(line);
  mySerial.print(",");
  mySerial.println(voltage);
}

void Ledcontrol(status lineSituations[12]) {
  for (byte i = 0; i < 12; i++) {

    if ((lineSituations[i] == 1) || (lineSituations[i] == 4)) {  // Fault mode
      sr.set(ledErrorsPins[i], ledBlinker1 );
      if (buzzerControl  && !generalFault && !fireTrace)
        digitalWrite(MCUbuzz, ledBlinker1 );
      else
        digitalWrite(MCUbuzz, LOW);

    } else if (lineSituations[i] == 3) {  // Fire mode
      sr.set(ledFirePins[i], ledBlinker1 );
      sr.set(ledefiremode, LOW);
      if (fireTrace)
        digitalWrite(MCUbuzz, HIGH);
      else
        digitalWrite(MCUbuzz, LOW);
    } else {
      sr.set(ledErrorsPins[i], HIGH);
      sr.set(ledFirePins[i], HIGH);
      if ((batteryFail || powerFail || supplyFault) && !generalFault)
        digitalWrite(MCUbuzz, ledBlinker1 );
      else
        digitalWrite(MCUbuzz, LOW);
    }
    if (fireTrace && !generalFault)
      digitalWrite(MCUbuzz, HIGH);
  }
  if ((generalFault && fireTrace) || (batteryFail || powerFail || supplyFault) && generalFault)
    sr.set(ledebuz, LOW);
  else
    sr.set(ledebuz, HIGH);
  if (supplyFault && !powerFail)
    sr.set(ledepower, LOW);
  else
    sr.set(ledepower, HIGH);
  if (batteryFail)
    sr.set(ledebat, LOW);
  else
    sr.set(ledebat, HIGH);
  if (batteryLowVoltage) {
    sr.set(ledebat, ledBlinker1 );
    sr.set(ledemainpower, ledBlinker1 );
  }
  if (powerFail)
    sr.set(ledemainpower, LOW);
  else
    sr.set(ledemainpower, HIGH);
  if (relayOn)
    sr.set(ledeearth, LOW);
  else
    sr.set(ledeearth, HIGH);
  if (fireTrace)
    sr.set(ledefiremode, LOW);
  else
    sr.set(ledefiremode, HIGH);
  sr.set(panelon, LOW);
  if (relayOn) {
    // mySerial.println("relayOn ok");
    if (!generalFault)
      digitalWrite(MCUbuzz, relayOn);
  }
  sr.set(generalfault, !(supplyFault || batteryFail || powerFail || relayOn));
}


// Function to toggle the LED state
void toggleLedState() {
  ledBlinkTime  = currentTime ;
  ledBlinker1  = !ledBlinker1 ;
}

// Function to update the mux position for analog readings
void updateMuxPosition() {
  if (muxPosition  < 8) {
    if (muxPosition  == 7) {
      muxPosition  = 0;
      readAnalogs = true;
    } else {
      muxPosition ++;
    }
  }
}

// Function to check and enable the beeperEnabled
void checkAndEnableBeeper() {
  if (enableBeeper()) {
    buzzerReadyTime  = currentTime ;
    beeperEnabled = true;
  }
}

// Function to check if the beeperEnabled should be enabled
bool enableBeeper() {
  return (buzzerControl  && generalFault && !fireTrace && (currentTime  - buzzerReadyTime  > 300));
}

// Function to process current conditions
 status processCurrentConditions(byte line) {
    static int fierLouckBit=0;
   static int fierDebounce=0;

       mySerial.print("\n"); 
       mySerial.print("timer.value ");
       mySerial.print(timer.value );
       mySerial.print("\n"); 
       mySerial.print("\n");
       mySerial.print("fierDebounce =");
       mySerial.print(fierDebounce);
       mySerial.print("\n"); 


  if((timer.value > 40 )&&(fierDebounce <= 10)&&(fierLouckBit==0))
 {
    mySerial.print("\n");
    mySerial.print("=========================================================================");
    mySerial.print("\n");
    mySerial.print("============================RESET DEBUNCE================================");
    mySerial.print("\n");
    mySerial.print("=========================================================================");
    mySerial.print("\n");
  fierDebounce=0;
  timer.value=0;
  timer.status=STOP;
 }


 /* mySerial.print("\n");
  mySerial.print("line=");
  mySerial.print(line);
  mySerial.print("Current=");
  mySerial.print(lineCurrent[line]) ;
  mySerial.print("\n");*/


   

  // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
  // Process the current conditions for the line

  if(firstSence[line] == 0){

    if (lineCurrent[line] < OPEN_THRESHOLD) { 

      return OPEN_CIRCUIT;
    
    } else if( ( lineCurrent[line] > OPEN_THRESHOLD ) && (lineCurrent[line] < NORMAL_THRESHOLD) ){

      return NORMAL;

    }

  } 
   
  if ((lineCurrent[line] > NORMAL_THRESHOLD ) && (lineCurrent[line] < FIRE_THRESHOLD)&&(fierLouckBit==0))
  {   
      
        fierDebounce++;
       timer.status = START;
     
     
     
      mySerial.print("\n");
      mySerial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>Detect<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
      mySerial.print("\n");

   

  } 
    

   


  
 

 if( (fierDebounce >9)&&((lineCurrent[line] > NORMAL_THRESHOLD ) && (lineCurrent[line] < FIRE_THRESHOLD) )||(fierLouckBit==1) )
  {

      fierLouckBit=1;
      fierDebounce=0;
      timer.status = STOP;

        mySerial.print("\n");
        mySerial.print("============================FIER================================");
       
        mySerial.print("\n");
    
      // Handle fire detection conditions
      fultSencetimer = 0;  // fire alarming
      delay(55);

      if (firstSence[line] == 1) {

        digitalWrite(lineControlPins[line], LOW);
        delay(55);

      } 
      if (firstSence[line] == 2) {

        digitalWrite(lineControlPins[line], HIGH);
      }

      if (firstSence[line] == 3) {

        fireTrace = true;
        fireFlag = true;
        relayControl = false;
        relayCustomOn = false;
        sr.set(ledebuz, HIGH);
        sr.set(ledesounder, HIGH);
        return FIER;
      }

    firstSence[line] = firstSence[line] + 1;
    delay(55);



          
      }




  
  else
  {
    //fierDebounce=0;

    if (firstSence[line] == 0)
   
       return SHORT_CIRCUIT;
     
  }

}

// Function to check if all values in a range are zero
bool allZerosInRange(int start, int end) {
  for (int i = start; i <= end; i++) {
    if (lineCurrent[i] != 0) {
      return false;
    }
  }
  return true;
}



// Function to handle card present errors
void handleCardPresentErrors() {
  if (cardSituation  == 1) {
    if (allZerosInRange(4, 7)) {
      CardPresentError  = 1;
    } else {
      CardPresentError  = 0;
    }
  }
  if (cardSituation  == 2) {
    if (allZerosInRange(4, 7) && allZerosInRange(8, 11)) {
      CardPresentError  = 3;
    } else if (allZerosInRange(8, 11)) {
      CardPresentError  = 2;
    } else {
      CardPresentError  = 0;
    }
  }
}



void Linechek() {

  switch (cardSituation ) {
    case 0:
      lineVoltage[3] = mux1Values[0];
      lineCurrent[3] = mux1Values[3];
      lineCurrent[2] = mux1Values[2];
      lineVoltage[2] = mux1Values[1];
      lineCurrent[1] = mux1Values[4];
      lineVoltage[1] = mux1Values[6];
      lineVoltage[0] = mux1Values[5];
      lineCurrent[0] = mux1Values[7];
      // mySerial.print(lineCurrent[i]);
      break;
    case 1:
      lineVoltage[3] = mux1Values[0];
      lineCurrent[3] = mux1Values[3];
      lineCurrent[2] = mux1Values[2];
      lineVoltage[2] = mux1Values[1];
      lineCurrent[1] = mux1Values[4];
      lineVoltage[1] = mux1Values[6];
      lineVoltage[0] = mux1Values[5];
      lineCurrent[0] = mux1Values[7];

      lineVoltage[7] = mux2Values[0];
      lineCurrent[7] = mux2Values[3];
      lineCurrent[6] = mux2Values[1];
      lineVoltage[6] = mux2Values[2];
      lineCurrent[5] = mux2Values[6];
      lineVoltage[5] = mux2Values[4];
      lineVoltage[4] = mux2Values[5];
      lineCurrent[4] = mux2Values[7];
      break;
    case 2:
      lineVoltage[3] = mux1Values[0];
      lineCurrent[3] = mux1Values[3];
      lineCurrent[2] = mux1Values[2];
      lineVoltage[2] = mux1Values[1];
      lineCurrent[1] = mux1Values[4];
      lineVoltage[1] = mux1Values[6];
      lineVoltage[0] = mux1Values[5];
      lineCurrent[0] = mux1Values[7];

      lineVoltage[7] = mux2Values[0];
      lineCurrent[7] = mux2Values[3];
      lineCurrent[6] = mux2Values[1];
      lineVoltage[6] = mux2Values[2];
      lineCurrent[5] = mux2Values[6];
      lineVoltage[5] = mux2Values[4];
      lineVoltage[4] = mux2Values[5];
      lineCurrent[4] = mux2Values[7];

      lineVoltage[11] = mux3Values[0];
      lineCurrent[11] = mux3Values[3];
      lineCurrent[10] = mux3Values[1];
      lineVoltage[10] = mux3Values[2];
      lineCurrent[9] = mux3Values[6];
      lineVoltage[9] = mux3Values[4];
      lineVoltage[8] = mux3Values[5];
      lineCurrent[8] = mux3Values[7];
      break;
    default:
      break;
  }
}


void Muxread(byte add) {
  // Define control values for Sela, Selb, and Selc
  byte controlValues[3] = { LOW, LOW, LOW };

  // Calculate control values based on the address (add)
  if (add & 0b001) controlValues[0] = HIGH;
  if (add & 0b010) controlValues[1] = HIGH;
  if (add & 0b100) controlValues[2] = HIGH;

  // Set the control values
  digitalWrite(Sela, controlValues[0]);
  digitalWrite(Selb, controlValues[1]);
  digitalWrite(Selc, controlValues[2]);

  // Delay as needed
  delay(25);

  // Read analog values and store them in the mux arrays
  for (int i = 0; i < 4; i++) {
    mux1Values[add] = (3.3 / 1023.00) * analogRead(Analog1);
    mux2Values[add] = (3.3 / 1023.00) * analogRead(Analog2);
    mux3Values[add] = (3.3 / 1023.00) * analogRead(Analog3);
    mux4Values[add] = (3.3 / 1023.00) * analogRead(Analog4);
  }

  // Delay as needed
  delay(15);
}


void buttonchek() {
  if (digitalRead(But5) == 0) {  // Buzzer off
    if (currentTime  - buttonPressTime  > 10) {
      buttonPressTime  = currentTime ;
      generalFault = !generalFault;
    }
  }
  if (digitalRead(But4) == 0) {  // LED chek
    sr.setAllLow();
    delay(550);
    byte initi = 0;
    initi = CardPresentError ;
    CardPresentError  = 0;
    int x = 0;
    do {
      IWatchdog.reload();
      delay(50);
      x++;
      if (x > 21)
        digitalWrite(LEDerror, LOW);
    } while (digitalRead(But4) == 0);
    IWatchdog.reload();
    delay(550);
    IWatchdog.reload();
    sr.setAllHigh();
    CardPresentError  = initi;
  }
  if (digitalRead(But3) == 0) {  // All Line Reset
    digitalWrite(Line1, LOW);
    digitalWrite(Line2, LOW);
    digitalWrite(Line3, LOW);
    digitalWrite(Line4, LOW);
    digitalWrite(Line5, LOW);
    digitalWrite(Line6, LOW);
    digitalWrite(Line7, LOW);
    digitalWrite(Line8, LOW);
    digitalWrite(Line9, LOW);
    digitalWrite(Line10, LOW);
    digitalWrite(Line11, LOW);
    digitalWrite(Line12, LOW);
    for (byte i = 0; i < 8; i++) {
      mux1Values[i] = 0;
      mux2Values[i] = 0;
      mux3Values[i] = 0;
      mux4Values[i] = 0;
    }
    for (byte i = 0; i < 12; i++) {
      lineCurrent[i] = 0;
      lineVoltage[i] = 0;
      //lineSituations[i] = 0;      // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
      lastLineSituations[i] = 0;  // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
      firstSence[i] = 0;
      shortCircuitDetected[i] = 0;
    }
    limitTimeSC  = 3;
    muxPosition  = 0;
    // cardSituation  = 0;
    currentTime  = 0;
    ledBlinkTime  = 0;
    buttonPressTime  = 0;
    shortCircuitTime  = 0;
    buzzerReadyTime  = 0;
    fultSencetimer = 0;
    ledBlinker1  = true;
    ledBlinker2 = true;
    buzzerControl  = false;
    sounderLedStatus  = false;
    supplyFault = false;
    batteryFail = false;
    powerFail = false;
    earthFail  = false;
    generalFault = false;
    fireTrace = false;
    relayControl = false;
    readAnalogs = false;
    buzzerEnabled = false;
    beeperEnabled = false;
    relayOn = false;
    fireFlag = false;
    faultFlag = false;
    fultCounter = 0;
    digitalWrite(MCUbuzz, LOW);
    IWatchdog.reload();
    delay(900);
    IWatchdog.reload();
    if (cardSituation  == 2) {
      digitalWrite(Line1, HIGH);
      digitalWrite(Line2, HIGH);
      digitalWrite(Line3, HIGH);
      digitalWrite(Line4, HIGH);
      digitalWrite(Line5, HIGH);
      digitalWrite(Line6, HIGH);
      digitalWrite(Line7, HIGH);
      digitalWrite(Line8, HIGH);
      digitalWrite(Line9, HIGH);
      digitalWrite(Line10, HIGH);
      digitalWrite(Line11, HIGH);
      digitalWrite(Line12, HIGH);
    } else if (cardSituation  == 1) {
      digitalWrite(Line1, HIGH);
      digitalWrite(Line2, HIGH);
      digitalWrite(Line3, HIGH);
      digitalWrite(Line4, HIGH);
      digitalWrite(Line5, HIGH);
      digitalWrite(Line6, HIGH);
      digitalWrite(Line7, HIGH);
      digitalWrite(Line8, HIGH);
    } else {
      digitalWrite(Line1, HIGH);
      digitalWrite(Line2, HIGH);
      digitalWrite(Line3, HIGH);
      digitalWrite(Line4, HIGH);
    }
    delay(100);
    IWatchdog.reload();
    Muxread(muxPosition );
  }
  if (digitalRead(But2) == 0) {  // Alarm rely on
    if (currentTime  - buttonPressTime  > 13) {
      buttonPressTime  = currentTime ;
      relayControl = true;
      sr.set(ledesounder, HIGH);
      //  mySerial.println("alarm on");
      if (sounderLedStatus ) {
        sounderLedStatus  = !sounderLedStatus ;
        relayCustomOn = false;
      }
    }
  }
  if (digitalRead(But1) == 0) {  // Alarm rely off

    if (currentTime  - buttonPressTime  > 13) {
      buttonPressTime  = currentTime ;
      relayControl = false;
      // mySerial.println("alarm off");
      if (fireFlag) {
        fireFlag = false;
        sr.set(ledesounder, LOW);
      }
      if (!sounderLedStatus )
        sounderLedStatus  = !sounderLedStatus ;
      relayCustomOn = true;
    }
  }
}

void GPIOInit(void) {

  pinMode(rel1, OUTPUT);
  pinMode(rel2, OUTPUT);
  pinMode(relo1, OUTPUT);
  pinMode(relo2, OUTPUT);
  digitalWrite(rel1, LOW);
  digitalWrite(rel2, LOW);
  digitalWrite(relo1, LOW);
  digitalWrite(relo2, LOW);
  // Line Settings
  pinMode(Line1, OUTPUT);
  pinMode(Line2, OUTPUT);
  pinMode(Line3, OUTPUT);
  pinMode(Line4, OUTPUT);
  pinMode(Line5, OUTPUT);
  pinMode(Line6, OUTPUT);
  pinMode(Line7, OUTPUT);
  pinMode(Line8, OUTPUT);
  pinMode(Line9, OUTPUT);
  pinMode(Line10, OUTPUT);
  pinMode(Line11, OUTPUT);
  pinMode(Line12, OUTPUT);
  digitalWrite(Line1, LOW);
  digitalWrite(Line2, LOW);
  digitalWrite(Line3, LOW);
  digitalWrite(Line4, LOW);
  digitalWrite(Line5, LOW);
  digitalWrite(Line6, LOW);
  digitalWrite(Line7, LOW);
  digitalWrite(Line8, LOW);
  digitalWrite(Line9, LOW);
  digitalWrite(Line10, LOW);
  digitalWrite(Line11, LOW);
  digitalWrite(Line12, LOW);
  // Button Settings
  pinMode(CS1, INPUT);
  pinMode(CS2, INPUT);
  pinMode(But1, INPUT);
  pinMode(But2, INPUT);
  pinMode(But3, INPUT);
  pinMode(But4, INPUT);
  pinMode(But5, INPUT);
  // Battery Charges Settings
  pinMode(Batcharges, OUTPUT);
  pinMode(ChangeVolt, OUTPUT);
  digitalWrite(Batcharges, LOW);
  digitalWrite(ChangeVolt, LOW);
  // Errors Settings
  pinMode(LEDerror, OUTPUT);
  pinMode(MCUbuzz, OUTPUT);
  digitalWrite(LEDerror, LOW);
  digitalWrite(MCUbuzz, LOW);
  // Analog settings
  pinMode(Sela, OUTPUT);
  pinMode(Selb, OUTPUT);
  pinMode(Selc, OUTPUT);
  digitalWrite(Sela, LOW);
  digitalWrite(Selb, LOW);
  digitalWrite(Selc, LOW);
  pinMode(Analog1, INPUT_ANALOG);
  pinMode(Analog2, INPUT_ANALOG);
  pinMode(Analog3, INPUT_ANALOG);
  pinMode(Analog4, INPUT_ANALOG);

   pinMode(LEDerror, OUTPUT);
}

void Update_IT_callback(void) {  // 10hz
  currentTime ++;
  fultSencetimer++;

    if (timer.status  == START) 
    {
       timer.value++;
    } else if (timer.status ==STOP) 
    {
      timer.value=0;
    }
  
  
  ledBlinker2 = !ledBlinker2;
  if (CardPresentError  > 0) {
    if (CardPresentError  == 1) {
      sr.set(ledErrorsPins[4], ledBlinker2);
      sr.set(ledErrorsPins[5], ledBlinker2);
      sr.set(ledErrorsPins[6], ledBlinker2);
      sr.set(ledErrorsPins[7], ledBlinker2);
    } else if (CardPresentError  == 2) {
      sr.set(ledErrorsPins[8], ledBlinker2);
      sr.set(ledErrorsPins[9], ledBlinker2);
      sr.set(ledErrorsPins[10], ledBlinker2);
      sr.set(ledErrorsPins[11], ledBlinker2);
    } else if (CardPresentError  == 3) {
      sr.set(ledErrorsPins[4], ledBlinker2);
      sr.set(ledErrorsPins[5], ledBlinker2);
      sr.set(ledErrorsPins[6], ledBlinker2);
      sr.set(ledErrorsPins[7], ledBlinker2);
      sr.set(ledErrorsPins[8], ledBlinker2);
      sr.set(ledErrorsPins[9], ledBlinker2);
      sr.set(ledErrorsPins[10], ledBlinker2);
      sr.set(ledErrorsPins[11], ledBlinker2);
    }
  }
  if (fultCounter > 0) {
    fultCounter = fultCounter - 1;
    faultFlag = true;
  } else
    faultFlag = false;
}

void Relaycont() {
  digitalWrite(rel2, faultFlag);
  // mySerial.println(faultFlag);
  if (fireFlag && !relayCustomOn) {
    digitalWrite(rel1, HIGH);
    digitalWrite(relo1, HIGH);
    digitalWrite(relo2, HIGH);
  } else {
    if (relayControl) {
      digitalWrite(rel1, HIGH);
      digitalWrite(relo1, HIGH);
      digitalWrite(relo2, HIGH);
      // mySerial.println("rely void");
    } else {
      digitalWrite(rel1, LOW);
      digitalWrite(relo1, LOW);
      digitalWrite(relo2, LOW);
    }
  }
}

