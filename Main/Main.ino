#include <Main.h>

SoftwareSerial mySerial(S1rx, S1tx);  // RX, TX
// Shiftregister setting
ShiftRegister74HC595<5> sr(PC6, PC7, PC13);
timerMS Timer;
timerMS batteryCheckTime;
timerMS readMuxBatteryTimer;
timerMS readMuxPowerSupplyTimer;
timerMS lineConditionTimer;
timerMS fierTimer;
flowDelay flow[10];

void Update_IT_callback1(void);
void Update_IT_callback2(void);
// Hardware Settings
int firstRepeat=0;
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
  MyTim->attachInterrupt(Update_IT_callback1);
  MyTim->resume();


  TIM_TypeDef *Instance1 =TIM3;
  HardwareTimer *MyTim2 = new HardwareTimer(Instance1);
  MyTim2->setOverflow(1000, HERTZ_FORMAT);  // 10 Hz
  MyTim2->attachInterrupt(Update_IT_callback2);
  MyTim2->resume();

  readMuxBatteryTimer.status== START;
  SUPPLY_VOLTAGE_IS_24_V


  // Check if Card 1 is present
  if (digitalRead(CS1) == 0)
    card1Present = true;

  // Check if Card 2 is present
  if (digitalRead(CS2) == 0)
    card2Present = true;

  if (card1Present & card2Present) {
    cardSituation = 2;
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

  } else if (card1Present) {
    cardSituation = 1;
    digitalWrite(Line1, HIGH);
    digitalWrite(Line2, HIGH);
    digitalWrite(Line3, HIGH);
    digitalWrite(Line4, HIGH);
    digitalWrite(Line5, HIGH);
    digitalWrite(Line6, HIGH);
    digitalWrite(Line7, HIGH);
    digitalWrite(Line8, HIGH);
  } else 
  {
    cardSituation = 0;
    digitalWrite(Line1, HIGH);
    digitalWrite(Line2, HIGH);
    digitalWrite(Line3, HIGH);
    digitalWrite(Line4, HIGH);
  }

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
  IWatchdog.begin(7000000);  // 5000ms
  shortCircuitTime = currentTime;

  digitalWrite(LEDerror, HIGH);
  digitalWrite(Sela, LOW);
  digitalWrite(Selb, LOW);
  digitalWrite(Selc, LOW);
  delay(25);
  mux4Values[0] = ((3.3 / 1023.00) * analogRead(Analog4))*100;
  delay(15);
  digitalWrite(Sela, HIGH);
  digitalWrite(Selb, HIGH);
  digitalWrite(Selc, LOW);
  delay(25);
  mux4Values[3] = ((3.3 / 1023.00) * analogRead(Analog4))*100;
 
}


void loop() {

     IWatchdog.reload();
     checkPower(readBattery(),readPowerSupply());
     readMux(muxPosition);
     
      checkButtons();
     distributionMuxValues();
   
     LINE_STATUS_DEBUG("\n <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<+>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
     
     if (firstRepeat>12)
     {
        for (i = 0; i < ((cardSituation + 1) * 4); i++) 
        {   
        //handelShortCircuit(i);
        lineStatus[i] = processCurrentConditions(lineCurrent[i],lineVoltage[i],i);
       // lineStatus[i] = processCurrentConditions(i);
        }
      }
      else
      {
          firstRepeat++;
      }
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

void handelShortCircuit(byte numberLine)
{
      LINE_STATUS_DEBUG("\n");
      LINE_STATUS_DEBUG("line");
      LINE_STATUS_DEBUG(numberLine+1);
      LINE_STATUS_DEBUG(" , Voltage =");
      LINE_STATUS_DEBUG(lineVoltage[numberLine]);
      LINE_STATUS_DEBUG(", Current=");
      LINE_STATUS_DEBUG(lineCurrent[numberLine]);
      LINE_STATUS_DEBUG(" ");
      LINE_STATUS_DEBUG(", STATUS =");

  

    if ((shortCircuitDetected[numberLine] > 0) && (currentTime - shortCircuitTime > limitTimeSC)) {

    shortCircuitTime = currentTime;
    lineON(numberLine);
    shortCircuitDetected[numberLine] = 0;
    limitTimeSC++;
    if (limitTimeSC > 55) limitTimeSC = 4;

    }

  
    
    if ((lineVoltage[numberLine] < SHORT_CIRCUIT_THRESHOLD) && (currentTime - shortCircuitTime>1 )) {

  
    LINE_STATUS_DEBUG("ShortCircuit line(detect from Current) &");

  

    lineOFF(numberLine);
    shortCircuitDetected[numberLine] = shortCircuitDetected[numberLine] + 2;
    } 
 
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
    buzzerControl = true;
  } else {
    relayOn = false;
  }
}
// Function to check if a supply failure is detected
// bool supplyFailureDetected() {
//   return (0.55 < mux4Values[0]);
// }
// Function to check if a power failure is detected

// bool powerFailureDetected() {
//   return (mux4Values[0] < 0.1);
// }

// Function to handle supply and power failures
void handleSupplyAndpowerFailures() {
  if ( (mux4Values[0])>0.55) {
    supplyFault = false;
    buzzerControl = true;
    batteryChecking = false;
  } else {
    if (!batteryChecking)
      supplyFault = true;
    else
      batteryChecking = false;
  }

  if ((mux4Values[0] ) < 0.1){
    powerFail = true;
    buzzerControl = true;
  } else {
    powerFail = false;
  }
}
// Function to check if it's time to toggle the LED
bool timeForLedBlink() {
  return (currentTime - ledBlinkTime) > 6;
}

void checkPower(float VoltageBattery, float VoltagePowerSupply) {

  static bool battery= false;
  static bool supply= false;

    typedef enum
    {
      NORMAL,
      BATTERY,
      LOW_BATTERY,
      POWER_SUPPLY,
      BATTERY_BROKEN,
      POWER_OFF

    } powerState;
   static powerState state;
 
   
   // Fiter VoltagePowerSupply & VoltageBattery Value
    if (VoltagePowerSupply<=6)VoltagePowerSupply=0;
    if (VoltageBattery<=6)VoltageBattery=0;

   //Debug

    POWER_CHECK_DEBUG("\n => STATE ==");        
    POWER_CHECK_DEBUG(state);
    POWER_CHECK_DEBUG(", VoltageBattery==");
    POWER_CHECK_DEBUG(VoltageBattery,10);
    POWER_CHECK_DEBUG(", VoltagePowerSupply==");
    POWER_CHECK_DEBUG(VoltagePowerSupply,10);
    POWER_CHECK_DEBUG(" |");

   
    supply=( VoltagePowerSupply >10 )? true : false;
    battery=( VoltageBattery >10 )? true : false;

    if((supply==true) && (battery == true)&&(state!=POWER_SUPPLY)&&(state!=BATTERY_BROKEN)){
   
      if (VoltageBattery<18)state= BATTERY_BROKEN;
      else if (VoltageBattery>18)state= NORMAL;
     
    }
    if((supply==true) && (battery == false))state = POWER_SUPPLY;
    if((supply==false) && (battery == true)){
       
        if((VoltageBattery<18)&&(VoltageBattery>17))state=LOW_BATTERY;
        else if(VoltageBattery<17)state= BATTERY_BROKEN;
        else if(VoltageBattery>18) state = BATTERY;
    }
   if((supply==false) && (battery == false))state = POWER_OFF;


  switch(state)
  { 
   
    case NORMAL:
        POWER_CHECK_DEBUG(" Normal"); 
        batteryCheckTime.status = START;

        if( batteryCheckTime.value >100)
        { 
            SUPPLY_VOLTAGE_IS_16_V
            batteryCheckTime.status = STOP;
        }
        else 
        {
          POWER_RELAY_ON;
          SUPPLY_VOLTAGE_IS_24_V
        }
       if( batteryCheckTime.value >120) 
       {
        fierCheckLock=false;
        batteryCheckTime.status = STOP;
        }

    break;

    case BATTERY:
      POWER_CHECK_DEBUG(" Battery");
    break; 

    case LOW_BATTERY:
       SUPPLY_VOLTAGE_IS_24_V
      POWER_CHECK_DEBUG(" LowBattery");

    break;

    case POWER_SUPPLY:
     SUPPLY_VOLTAGE_IS_24_V
     POWER_RELAY_OFF
     POWER_CHECK_DEBUG(" PowerSupply");
     if((supply==true) && (battery == true))state=NORMAL;

    break;

    case BATTERY_BROKEN:
     SUPPLY_VOLTAGE_IS_24_V
     POWER_RELAY_OFF
     POWER_CHECK_DEBUG(" Battery is Broken"); 
    break;

    case POWER_OFF:
     SUPPLY_VOLTAGE_IS_24_V
    POWER_CHECK_DEBUG("Power is OFF"); 
    break;

    

  } ;
  
 if( batteryCheckTime.value >110) batteryCheckTime.status = STOP;

}

void batteryCheck() {
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

    if (currentTime - batteryScanTime > 50) {
    
      //24v Supply
      if ((mux4Values[0] * 41) > 16) {  //power check power 24 to 16v
        //chose mult chanel .0
          digitalWrite(Sela, LOW);
          digitalWrite(Selb, LOW);
          digitalWrite(Selc, LOW);
        batteryScanTime = currentTime;
        digitalWrite(ChangeVolt, HIGH);//voltage is 16
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
        } while ((currentTime - batteryScanTime) > 21);
        //chose mul battery voltage read
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
        digitalWrite(ChangeVolt, LOW);//power go up 16 to 24
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

void Ledcontrol(status lineSituations[12]) {
  for (byte i = 0; i < 12; i++) {

    if ((lineSituations[i] == 1) || (lineSituations[i] == 4)) {  // Fault mode
      sr.set(ledErrorsPins[i], ledBlinker1);
      if (buzzerControl && !generalFault && !fireTrace)
        digitalWrite(MCUbuzz, ledBlinker1);
      else
        digitalWrite(MCUbuzz, LOW);

    } else if (lineSituations[i] == 3) {  // Fire mode
      sr.set(ledFirePins[i], ledBlinker1);
      sr.set(ledefiremode, LOW);
      if (fireTrace)
        digitalWrite(MCUbuzz, HIGH);
      else
        digitalWrite(MCUbuzz, LOW);
    } else {
      sr.set(ledErrorsPins[i], HIGH);
      sr.set(ledFirePins[i], HIGH);
      if ((batteryFail || powerFail || supplyFault) && !generalFault)
        digitalWrite(MCUbuzz, ledBlinker1);
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
    sr.set(ledebat, ledBlinker1);
    sr.set(ledemainpower, ledBlinker1);
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
  ledBlinkTime = currentTime;
  ledBlinker1 = !ledBlinker1;
}

// Function to update the mux position for analog readings
void updateMuxPosition() {
  if (muxPosition < 8) {
    if (muxPosition == 7) {
      muxPosition = 0;
      readAnalogs = true;
    } else {
      muxPosition++;
    }
  }
}

// Function to check and enable the beeperEnabled
void checkAndEnableBeeper() {
  if (enableBeeper()) {
    buzzerReadyTime = currentTime;
    beeperEnabled = true;
  }
}

// Function to check if the beeperEnabled should be enabled
bool enableBeeper() {
  return (buzzerControl && generalFault && !fireTrace && (currentTime - buzzerReadyTime > 300));
}

status processCurrentConditions(float current , float voltage,int numberLine){


    
  static status state;
  static limit Limit;
  static unsigned long repeatFireDetection = 0;
  static unsigned long repeat= 0;


  #define MINIMUM_LIMIT_OPEN_CIRCUIT 0
  #define MAXIMUM_LIMIT_OPEN_CIRCUIT 6
  #define MINIMUM_LIMIT_NORMAL 7
  #define MAXIMUM_LIMIT_NORMAL 15
  #define MINIMUM_LIMIT_FIER 16
  #define MAXIMUM_LIMIT_FIER 80
  #define MINIMUM_LIMIT_CURRENT_SHORT_CIRCUIT 80
  #define MAXIMUM_LIMIT_CURRENT_SHORT_CIRCUIT 10000
  #define MINIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT 0.80
  #define MAXIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT 0.80

  Limit.minimumOpenCircuit = MINIMUM_LIMIT_OPEN_CIRCUIT ;
  Limit.maximumOpenCircuit = MAXIMUM_LIMIT_OPEN_CIRCUIT ;
  Limit.minimumNormal = MINIMUM_LIMIT_NORMAL ;
  Limit.maximumNormal = MAXIMUM_LIMIT_NORMAL ;
  Limit.minimumFier= MINIMUM_LIMIT_FIER ;
  Limit.maximumFier= MAXIMUM_LIMIT_FIER ;
  Limit.minimumCurrentShortCircuit= MINIMUM_LIMIT_CURRENT_SHORT_CIRCUIT ;
  Limit.maximumCurrentShortCircuit= MAXIMUM_LIMIT_CURRENT_SHORT_CIRCUIT;
  Limit.minimumVoltageShortCircuit= MINIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT ;
  Limit.maximumVoltageShortCircuit= MAXIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT;


  if (( voltage <=Limit.minimumVoltageShortCircuit)||( current <Limit.minimumCurrentShortCircuit))
  {
    state = SHORT_CIRCUIT;
  }
  else if (( current >=Limit.minimumOpenCircuit)&&(current <=Limit.maximumOpenCircuit))
  {
    state = OPEN_CIRCUIT;
  }
  else  if ( (current >= Limit.minimumNormal)&&(current <= Limit.maximumNormal))
  {
    state = NORMAL;

  }
  else if (( current >=Limit.minimumFier)&&(current <=Limit.maximumFier))
  {
        fierTimer.status = START;
          repeat++;
        if (repeat>9)repeatFireDetection++;
          
        if ((fierTimer.value > MAXIMUM_TIME_FIER_DETECT) && (repeatFireDetection <= MINIMUM_REPEAT_FIER_DETECT) && (fierLouckBit == 0)) {
          repeatFireDetection = 0;
          repeat=0;
          fierTimer.value = 0;
          fierTimer.status = STOP;
          state = NORMAL;
      }
      if ((repeatFireDetection > LIMIT_REPEAT_FOR_FIER_DETECT) || (fierLouckBit == 1)) {
          fierLouckBit = 1;
          repeatFireDetection = 0;
          fierTimer.status = STOP;
          state =FIER;
        }


  }

  switch(state)
  {

    case OPEN_CIRCUIT:
      LINE_STATUS_DEBUG("OPEN_CIRCUIT");
    // return OPEN_CIRCUIT;
    break;

    case NORMAL:
      LINE_STATUS_DEBUG("NORMAL");
      //return NORMAL;
    break;

    case FIER:
        LINE_STATUS_DEBUG("FIER");
        return FIER;
    break;

    case SHORT_CIRCUIT:
    

        LINE_FIER_DEBUG("lineConditionTimer.value");
        LINE_FIER_DEBUG(lineConditionTimer.value);
        if (lineConditionTimer.value >=200)
        {
          lineON(numberLine);
          if (lineConditionTimer.value >=250)
          {
          lineConditionTimer.status=STOP;
          lineConditionTimer.value=0;
          lineOFF(numberLine);
          }
        }
        else{
          lineOFF(numberLine);
          lineConditionTimer.status=START;
        }
        return SHORT_CIRCUIT;
        
    break;

  }
        
          LINE_STATUS_DEBUG("\n");
          LINE_STATUS_DEBUG("line");
          LINE_STATUS_DEBUG(numberLine);
          LINE_STATUS_DEBUG(" , Voltage =");
          LINE_STATUS_DEBUG(voltage);
          LINE_STATUS_DEBUG(", Current=");
          LINE_STATUS_DEBUG(current);
          LINE_STATUS_DEBUG("STATE");
          LINE_STATUS_DEBUG(state);
          LINE_STATUS_DEBUG(" ");



  return state;

}











// // Function to process current conditions
// status processCurrentConditions(byte line) {

//    static int repeatFireDetection = 0;
//    static int repeat= 0;
  
//     delay(20);

//  if (line > 3)//extera lines(card 1 or 2 or bouth)
//  {
//     MINIMUM_REPEAT_FIER_DETECT = MINIMUM_REPEAT_FIER_DETECT_FOR_EXTERA_LINES;
//     LIMIT_REPEAT_FOR_FIER_DETECT =  LIMIT_REPEAT_FOR_FIER_DETECT_EXTERA_LINES;
//     MAXIMUM_TIME_FIER_DETECT = MAXIMUM_TIME_FIER_DETECT_FOR_EXTERA_LINES;
//  }
//  else{

//     MINIMUM_REPEAT_FIER_DETECT = MINIMUM_REPEA_FIER_DETECT_FOR_MAIN_LINES;
//     LIMIT_REPEAT_FOR_FIER_DETECT = LIMIT_REPEAT_FOR_FIER_DETECT_MAIN_LINES;
//     MAXIMUM_TIME_FIER_DETECT = MAXIMUM_TIME_FIER_DETECT_FOR_EXTERA_LINES;
//  }

//   if ((Timer.value > MAXIMUM_TIME_FIER_DETECT) && (repeatFireDetection <= MINIMUM_REPEAT_FIER_DETECT) && (fierLouckBit == 0)) {

//     LINE_FIER_DEBUG("\n");
//     LINE_FIER_DEBUG("=========================================================================");
//     LINE_FIER_DEBUG("\n");
//     LINE_FIER_DEBUG("============================RESET DEBUNCE================================");
//     LINE_FIER_DEBUG("\n");
//     LINE_FIER_DEBUG("=========================================================================");
//     LINE_FIER_DEBUG("\n");

//     repeatFireDetection = 0;
//     repeat=0;
//     Timer.value = 0;
//     Timer.status = STOP;
//   }

//   // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
//   // Process the current conditions for the line

//   if (firstSence[line] == 0) {

//     if (lineCurrent[line] < OPEN_THRESHOLD) {


//          LINE_STATUS_DEBUG("OPEN_CIRCUIT" );
//         return OPEN_CIRCUIT;

//     } else if ((lineCurrent[line] > OPEN_THRESHOLD) && (lineCurrent[line] < NORMAL_THRESHOLD)) {
//        LINE_STATUS_DEBUG(" NORMAL  ");
//       return NORMAL;
//     }
//   }

//   if ((lineCurrent[line] > NORMAL_THRESHOLD) && (lineCurrent[line] < FIRE_THRESHOLD) && (fierLouckBit == 0)&&(fierCheckLock == false)) {
//          Timer.status = START;
//         repeat++;
//         if (repeat>9){
//         repeatFireDetection++;
//          // LINE_FIER_DEBUG(">>>>>>>>>>>> Detect Fier <<<<<<<<<<<<<<<<<<");
//           LINE_FIER_DEBUG(repeatFireDetection);
//        }
//         //LINE_FIER_DEBUG(">>>>>>>>>>>> Detect Fier <<<<<<<<<<<<<<<<<<");
//        // mySerial.printf("\n repeatFireDetection= %d repeat= %d Timer value=%d",repeatFireDetection,repeat,Timer.value); 
//   }

//   if ((repeatFireDetection > LIMIT_REPEAT_FOR_FIER_DETECT) && ((lineCurrent[line] > NORMAL_THRESHOLD) && (lineCurrent[line] < FIRE_THRESHOLD)) || (fierLouckBit == 1)) {

//     fierLouckBit = 1;
//     repeatFireDetection = 0;
//     Timer.status = STOP;

//      LINE_FIER_DEBUG("\n");
//      LINE_FIER_DEBUG("<<<!!!!!!!!!!!!!>>> FIER <<<!!!!!!!!!!!!!!>>>");
//      LINE_FIER_DEBUG("\n");

//     // Handle fire detection conditions
//     fultSencetimer = 0;  // fire alarming
//     delay(55);

//     if (firstSence[line] == 1) {

//       digitalWrite(lineControlPins[line], LOW);
//       delay(55);
//     }
//     if (firstSence[line] == 2) {

//       digitalWrite(lineControlPins[line], HIGH);
//     }

//     if (firstSence[line] == 3) {

//       fireTrace = true;
//       fireFlag = true;
//       relayControl = false;
//       relayCustomOn = false;
//       sr.set(ledebuz, HIGH);
//       sr.set(ledesounder, HIGH);
//       return FIER;
//     }

//     firstSence[line] = firstSence[line] + 1;
//     delay(55);

//   } 
//   else
//    {
//     //repeatFireDetection=0;
//       if (firstSence[line] == 0)
//       return SHORT_CIRCUIT;
//   }
// }

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
  if (cardSituation == 1) {
    if (allZerosInRange(4, 7)) {
      CardPresentError = 1;
    } else {
      CardPresentError = 0;
    }
  }
  if (cardSituation == 2) {
    if (allZerosInRange(4, 7) && allZerosInRange(8, 11)) {
      CardPresentError = 3;
    } else if (allZerosInRange(8, 11)) {
      CardPresentError = 2;
    } else {
      CardPresentError = 0;
    }
  }
}


void distributionMuxValues() {

  switch (cardSituation) {
    case 0:
      lineVoltage[3] = mux1Values[0];
      lineCurrent[3] = mux1Values[3];
      lineCurrent[2] = mux1Values[2];
      lineVoltage[2] = mux1Values[1];
      lineCurrent[1] = mux1Values[4];
      lineVoltage[1] = mux1Values[6];
      lineVoltage[0] = mux1Values[5];
      lineCurrent[0] = mux1Values[7];
       
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


void readMux(byte add) {
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
    mux1Values[add] = ((3.3 / 1023.000) * analogRead(Analog1))*100;
    mux2Values[add] = (3.3 / 1023.000) * analogRead(Analog2)*100;
    mux3Values[add] = (3.3 / 1023.000) * analogRead(Analog3)*100;
    mux4Values[add] = (3.3 / 1023.000) * analogRead(Analog4)*100;
    // mux1Values[add] = analogRead(Analog1);
    // mux2Values[add] = analogRead(Analog2);
    // mux3Values[add] = analogRead(Analog3);
    // mux4Values[add] = analogRead(Analog4);
  }

  // Delay as needed

  delay(15);
  
 
 
}


void checkButtons() {

  if (digitalRead(But5) == 0) {  // Buzzer off
    if (currentTime - buttonPressTime > 10) {
      buttonPressTime = currentTime;
      generalFault = !generalFault;
    }
  }
  if (digitalRead(But4) == 0) {  // LED check
    sr.setAllLow();
    delay(550);
    byte initi = 0;
    initi = CardPresentError;
    CardPresentError = 0;
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
    CardPresentError = initi;
  }
  
 if((digitalRead(But3) == 0)&&(digitalRead(JUMPER) == 0)){  // All Line Reset

    fierLouckBit=0;
   /* digitalWrite(Line1, LOW);
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
    }*/
    for (byte i = 0; i < 12; i++) {
      lineCurrent[i] = 0;
      lineVoltage[i] = 0;
      //lineSituations[i] = 0;      // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
      
      firstSence[i] = 0;
      shortCircuitDetected[i] = 0;
    }

    limitTimeSC = 3;
    muxPosition = 0;
    // cardSituation  = 0;
    currentTime = 0;
    ledBlinkTime = 0;
    buttonPressTime = 0;
    shortCircuitTime = 0;
    buzzerReadyTime = 0;
    fultSencetimer = 0;
    ledBlinker1 = true;
    ledBlinker2 = true;
    buzzerControl = false;
    sounderLedStatus = false;
    supplyFault = false;
    batteryFail = false;
    powerFail = false;
    earthFail = false;
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
    if (cardSituation == 2) {
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
    } else if (cardSituation == 1) {
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
    readMux(muxPosition);
  }
  if (digitalRead(But2) == 0) {  // Alarm rely on
    if (currentTime - buttonPressTime > 13) {
      buttonPressTime = currentTime;
      relayControl = true;
      sr.set(ledesounder, HIGH);
      //  mySerial.println("alarm on");
      if (sounderLedStatus) {
        sounderLedStatus = !sounderLedStatus;
        relayCustomOn = false;
      }
    }
  }
  if (digitalRead(But1) == 0) {  // Alarm rely off

    if (currentTime - buttonPressTime > 13) {
      buttonPressTime = currentTime;
      relayControl = false;
      // mySerial.println("alarm off");
      if (fireFlag) {
        fireFlag = false;
        sr.set(ledesounder, LOW);
      }
      if (!sounderLedStatus)
        sounderLedStatus = !sounderLedStatus;
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
  pinMode(JUMPER, INPUT);
  pinMode(But1, INPUT);
  pinMode(But2, INPUT);
  pinMode(But3, INPUT);
  pinMode(But4, INPUT);
  pinMode(But5, INPUT);
  // Battery Charges Settings
  pinMode(Batcharges, OUTPUT);
  pinMode(ChangeVolt, OUTPUT);
 //digitalWrite(Batcharges, HIGH);
// digitalWrite(ChangeVolt, LOW);
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



void Update_IT_callback1(void) {  // 10hz
  currentTime++;
  fultSencetimer++;
 batteryCheckTime.update();
 Timer.update();
lineConditionTimer.update();
fierTimer.update();

  ledBlinker2 = !ledBlinker2;
  if (CardPresentError > 0) {
    if (CardPresentError == 1) {
      sr.set(ledErrorsPins[4], ledBlinker2);
      sr.set(ledErrorsPins[5], ledBlinker2);
      sr.set(ledErrorsPins[6], ledBlinker2);
      sr.set(ledErrorsPins[7], ledBlinker2);
    } else if (CardPresentError == 2) {
      sr.set(ledErrorsPins[8], ledBlinker2);
      sr.set(ledErrorsPins[9], ledBlinker2);
      sr.set(ledErrorsPins[10], ledBlinker2);
      sr.set(ledErrorsPins[11], ledBlinker2);
    } else if (CardPresentError == 3) {
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

void Update_IT_callback2(void) { 

 //for(int i=0;i<=10;i++) {flow[i].update();}
  readMuxPowerSupplyTimer.update();
  readMuxBatteryTimer.update();


}


void Relaycont() {
  digitalWrite(rel2, faultFlag);
  // mySerial.println(faultFlag);y
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

float readBattery(void){
  static char state;
  static float result;
  //  if( state==0) readMuxBatteryTimer.status=START;
  //  if (readMuxBatteryTimer.value > 50){
   delay(25);
    digitalWrite(Sela, HIGH);
    digitalWrite(Selb, HIGH);
    digitalWrite(Selc, LOW);
  //  } 
  //  if ((readMuxBatteryTimer.value>100)){
    delay(25);
     readMuxBatteryTimer.status=STOP;
     state=readMuxPowerSupplyTimer.status=START;
      result =(analogRead(Analog4)*0.0819672131)-1.3770491803;
     
    //  } 
     return result;
}
float readPowerSupply(void){

  static float result;
  static char state;
  delay(25);
  //  if( state==0) readMuxPowerSupplyTimer.status=START;
  //  if ((readMuxPowerSupplyTimer.value > 50)){ 

      digitalWrite(Sela, LOW);
      digitalWrite(Selb, LOW);
      digitalWrite(Selc, LOW);
    // } 
  //  if ((readMuxPowerSupplyTimer.value>100)){ 
  delay(25);
    readMuxPowerSupplyTimer.status=STOP;
    state=readMuxBatteryTimer.status=START;
    result =(analogRead(Analog4)*0.1020408163265)-26.857142857127;

    // } 
 return result;




}

