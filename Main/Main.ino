
#include <Main.h>

SoftwareSerial mySerial(S1rx, S1tx);  // RX, TX
// Shiftregister setting
ShiftRegister74HC595<5> sr(PC6, PC7, PC13);

timerMS batteryCheckTime;
timerMS fierTimer;
flowDelay flow[12];
timerMS shortCircuitTimer[12];

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


  //////////////////////////////////////////////////////SUPPLY_VOLTAGE_IS_24_V
  POWER_RELAY_ON

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
       readMux(muxPosition);
       //batteryVoltage=mux4Values[3]*0.2243157656080861;
       //powerSupplyVoltage=(mux4Values[0]/0.3225806451612903)*0.040979697*1.303529646264329;
       batteryVoltage=((mux4Values[3]/0.3225806451612903)*0.0819672131)-1.3770491803;
       powerSupplyVoltage=((mux4Values[0]/0.3225806451612903)*0.1020408163265)-26.857142857127;
       voltage =checkPower(batteryVoltage ,powerSupplyVoltage );
      
     // checkButtons();

      distributionMuxValues();
   
  
  
     LINE_STATUS_DEBUG("\n <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<+>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
     
     if (firstRepeat>12)
     {
        for (i = 0; i < ((cardSituation + 1) * 4); i++) 
        {   
          lineStatus[i] = evaluateLineStatus(lineCurrent[i],lineVoltage[i],i,voltage);
        }
      }
      else
      {
          firstRepeat++;
      }
    handleThresholdFaults( ) ;
    handleSupplyAndpowerFailures();
    handleCardPresentErrors();
    if (timeForLedBlink()) {
      toggleLedState();
      Ledcontrol(lineStatus);
    }
    Relaycont();
    IWatchdog.reload();
  //   if (muxPosition==0){ 
 //voltage =checkPower(readBattery(),readPowerSupply());
  // digitalWrite(Sela, HIGH);
  // digitalWrite(Selb, HIGH);
  // digitalWrite(Selc,HIGH);
  //   }
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
      shortCircuitDetected[numberLine] = 0;
      limitTimeSC++;
      if (limitTimeSC > 55) limitTimeSC = 4;

    }

  
    
    if ((lineVoltage[numberLine] < SHORT_CIRCUIT_THRESHOLD) && (currentTime - shortCircuitTime>1 )) {
      LINE_STATUS_DEBUG("ShortCircuit line(detect from Current) &");
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

float checkPower(float VoltageBattery, float VoltagePowerSupply) {

  #define MINIMUM_VOLTAGE_VALIDE 6
  #define EXIST_POWER_SUPPLY_THRESHOLD 10
  #define EXIST_BATTERY_THRESHOLD 10
  #define NORMAL_BATTERY_THRESHOLD 18
  #define LOW_BATTERY_THRESHOLD 17
  static bool battery= false;
  static bool supply= false;
  static float  outputVoltage;

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

   //  // Fiter VoltagePowerSupply and  VoltageBattery Values
    if (VoltagePowerSupply <= MINIMUM_VOLTAGE_VALIDE )VoltagePowerSupply=0;
    if (VoltageBattery <= MINIMUM_VOLTAGE_VALIDE )VoltageBattery=0;
 

   //Debug
 
    POWER_CHECK_DEBUG("\n\nVoltageBattery=");
    POWER_CHECK_DEBUG(VoltageBattery,5);
    POWER_CHECK_DEBUG(", VoltagePowerSupply=");
    POWER_CHECK_DEBUG(VoltagePowerSupply,5);
    POWER_CHECK_DEBUG(" state :");

    supply=( VoltagePowerSupply > EXIST_POWER_SUPPLY_THRESHOLD )? true : false;
    battery=( VoltageBattery > EXIST_BATTERY_THRESHOLD )? true : false;

    if((supply==true) && (battery == true)&&(state!=POWER_SUPPLY)&&(state!=BATTERY_BROKEN)){

      if (VoltageBattery < NORMAL_BATTERY_THRESHOLD) state = BATTERY_BROKEN;
      else if (VoltageBattery > NORMAL_BATTERY_THRESHOLD) state = NORMAL;
      
    }
    if((supply==true) && (battery == false))state = POWER_SUPPLY;
    if((supply==false) && (battery == true)){
        
      if((VoltageBattery < NORMAL_BATTERY_THRESHOLD) && (VoltageBattery > LOW_BATTERY_THRESHOLD)) state = LOW_BATTERY;
      else if(VoltageBattery < LOW_BATTERY_THRESHOLD ) state = BATTERY_BROKEN;
      else if(VoltageBattery >  NORMAL_BATTERY_THRESHOLD) state = BATTERY;
      
    }
    if((supply==false) && (battery == false))state = POWER_OFF;


    switch(state)
    { 
    
      case NORMAL:
        POWER_CHECK_DEBUG(" Normal  >>>"); 
        batteryCheckTime.status = START;
        if( batteryCheckTime.value >100)
        { 
          outputVoltage=VoltagePowerSupply;
          SUPPLY_VOLTAGE_IS_16_V
          POWER_CHECK_DEBUG(" SUPPLY_VOLTAGE_LEVEL_IS_16_V"); 
        }
        else 
        {
          POWER_RELAY_ON;
          POWER_CHECK_DEBUG(" SUPPLY_VOLTAGE_LEVEL_IS_24_V"); 
          SUPPLY_VOLTAGE_IS_24_V
          outputVoltage=VoltagePowerSupply;
        }
        if( batteryCheckTime.value >150) 
        {
          fierCheckLock=false;
          batteryCheckTime.status = STOP;
        }
      break;

      case BATTERY:
        POWER_CHECK_DEBUG(" Battery");
        outputVoltage=VoltageBattery;
      break; 

      case LOW_BATTERY:
        SUPPLY_VOLTAGE_IS_24_V
        POWER_CHECK_DEBUG(" LowBattery");
        outputVoltage=VoltageBattery;
      break;

      case POWER_SUPPLY:
        SUPPLY_VOLTAGE_IS_24_V
        POWER_RELAY_OFF
        POWER_CHECK_DEBUG(" PowerSupply");
        if((supply==true) && (battery == true))state=NORMAL;
        outputVoltage=VoltagePowerSupply;
      break;

      case BATTERY_BROKEN:
        SUPPLY_VOLTAGE_IS_24_V
        POWER_RELAY_OFF
        POWER_CHECK_DEBUG(" Battery is Broken"); 
        outputVoltage=VoltagePowerSupply;
      break;

      case POWER_OFF:
        SUPPLY_VOLTAGE_IS_24_V
        POWER_CHECK_DEBUG("Power is OFF"); 
      break;


    } ;
    if( batteryCheckTime.value >1500) batteryCheckTime.status = STOP;
    return outputVoltage;
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

status evaluateLineStatus(float current , float voltage,int numberLine,float supplyVoltage){

  static status state;
  static limit Limit;
  static unsigned long repeatFireDetection=0;
  static unsigned long repeat= 0;
  static bool shortCircuitLock[12]={false};
  static bool  fierLock[12]={false};
  static char parity;

  #define MINIMUM_LIMIT_OPEN_CIRCUIT 0
  #define MAXIMUM_LIMIT_OPEN_CIRCUIT 6
  #define MINIMUM_LIMIT_NORMAL 6
  #define MAXIMUM_LIMIT_NORMAL 20
  #define MINIMUM_LIMIT_FIER 20
  #define MAXIMUM_LIMIT_FIER 90
  #define MINIMUM_LIMIT_CURRENT_SHORT_CIRCUIT 90
  #define MINIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT 110
  #define SHORT_CIRCUIT_TIME 3000
  #define SHORT_CIRCUIT_LINE_ON_TIME 200
  #define FIER_DETECTION_TIME 3500
  #define ACCEPTABLE_NUMBER_OF_REPEAT_FIER  30
  #define ACCEPTABLE_NUMBER_OF_REPEAT_FIER_EXTERA_LINES  3
  #define OPEN_CIRCUIT_CURRENT ( current >= MINIMUM_LIMIT_OPEN_CIRCUIT) && (current < MAXIMUM_LIMIT_OPEN_CIRCUIT) 
  #define NORMAL_CURRENT (current >= MINIMUM_LIMIT_NORMAL )&&(current < MINIMUM_LIMIT_FIER)
  #define FIER_CURRENT  (current >= MINIMUM_LIMIT_FIER )&&(current < MAXIMUM_LIMIT_FIER )
  #define SHORT_CIRCUIT_VOLTAGE (voltage > MINIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT)
  #define SHORT_CIRCUIT_CURRENT (current >= MINIMUM_LIMIT_CURRENT_SHORT_CIRCUIT)
  if( current < 2 )current=0; 
  if( voltage < 50 )voltage=0;

  LINE_SC_DEBUG("\n");
  LINE_SC_DEBUG("line");
  LINE_SC_DEBUG(numberLine+1);
  LINE_SC_DEBUG(" Voltage =");
  LINE_SC_DEBUG(voltage,3);
  LINE_SC_DEBUG(", Current=");
  LINE_SC_DEBUG(current,2);

  if ( (SHORT_CIRCUIT_VOLTAGE || SHORT_CIRCUIT_CURRENT ) && (current>0) ) state = SHORT_CIRCUIT;
  else 
  {

    if (( voltage == 0) && ( current == 0)&&(shortCircuitLock[numberLine] == false))state = OPEN_CIRCUIT;
    if ( OPEN_CIRCUIT_CURRENT && (shortCircuitLock[numberLine] == false)) state = OPEN_CIRCUIT;
    else if (NORMAL_CURRENT) state = NORMAL;
    else  if (FIER_CURRENT)
    {
      fierTimer.status = START;
      repeatFireDetection++;
      if ((fierTimer.value > FIER_DETECTION_TIME) && (repeatFireDetection < ACCEPTABLE_NUMBER_OF_REPEAT_FIER) && (fierLouckBit == 0)) 
      {  
        repeatFireDetection = 0;
        fierTimer.value = 0;
        fierTimer.status = STOP;
        state = NORMAL;
      } 
      if ((fierTimer.value > FIER_DETECTION_TIME) && (repeatFireDetection >= ACCEPTABLE_NUMBER_OF_REPEAT_FIER)  || (fierLouckBit == 1))
      {
        fierLouckBit = 1;
        repeatFireDetection = 0;
        fierTimer.status = STOP;
        state =FIER;
        fireTrace=true;
      }
      if (fierTimer.value > FIER_DETECTION_TIME)
      {    
        repeatFireDetection = 0;
        fierTimer.status = STOP;
      }
      LINE_FIER_DEBUG(" repeatFireDetection=");
      LINE_FIER_DEBUG(repeatFireDetection);
      LINE_FIER_DEBUG(" fierTimer.value="); 
      LINE_FIER_DEBUG(fierTimer.value);
    }
    else{
       state = SHORT_CIRCUIT;
    }
            
  }

//lineON(numberLine); 
  switch(state)
  {

    case OPEN_CIRCUIT:
      LINE_STATUS_DEBUG(" OPEN_CIRCUIT ");
      if (shortCircuitLock[numberLine]== true)state =SHORT_CIRCUIT;
      else lineON(numberLine);
    break;

    case NORMAL:
      LINE_STATUS_DEBUG(" NORMAL ");
    break;

    case FIER:
      LINE_STATUS_DEBUG(" FIER ");
      fierLock[numberLine]=true;
      return FIER;
    break;

    case SHORT_CIRCUIT:
      shortCircuitLock[numberLine] = true;
      if(flow[numberLine].Delay(SHORT_CIRCUIT_TIME) == false) {lineOFF(numberLine);}
      if(flow[numberLine].value>= SHORT_CIRCUIT_TIME-SHORT_CIRCUIT_LINE_ON_TIME )lineON(numberLine);
      LINE_STATUS_DEBUG(" SHORT_CIRCUIT ");  
    break;

    case DAMAGED:
      LINE_STATUS_DEBUG(" DAMAGED ");
      return DAMAGED; 
    break;
  }

  return state;

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
      lineVoltage[2] = mux1Values[1];
      lineCurrent[2] = mux1Values[2];
      lineCurrent[3] = mux1Values[3];
      lineCurrent[1] = mux1Values[4];
      lineVoltage[1] = mux1Values[6];
      lineVoltage[0] = mux1Values[5];
      lineCurrent[0] = mux1Values[7];

      // lineVoltage[3-3+4] = mux2Values[0];
      // lineCurrent[3-2+4] = mux2Values[1];
      // lineVoltage[3-2+4] =  mux2Values[2];
      // lineCurrent[3-3+4] =  mux2Values[3];
      // lineVoltage[3-1+4] =  mux2Values[4];
      // lineVoltage[3-0+4] =  mux2Values[5];
      // lineCurrent[3-1+4] =  mux2Values[6];
      // lineCurrent[3-0+4] =  mux2Values[7];
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
   delay(5);
    // Read analog values and store them in the mux arrays
    for (int i = 0; i < 4; i++) {
      mux1Values[add] = ((3.3 / 1023.000) * analogRead(Analog1))*100*2;
      mux2Values[add] = (3.3 / 1023.000) * analogRead(Analog2)*100*2;
      mux3Values[add] = (3.3 / 1023.000) * analogRead(Analog3)*100*2;
      mux4Values[add] =  (3.3 / 1023.000) * analogRead(Analog4)*100;

    }
  
    // Delay as needed

   delay(5);
  
 
 
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
  
  for(int i=0;i<=12;i++) {
    shortCircuitTimer[i].update();
    flow[i].update();}

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



