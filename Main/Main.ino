
#include <Main.h>
// Hardware Settings
 int myRep=0;
  void setup() {

    // Initialize GPIO pins
    GPIOInit();

    // Set analog read resolution to 12 Bits
    analogReadResolution(12);

    // Initialize SoftwareSerial for communication
    mySerial.begin(9600);

    // Initialize ShiftRegister and set all outputs to HIGH
    sr.setAllHigh();

    // Configure and start hardware timers for periodic tasks
    configureTimers();

   // Power relay trun on
    POWER_RELAY_ON

    // Set cardSituation based on card presence
    cardSituation= setCardSituation();

    // Initialize watchdog timer
    IWatchdog.begin(7000000);

    // Read initial values from analog mux
    readInitialMuxValues();
  }
  void(* resetFunc) (void) = 0;//declare reset function at address 0
  void loop() {
  
   mySerial.printf("\n Reped== %d",myRep);
   buzzer.Begin(buzzerActive,myRep);
 
    // Read values from analog mux
   readMux(muxPosition,mux);

    // Check power status
    powerStatus =checkPower( readBattery(mux.Values4[3]) , readPowerSupply(mux.Values4[0]) );

    // Check button inputs
    checkButtons(resetFier,buzzerActive);

    // Distribute mux values to corresponding lines based on cardSituation
    distributionMuxValues(cardSituation,mux);

    // Print debug information
    LINE_STATE_DEBUG("\n <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<+>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
   
    // Perform line status evaluation after The first time the board is turned on
    if (firstRepeat>12){

      for (i = 0; i < ((cardSituation + 1) * 4); i++){ 

        lineStatus[i] = evaluateLineStatus( lineCurrent[i] , lineVoltage[i] , readMainVoltage(mux.Values4[2]) , i );
      }
   }
    else{
      firstRepeat++;
    }

    // Handle card present errors
    handleCardPresentErrors(cardSituation);
    //test buzzer
    //buzzer.Repead(buzzerActive, 3,10000, 10000);
    // Toggle LED state based on time
    if (timeForLedBlink()) {
       ledBlinkTime = currentTime;
      LEDControl(lineStatus,powerStatus,readMainVoltage(mux.Values4[2]), toggleLedState(),resetFier,buzzerActive);
    }

    // Control relay
    Relaycont();

    // Reload watchdog timer
    IWatchdog.reload();

    // Update mux position
    updateMuxPosition(cardSituation);

    // Check and enable beeper 
    checkAndEnableBeeper();
      // if(buzzerActive)
      // {
      //  buzzer.Singel(100,1000);
      // }
    // buzzer.Repead(buzzerActive,3, 100, 1000);

   buzzer.Singel(buzzerActive,5,50,3000);
  }


// Function to check if it's time to toggle the LED
bool timeForLedBlink() {
  return (currentTime - ledBlinkTime) > 6;
}
// Function to check power state based on battery and power supply voltages
powerState checkPower(float VoltageBattery, float VoltagePowerSupply) {

  // Define constants for voltage thresholds
  #define MINIMUM_VOLTAGE_VALIDE 6
  #define EXIST_POWER_SUPPLY_THRESHOLD 10
  #define EXIST_BATTERY_THRESHOLD 10
  #define NORMAL_BATTERY_THRESHOLD 18
  #define LOW_BATTERY_THRESHOLD 17
  #define POWER_LOW_TIME 100
  #define CLEAR_TIME 150
  #define MAXIMUM_TIME 1500
  // Static variables to maintain state between function calls
  static bool battery= false;
  static bool supply= false;
  static float  outputVoltage;
  static powerState state;
  
  // Fiter VoltagePowerSupply and  VoltageBattery Values
  if (VoltagePowerSupply <= MINIMUM_VOLTAGE_VALIDE )VoltagePowerSupply=0;
  if (VoltageBattery <= MINIMUM_VOLTAGE_VALIDE )VoltageBattery=0;

  // Debug messages
  POWER_CHECK_DEBUG("\n\nVoltageBattery=");
  POWER_CHECK_DEBUG(VoltageBattery,5);
  POWER_CHECK_DEBUG(", VoltagePowerSupply=");
  POWER_CHECK_DEBUG(VoltagePowerSupply,5);
  POWER_CHECK_DEBUG(" state :");
  
  // Determine if power supply and battery are present based on thresholds
  supply=( VoltagePowerSupply > EXIST_POWER_SUPPLY_THRESHOLD )? true : false;
  battery=( VoltageBattery > EXIST_BATTERY_THRESHOLD )? true : false;

  // State transitions based on power supply and battery conditions
  if((supply==true) && (battery == true)&&(state!=POWER_SUPPLY)&&(state!=BATTERY_BROKEN)){

    // Check battery voltage and transition to appropriate state
    if (VoltageBattery < NORMAL_BATTERY_THRESHOLD) state = BATTERY_BROKEN;
    else if (VoltageBattery > NORMAL_BATTERY_THRESHOLD) state = NORMAL_POWER;
    
  }

  // If power supply is present but battery is not
  if((supply==true) && (battery == false))state = POWER_SUPPLY;

   // If power supply is not present but battery is
  if((supply==false) && (battery == true)){

    // Check battery voltage and transition to the appropriate state 
    if((VoltageBattery < NORMAL_BATTERY_THRESHOLD) && (VoltageBattery > LOW_BATTERY_THRESHOLD)) state = LOW_BATTERY;
    else if(VoltageBattery < LOW_BATTERY_THRESHOLD ) state = BATTERY_BROKEN;
    else if(VoltageBattery >  NORMAL_BATTERY_THRESHOLD) state = BATTERY;
    
  }
  // If neither power supply nor battery is present
  if((supply==false) && (battery == false))state = POWER_OFF;

  // Perform actions based on the current state
  switch(state)
  { 

    //Normal State: first  Set supply voltage level to 24V and  after  POWER_LOW_TIME Set supply voltage level to 16V for sense power state
    case NORMAL_POWER:
      POWER_CHECK_DEBUG(" Normal  >>>");
      // Start battery check timer
      batteryCheckTime.status = START;
      // If battery check timer exceeds a threshold, perform actions
      if( batteryCheckTime.value > POWER_LOW_TIME)
      { 
        outputVoltage=VoltagePowerSupply;
        SUPPLY_VOLTAGE_IS_16_V
        POWER_CHECK_DEBUG(" SUPPLY_VOLTAGE_LEVEL_IS_16_V"); 
      }
      else 
      {
          // Turn on power relay, set supply voltage level, and update output voltage
        POWER_RELAY_ON;
        POWER_CHECK_DEBUG(" SUPPLY_VOLTAGE_LEVEL_IS_24_V"); 
        SUPPLY_VOLTAGE_IS_24_V
        outputVoltage=VoltagePowerSupply;
      }
      // If battery check timer exceeds another threshold, stop battery check
      if( batteryCheckTime.value > CLEAR_TIME) 
      {
        fierCheckLock=false;
        batteryCheckTime.status = STOP;
      }
    break;

   //Battery State: Set output voltage to battery voltage 
    case BATTERY:
      POWER_CHECK_DEBUG(" Battery");
      outputVoltage=VoltageBattery;
    break; 

    //Low Battery State:  Set supply voltage level to 24V and update output voltage 
    case LOW_BATTERY:
      SUPPLY_VOLTAGE_IS_24_V
      POWER_CHECK_DEBUG(" LowBattery");
      outputVoltage=VoltageBattery;
    break;

    //Power Supply State: Set supply voltage level to 24V, turn off power relay, and update output voltage
    case POWER_SUPPLY:
      SUPPLY_VOLTAGE_IS_24_V
      POWER_RELAY_OFF
      POWER_CHECK_DEBUG(" PowerSupply");
       // If both supply and battery are present, transition to normal power state
      if((supply==true) && (battery == true))state=NORMAL_POWER;
      outputVoltage=VoltagePowerSupply;
    break;

    // Battery Broken State: Set supply voltage level to 24V, turn off power relay, and update output voltage
    case BATTERY_BROKEN:
      SUPPLY_VOLTAGE_IS_24_V
      POWER_RELAY_OFF
      POWER_CHECK_DEBUG(" Battery is Broken"); 
      outputVoltage=VoltagePowerSupply;
    break;

    // Power OFF State: Set supply voltage level to 24V and update output voltage
    case POWER_OFF:
      SUPPLY_VOLTAGE_IS_24_V
      POWER_CHECK_DEBUG("Power is OFF"); 
    break;


  };

  // Stop battery check after a certain time
  if( batteryCheckTime.value > MAXIMUM_TIME) batteryCheckTime.status = STOP;

  // Return the current power state
  return state;
}

// {
//   if((buzzerActive)&&(buzzerActive)){
//     BUZZER_ON
//     return true;
//   }
//   else{
//     BUZZER_OFF
//     return false;
//   }

// // ((buzzerActive) && (buzzerState))?(BUZZER_ON):(BUZZER_OFF); 
// }
// // void singelBuzzer() 
// {
//   BUZZER_ON 
//   delay(60);
//   BUZZER_OFF
// }
// Function to print voltage alert
void LEDControl(status lineStatus[12],powerState powerStatus,double mainVoltage, bool ledStatus,bool &resetFier,bool &buzzerActive) {
  static bool lockFier[12]={false};
  bool ledBlinker1 = ledStatus;
 for (byte i = 0; i < 12; i++) {
  
    if((lockFier[i]==true)&&(!resetFier)){
    sr.set(ledFirePins[i], ledBlinker1);
    buzzer.Active(buzzerActive,HIGH);
    }
    if ((lineStatus[i] == OPEN_CIRCUIT) || (lineStatus[i] == SHORT_CIRCUIT)) {  // Fault mode
      sr.set(ledErrorsPins[i], ledBlinker1);

    } else if (lineStatus[i] == FIER) {  // Fire mode
       fireTrace=true;
       lockFier[i]=true;
    } else {
      sr.set(ledErrorsPins[i], HIGH);
       sr.set(ledFirePins[i], HIGH);
     
    }

  }
  if ((generalFault && fireTrace) || (batteryFail || powerFail || supplyFault) && generalFault)
    sr.set(ledebuz, LOW);
  else
    sr.set(ledebuz, HIGH);
  if (supplyFault && !powerFail)
    sr.set(ledepower, LOW);
  else
    sr.set(ledepower, HIGH);

  if (powerStatus==POWER_SUPPLY)//powersup
    sr.set(ledebat, LOW);
  else
    sr.set(ledebat, HIGH);

  if (powerStatus==LOW_BATTERY) {//low power
    sr.set(ledebat, ledBlinker1);
    sr.set(ledemainpower, ledBlinker1);
   if(fireTrace==false) digitalWrite(MCUbuzz, ledBlinker1);
  }

  if (powerStatus==BATTERY)//battery
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
    // if (!generalFault)
    //   digitalWrite(MCUbuzz, relayOn);
  }
  sr.set(generalfault, !(supplyFault || batteryFail || powerFail || relayOn));
}

// Function to toggle the LED state
bool toggleLedState() {
  bool static ledBlinker1;
  return  ledBlinker1 = !ledBlinker1;
}
// Function to update the mux position for analog readings
void updateMuxPosition(char &cardSituation) {
  
  if (muxPosition< 8) {

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
  if ( (buzzerControl && generalFault && !fireTrace && (currentTime - buzzerReadyTime > 300))) {
    buzzerReadyTime = currentTime;
    beeperEnabled = true;
  }
}

// Function to evaluate the status of a line
status evaluateLineStatus(float current , float voltage,double supplyVoltage,int numberLine){

  static status state;
  static unsigned long repeatFireDetection[12]={0};
  static unsigned long repeat= 0;
  static bool shortCircuitLock[12]={false};
  static bool  fierDetectLock[12]={false};
  // Calculate Line Resistance
  float lineResistor= (supplyVoltage/current)*1000 ;

 // Debug information for line status evaluation
  LINE_STATUS_DEBUG("\n");
  LINE_STATUS_DEBUG("line");
  LINE_STATUS_DEBUG(numberLine+1);
  LINE_STATUS_DEBUG(", Current=");
  LINE_STATUS_DEBUG(current,2);
  LINE_SC_DEBUG(" Voltage =");
  LINE_SC_DEBUG(voltage,3);

  // Define constants for line status evaluation
  #define MINIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT 120
  #define SHORT_CIRCUIT_TIME 3000
  #define SHORT_CIRCUIT_LINE_ON_TIME  SHORT_CIRCUIT_TIME-200
  #define FIER_DETECTION_TIME 3500
  #define ACCEPTABLE_NUMBER_OF_REPEAT_FIER  46
  #define ACCEPTABLE_NUMBER_OF_REPEAT_FIER_EXTERA_LINES  3
  //============ change Moode ============//
  /*
  If defined "LINE_STATUS_BY_RESISTANCE", the line status reading is determined via 
  the line resistance, otherwise via the line current.
  */
  //=====================================//
  #define LINE_STATUS_BY_RESISTANCE;
  //=====================================//
 // Determine the line status based on the chosen mode
  #ifndef LINE_STATUS_BY_RESISTANCE
  // Determining the status of the line status with the line current
  state=handleLineStatusByCurrent(current,voltage,shortCircuitLock[numberLine]);
  #endif 

  #ifdef LINE_STATUS_BY_RESISTANCE
  // Determining the condition of the line with line resistance
  state=handleLineStatusByResistance(lineResistor,voltage,shortCircuitLock[numberLine]);
  LINE_STATUS_DEBUG(", MainV=");
  LINE_STATUS_DEBUG( supplyVoltage,2);
  #endif 

  // Switch based on the determined line status
  switch(state)
  {
    //Handle Open Circuit state
    case OPEN_CIRCUIT:
      LINE_STATE_DEBUG(" OPEN_CIRCUIT ");
      // If there is a previous short circuit, change state to SHORT_CIRCUIT, else turn the line ON
      if (shortCircuitLock[numberLine]== true)state =SHORT_CIRCUIT;
      else lineON(numberLine);
    break;

    //Handle Normal state
    case NORMAL:
      LINE_STATE_DEBUG(" NORMAL ");
    break;

    //Handle Fier state
    case FIER:
      LINE_STATE_DEBUG(" FIER ");
      //turn the line ON
      lineON(numberLine);
      return FIER;
    break;

    //Handle short Circuit state: when a short connection occurs, the line is turned off,
    // after a period of SHORT_CIRCUIT_TIME time equal to SHORT_CIRCUIT_LINE_ON_TIME, the line is turned on and The line status is checked again
    case SHORT_CIRCUIT:
      lineON(numberLine);
      shortCircuitLock[numberLine] = true;
      if(shortCircuitFlow[numberLine].Delay(SHORT_CIRCUIT_TIME) == false) lineOFF(numberLine); 
      if(shortCircuitFlow[numberLine].value>=SHORT_CIRCUIT_LINE_ON_TIME)  lineON(numberLine);
      LINE_STATE_DEBUG(" SHORT_CIRCUIT ");  
    break;

    //Handle Damage of line state
    case DAMAGED:
      LINE_STATE_DEBUG(" DAMAGED ");
      return DAMAGED; 
    break;

    //Handle Check Fier state:Turn the line ON, increment the repeatFireDetection[numberLine] counter, and manage FIER detection
    case CHECK:
      LINE_STATE_DEBUG(" CHECK ");
      lineON(numberLine);
      repeatFireDetection[numberLine]++;
      // Check the number of repeats and set the state accordingly
      if(fierFlow.Delay(FIER_DETECTION_TIME)||fierLouckBit[numberLine] )
      {
        if ((repeatFireDetection[numberLine] < ACCEPTABLE_NUMBER_OF_REPEAT_FIER) && fierLouckBit[numberLine] == false)state = NORMAL; 
        else state =FIER;
        fierLouckBit[numberLine] = true;
        repeatFireDetection[numberLine] = 0;
      }
      LINE_FIER_DEBUG(" repeatFireDetection[numberLine]=");
      LINE_FIER_DEBUG(repeatFireDetection[numberLine]);
      LINE_FIER_DEBUG(" fierTimer.value="); 
      LINE_FIER_DEBUG(fierFlow.value);

    break;

    
  }

  return state;
}
// Function to handle line status based on resistance
status handleLineStatusByResistance(double lineResistor,double voltage,bool shortCircuit){
  static status state;

  // Debug information: Log the line resistor value
  LINE_STATUS_DEBUG(", Resistor=");
  LINE_STATUS_DEBUG( lineResistor,4);

  // Define threshold values for different line statuses
  #define MINIMUM_LIMIT_RES_OPEN_CIRCUIT 0
  #define MAXIMUM_LIMIT_RES_OPEN_CIRCUIT 4289
  #define MINIMUM_LIMIT_RES_NORMAL 4289
  #define MAXIMUM_LIMIT_RES_NORMAL 1581
  #define MINIMUM_LIMIT_RES_FIER 1581
  #define MAXIMUM_LIMIT_RES_FIER 400
  #define MINIMUM_LIMIT_RES_CURRENT_SHORT_CIRCUIT 400
  #define V_R_IS_0 ( voltage == 0) && ( lineResistor == 0)
  #define OPEN_CIRCUIT_RESISTOR ((lineResistor >MAXIMUM_LIMIT_RES_OPEN_CIRCUIT)) 
  #define NORMAL_RESISTOR ( (lineResistor >=MAXIMUM_LIMIT_RES_NORMAL)&&(lineResistor < MINIMUM_LIMIT_RES_NORMAL))
  #define FIER_RESISTOR ( (lineResistor >=MAXIMUM_LIMIT_RES_FIER)&&(lineResistor < MINIMUM_LIMIT_RES_FIER))
  #define SHORT_CIRCUIT_RESISTOR (lineResistor < MINIMUM_LIMIT_RES_CURRENT_SHORT_CIRCUIT)

  // Ensure lineResistor and voltage are not extremely low
  if( lineResistor < 2 )lineResistor=0; 
  if( voltage < 50 )voltage=0;

  // Determine the line status based on resistance
  if ( (SHORT_CIRCUIT_RESISTOR)  && (lineResistor>0)) state = SHORT_CIRCUIT;
  else if ((V_R_IS_0||OPEN_CIRCUIT_RESISTOR) && (shortCircuit == false))state = OPEN_CIRCUIT;
  else if (NORMAL_RESISTOR) state = NORMAL;  
  if (FIER_RESISTOR)state=CHECK;  
  return state;
}
// Function to handle line status based on current
status handleLineStatusByCurrent(double current,double voltage,bool shortCircuit){
  static status state;
   // Define threshold values for different line statuses
  #define MINIMUM_LIMIT_OPEN_CIRCUIT 0
  #define MAXIMUM_LIMIT_OPEN_CIRCUIT 6
  #define MINIMUM_LIMIT_NORMAL 6
  #define MAXIMUM_LIMIT_NORMAL 20
  #define MINIMUM_LIMIT_FIER 20
  #define MAXIMUM_LIMIT_FIER 110
  #define MINIMUM_LIMIT_CURRENT_SHORT_CIRCUIT 110
  #define V_I_IS_0 ( voltage == 0) && ( current == 0)
  #define OPEN_CIRCUIT_CURRENT ( current >= MINIMUM_LIMIT_OPEN_CIRCUIT) && (current < MAXIMUM_LIMIT_OPEN_CIRCUIT) 
  #define NORMAL_CURRENT (current >= MINIMUM_LIMIT_NORMAL )&&(current < MINIMUM_LIMIT_FIER)
  #define FIER_CURRENT  (current >= MINIMUM_LIMIT_FIER )&&(current < MAXIMUM_LIMIT_FIER )
  #define SHORT_CIRCUIT_VOLTAGE (voltage >= MINIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT)
  #define SHORT_CIRCUIT_CURRENT (current >= MINIMUM_LIMIT_CURRENT_SHORT_CIRCUIT)

 // Ensure current and voltage are not extremely low
  if( current < 2 )current=0; 
  if( voltage < 50 )voltage=0;

  // Determine the line status based on current
  if ((SHORT_CIRCUIT_CURRENT) && (current>0)) state = SHORT_CIRCUIT;
  else if ((V_I_IS_0||OPEN_CIRCUIT_CURRENT) && (shortCircuit == false))state = OPEN_CIRCUIT;
  else if (NORMAL_CURRENT) state = NORMAL;  
  if (FIER_CURRENT) state=CHECK;
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
void handleCardPresentErrors( char &cardSituation) {
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

// Function to distribute values from multiple multiplexers to corresponding arrays
void distributionMuxValues(char cardSituation,Mux &mux) {

  // Switch based on the current card situation
  switch (cardSituation) {

    // Case 0:Maine Lines configuration
    case 0:
      // Assign values from mux1 to corresponding arrays
      lineVoltage[3] = mux.Values1[0];
      lineCurrent[3] = mux.Values1[3];
      lineCurrent[2] = mux.Values1[2];
      lineVoltage[2] = mux.Values1[1];
      lineCurrent[1] = mux.Values1[4];
      lineVoltage[1] = mux.Values1[6];
      lineVoltage[0] = mux.Values1[5];
      lineCurrent[0] = mux.Values1[7];
      break;

    // Case 1: Main lines & Single card configuration
    case 1:
      // Assign values from mux1 to corresponding arrays
      lineVoltage[3] = mux.Values1[0];
      lineVoltage[2] = mux.Values1[1];
      lineCurrent[2] = mux.Values1[2];
      lineCurrent[3] = mux.Values1[3];
      lineCurrent[1] = mux.Values1[4];
      lineVoltage[1] = mux.Values1[6];
      lineVoltage[0] = mux.Values1[5];
      lineCurrent[0] = mux.Values1[7];

      // Assign values from mux2 to corresponding arrays
      lineVoltage[4] = mux.Values2[0];
      lineCurrent[4] = mux.Values2[3];
      lineCurrent[5] = mux.Values2[1];
      lineVoltage[5] = mux.Values2[2];
      lineCurrent[6] = mux.Values2[6];
      lineVoltage[6] = mux.Values2[4];
      lineVoltage[7] = mux.Values2[5];
      lineCurrent[7] = mux.Values2[7];
      break;

    // Case 2: Two-card & main lines configuration
    case 2:
      // Assign values from mux1 to corresponding arrays
      lineVoltage[3] = mux.Values1[0];
      lineCurrent[3] = mux.Values1[3];
      lineCurrent[2] = mux.Values1[2];
      lineVoltage[2] = mux.Values1[1];
      lineCurrent[1] = mux.Values1[4];
      lineVoltage[1] = mux.Values1[6];
      lineVoltage[0] = mux.Values1[5];
      lineCurrent[0] = mux.Values1[7];

      // Assign values from mux2 to corresponding arrays
      lineVoltage[4] = mux.Values2[0];
      lineCurrent[4] = mux.Values2[3];
      lineCurrent[5] = mux.Values2[1];
      lineVoltage[5] = mux.Values2[2];
      lineCurrent[6] = mux.Values2[6];
      lineVoltage[6] = mux.Values2[4];
      lineVoltage[7] = mux.Values2[5];
      lineCurrent[7] = mux.Values2[7];

      // Assign values from mux3 to corresponding arrays
      lineVoltage[8] = mux.Values3[0];
      lineCurrent[8] = mux.Values3[3];
      lineCurrent[9] = mux.Values3[1];
      lineVoltage[9] = mux.Values3[2];
      lineCurrent[10] = mux.Values3[6];
      lineVoltage[10] = mux.Values3[4];
      lineVoltage[11] = mux.Values3[5];
      lineCurrent[11] = mux.Values3[7];
      break;
  }
}
// Function to read multiplexer from ADC's
void readMux(byte address, Mux &mux) {

  // Define constants
  #define ADC_RESOLUTION 4096
  #define VREF 3.3
  #define VOLTAGE_ATTENUATION 2.00 //The ADC voltage is halved by resistive division
  #define VOLTAGE_DROP_MUX 0.032 //Voltage drop on the multiplexer
  #define RSHANT  18.00 //Voltage drop on the multiplexer

  // Define control values for Sela, Selb, and Selc
  byte controlValues[3] = { LOW, LOW, LOW };

  // // Calculate control values based on the address (add)
  if (address & 0b001) controlValues[0] = HIGH;
  if (address & 0b010) controlValues[1] = HIGH;
  if (address & 0b100) controlValues[2] = HIGH;

  // Set the control values
  digitalWrite(Sela, controlValues[0]);
  digitalWrite(Selb, controlValues[1]);
  digitalWrite(Selc, controlValues[2]);
  // Delay as needed
    delay(5);
  // Read analog values and store them in the mux arrays
   for (int i = 0; i < 4; i++) {
    mux.Values1[address] = ((((analogRead(Analog1)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX)/RSHANT)*1000;
    mux.Values2[address] = ((((analogRead(Analog2)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX)/RSHANT)*1000;
    mux.Values3[address] = ((((analogRead(Analog3)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX)/RSHANT)*1000;
    mux.Values4[address] = ((analogRead(Analog4)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX;
  }

  // Delay as needed
  delay(5);

}

void checkButtons(bool &resetFier,bool &buzzerActive) {
static bool buzzerButtonFlag=false;
static bool resetButtonFlag=false;
static bool relayONButtonFlag=false;
static bool relayOFFButtonFlag=false;

#define PRESS_BUZZER_BUTTON   digitalRead(But5) == 0
#define PRESS_LED_CHECK_BUTTON    digitalRead(But4) == 0
#define PRESS_RESET_BUTTON    digitalRead(But3) == 0
#define CONNECTED_JUMPER    digitalRead(JUMPER) == 0
#define PRESS_RELEY_ON_BUTTON   digitalRead(But2) == 0
#define PRESS_RELEY_OFF_BUTTON    digitalRead(But1) == 0
#define BUZZER_ON_TIME 60
  if (PRESS_BUZZER_BUTTON) {  // Buzzer off
    buzzerButtonFlag =true;
  }
  if((! PRESS_BUZZER_BUTTON )&&( buzzerButtonFlag ==true)){
    buzzer.Singel(1,1,BUZZER_ON_TIME,0);
      buzzerButtonFlag =false; 
      buzzerActive = !buzzerActive;
    }

  if (PRESS_LED_CHECK_BUTTON) {  // LED check
   buzzer.Singel(1,1,BUZZER_ON_TIME,0);
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
    } while (PRESS_LED_CHECK_BUTTON);
    IWatchdog.reload();
    delay(550);
    IWatchdog.reload();
    sr.setAllHigh();
    CardPresentError = initi;
  }
  
  if( PRESS_RESET_BUTTON ){  // All Line Reset
   resetButtonFlag = true; 
  }
  if( (! PRESS_RESET_BUTTON) && (resetButtonFlag == true) )
  {
    resetButtonFlag = false;
   buzzer.Singel(1,1,BUZZER_ON_TIME,0);
    if(CONNECTED_JUMPER)
    {
     resetFunc(); //call reset
    }
  }

  if (PRESS_RELEY_ON_BUTTON) {  // Alarm rely on
    relayONButtonFlag=true; 
  }
  if((!PRESS_RELEY_ON_BUTTON)&&(relayONButtonFlag==true)){
    relayONButtonFlag=false;
    buzzer.Singel(1,1,BUZZER_ON_TIME,0);
    relayControl = true;
    sr.set(ledesounder, HIGH);
    if (sounderLedStatus) {
      sounderLedStatus = !sounderLedStatus;
      relayCustomOn = false;
    }
  }

  if (PRESS_RELEY_OFF_BUTTON) {  // Alarm rely off
    relayOFFButtonFlag=true;
  }
  if((!PRESS_RELEY_OFF_BUTTON) && (relayOFFButtonFlag==true) )
  {
    relayOFFButtonFlag=false;
    buzzer.Singel(1,1,BUZZER_ON_TIME,0);
    relayControl = false;
    if (fireFlag) {
      fireFlag = false;
      sr.set(ledesounder, LOW);
    }
    if (!sounderLedStatus)
      sounderLedStatus = !sounderLedStatus;
      relayCustomOn = true;
  }
}

void GPIOInit(void) 
{
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
  
 
  batteryCheckTime.update();

   
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

}

void Update_IT_callback2(void) { 
 
  for(int i=0;i<=12;i++) {
    shortCircuitFlow[i].update();}
    buzzer.flowDelayUpdate(); 
   buzzer.buzzerFlow.update();
    buzzer.buzzerRepeadFlow.update();
}

void Relaycont(){
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

float readBattery(float VADC){
  float batteryVoltage;
  //batteryVoltage=((mux4Values[3]/0.3225806451612903)*0.0819672131)-1.3770491803;
  batteryVoltage=(((VADC/2)*11.5)/4.7)*10*0.955119893;
  if((batteryVoltage>18)&&(batteryVoltage<20))batteryVoltage=batteryVoltage+0.2;
  if((batteryVoltage>=20)&&(batteryVoltage<22))batteryVoltage=batteryVoltage+0.5;
  if(batteryVoltage>=22)batteryVoltage=batteryVoltage+0.6;
  return batteryVoltage;
}

float readPowerSupply(float VADC){
  float powerSupplyVoltage;
  return powerSupplyVoltage=((VADC/0.3225806451612903)*0.1020408163265)-26.857142857127; 
}

double readMainVoltage(double VADC) {
 // Define constants
  const float R1=220;// The resistor connected to VCC
  const float R2=22;// The resistor connected to ground 
  const float VDiode=0.48; //220.363kΩ
  double voltage =((VADC)*(R1+R2)/R2 )+ VDiode;
  double a1 = 20.0;
  double b1 = 20.0;
  double a2 = 28.0;
  double b2 = 23.891;
  // Calculate the conversion factor
  double conversionFactor = (a2 - a1) / (b2 - b1);
  // Calculate the corresponding value of a
  voltage= a1 + conversionFactor * (voltage - b1);
  return voltage;
 }

// Function to configure timers for periodic tasks
void configureTimers(void){

    void Update_IT_callback1(void);
    void Update_IT_callback2(void);
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

}
// Function to set cardSituation based on card presence
char setCardSituation(void){
 char cardSituation =0;
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
  return cardSituation;


}

void readInitialMuxValues(void){

  digitalWrite(LEDerror, HIGH);
  digitalWrite(Sela, LOW);
  digitalWrite(Selb, LOW);
  digitalWrite(Selc, LOW);
  delay(25);
  mux.Values4[0] = ((3.3 / 4095.00) * analogRead(Analog4))*100;
  delay(15);
  digitalWrite(Sela, HIGH);
  digitalWrite(Selb, HIGH);
  digitalWrite(Selc, LOW);
  delay(25);
  mux.Values4[3] = ((3.3 / 4095.00) * analogRead(Analog4))*100;

  }
