
#include <Main.h>
// Hardware Settings
  void setup() {



    // Initialize GPIO pins
    GPIOInit();

    // Set analog read resolution to 12 Bits
    analogReadResolution(12);

    // Initialize SoftwareSerial for communication
    mySerial.begin(9600);

    // Initialize ShiftRegister and set all outputs to HIGH
    shiftRegister.setAllHigh();

    // Configure and start hardware timers for periodic tasks
    configureTimers();

   // Power relay trun on
    POWER_RELAY_ON

    // Set cardSituation based on card presence
    cardSituation= setCardSituation();

    // Initialize watchdog timer
    IWatchdog.begin(5000000);

    // Read initial values from analog mux
    readInitialMuxValues();
  }
  void loop() { 
   struct outputParametrs{
    powerState *PowerStatus;
    status *LineStatus;
    ButtonState *buttonStatus;
    double MainVoltage=0.0;
    bool MainVoltageState=false;
    bool AlatrState=false;
    bool fireTrace=false;
   }; 
   outputParametrs parametr;

    IWatchdog.reload();

    readMux(muxPosition,mux);
 
    LINE_STATE_DEBUG("\n <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<+>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    // Check power status
    parametr.PowerStatus =checkPower(
    readBattery(MUX_BATTERY_VOLTAGE) ,
    readPowerSupply(MUX_POWER_SUPPLY_VOLTAGE) 
    );
    
    // Check button inputs
    parametr.buttonStatus=checkButtons();

    // Distribute mux values to corresponding lines based on cardSituation
     distributionMuxValues(cardSituation,mux);
    
    // Print debug information
    LINE_STATE_DEBUG("\n <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<+>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    // Perform line status evaluation after The first time the board is turned on
    if (firstRepeat>MAXIMUM_REPEAD_TURN_ON){

      for (lineNumber = 0; lineNumber < ((cardSituation + 1) * 4); lineNumber++){ 

         //Check and determine the condition of the lines
         lineStatus[lineNumber] = evaluateLineStatus( 
                                                      lineCurrent[lineNumber] ,// line current value
                                                      lineVoltage[lineNumber] ,// line voltage value 
                                                      readMainVoltage(MUX_MAIN_VOLTAGE),// the Main voltage value
                                                      lineNumber //The number of the line being checked
          );
    
        if ( lineStatus[lineNumber] == FIER) parametr.fireTrace=true;
      }
   }

  else{
    firstRepeat++;
  }
 
   parametr.LineStatus=lineStatus;
   parametr.MainVoltage=readMainVoltage(MUX_MAIN_VOLTAGE);
   parametr.MainVoltageState=mainVoltageState(parametr.MainVoltage);
   parametr.AlatrState=readOutputsAlart(MUX_VOLTAGE_ALART_1,MUX_VOLTAGE_ALART_2);

    Output::LEDManager(
            parametr.LineStatus,//line status
            *parametr.PowerStatus,//power state (battery and power suply)
            parametr.buttonStatus,//botton status
            parametr.MainVoltageState,//read main voltage value
            parametr.AlatrState,//read state output Alart
            readEarth(MUX_EARTH)//read earth value
      );
    
    Output::BuzzerManager(
            parametr.buttonStatus,//botton status
            newEvent( parametr.LineStatus,
                      parametr.PowerStatus,
                      parametr.MainVoltageState,
                      parametr.AlatrState
                    ),
            parametr.fireTrace
      );

    Output::RelayManager( 
            parametr.fireTrace,
            parametr.buttonStatus,  
            newEvent( parametr.LineStatus,
                      parametr.PowerStatus,
                      parametr.MainVoltageState,
                      parametr.AlatrState
                  )
      );
    // Handle card present errors
    handleCardPresentErrors(cardSituation);
    // Reload watchdog timer
    IWatchdog.reload();
    // Update mux position
    updateMuxPosition(cardSituation);
 

  }
bool mainVoltageState(double mainVoltage)
{
  #define MINIMUM_VOLTAGE_MAIN 20
  return( mainVoltage>=MINIMUM_VOLTAGE_MAIN);
}


// Function to check power state based on battery and power supply voltages
powerState* checkPower(float VoltageBattery, float VoltagePowerSupply) {

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
  return &state;
}

void Output::LEDManager(status lineStatus[12],powerState powerStatus,ButtonState *buttonStatus, bool mainVoltageState,bool outputAlart,bool existenceEarth){
  #define BLINK_LEDS_ON_TIME 300
  #define BLINK_LEDS_OFF_TIME 300
  #define FAST_BLINK_LEDS_ON_TIME 100
  #define FAST_BLINK_LEDS_OFF_TIME 100
  LEDs LED;
  static bool fireTrace[12]={false};
  

    LEDWarning.turnOn(LED.ALL_CONDITION);//The fire control panel is power supplied
    digitalWrite(LED.SYSTEM, HIGH);//System fault 

    for (char i = 0;i < 12; i++) {
      if ((lineStatus[i] == OPEN_CIRCUIT) || (lineStatus[i] == SHORT_CIRCUIT)) 
       LEDWarning.blinkCustum(LED.WARNING[i],BLINK_LEDS_ON_TIME,BLINK_LEDS_OFF_TIME);

      if (lineStatus[i] == FIER) fireTrace[i]=true;
    
      if(fireTrace[i]==true){
        LEDFier.turnOn(LED.FIER[i]);
        LEDFier.turnOn(LED.FIER_OUTBREAK);
      }
      else if (lineStatus[i] == CHECK){
       LEDFier.blinkCustum(LED.FIER[i], FAST_BLINK_LEDS_ON_TIME,FAST_BLINK_LEDS_OFF_TIME);
      }
      else if((fireTrace[i]==false)&&(lineStatus[i] != CHECK))
      {
        LEDFier.turnOff(LED.FIER[i]);
      }
    
        if((powerStatus!=NORMAL_POWER)||(lineStatus[i]!=NORMAL)||(!mainVoltageState)){
         LEDWarning.turnOn(LED.GENERAL);
        }
        else
        {
          LEDWarning.turnOff(LED.GENERAL);
        } 
       
  }

  if(powerStatus==POWER_SUPPLY) LEDPower.turnOn(LED.BATTERY);
  else if(powerStatus==LOW_BATTERY) LEDWarning.blinkCustum(LED.BATTERY,BLINK_LEDS_ON_TIME,BLINK_LEDS_OFF_TIME);
  else LEDPower.turnOff(LED.BATTERY);



  ((powerStatus==BATTERY)||(powerStatus==LOW_BATTERY))? 
  LEDPower.turnOn(LED.POWER): 
  LEDPower.turnOff(LED.POWER);

  (mainVoltageState)? 
  LEDPower.turnOff(LED.MAIN_VOLTAGE): 
  LEDPower.turnOn(LED.MAIN_VOLTAGE);

  (outputAlart)? 
   LEDWarning.turnOff(LED.MANITOR_ALARM):
   LEDWarning.blinkCustum(LED.MANITOR_ALARM,BLINK_LEDS_ON_TIME,BLINK_LEDS_OFF_TIME);

  (buttonStatus->ALARM_RELAY)?
   LEDWarning.turnOn(LED.ALARM): 
   LEDWarning.turnOff(LED.ALARM);

  (buttonStatus->BUZZER)?
  LEDWarning.turnOn(LED.BUZZER): 
  LEDWarning.turnOff(LED.BUZZER);

  if(buttonStatus->LED_CHECK){
        //It stays here until Watchdogs resets
        while(1){
         digitalWrite(LED.SYSTEM, HIGH);
         LEDFier.turnOnArry(LED.WARNING,12);
         LEDFier.turnOnArry(LED.FIER,12);
         LEDFier.turnOnArry(LED.SINGEL_LEDS,10);
      }
  }

}

void Output::RelayManager(bool fierTrack,ButtonState *buttonStatus,eventStatus newEvent){

  if(newEvent!=NormalEvent)
  digitalWrite(rel2, HIGH);
  else digitalWrite(rel2, LOW);
  // mySerial.println(faultFlag);y
  if ((fierTrack) &&(!buttonStatus->ALARM_RELAY)) {
    digitalWrite(rel1, HIGH);
    digitalWrite(relo1, HIGH);
    digitalWrite(relo2, HIGH);
  } else 
    {
      if (buttonStatus->ALARM_RELAY) {
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

eventStatus newEvent(status *lineStatus,powerState *powerStatus,bool mainVoltageState,bool outputAlartState)
{   

    static event last;
    static eventStatus newEvent;
    bool event=false;
    static  bool lineNormalLatch=false;

   #define lineNewEvent(numberLine)  last.LineStatus[numberLine]!=lineStatus[numberLine]
   #define powerNewEvent             last.PowerState!=*powerStatus
   #define mainVoltageNewEvent       last.MainVoltageState!=mainVoltageState
   #define outoutAlartNewEvent       last.OutputAlartState!=outputAlartState
   #define unormalEvent(numberLine)  (lineStatus[numberLine]!=NORMAL)&&(!mainVoltageNewEvent)&&(*powerStatus!=NORMAL_POWER)&&(!outputAlartState)             
       
       for(char numberLine=0; numberLine<=11;numberLine++){

         if(unormalEvent(numberLine))lineNormalLatch=true;
         if((lineNewEvent(numberLine))||(powerNewEvent)||(mainVoltageNewEvent)||(outoutAlartNewEvent))
          {
            last.LineStatus[numberLine]= lineStatus[numberLine];
            last.MainVoltageState=mainVoltageState;
            last.PowerState =*powerStatus;
            last.OutputAlartState=outputAlartState;

            event=true;
          }
         }
          if(lineNormalLatch==false)return NormalEvent;
          if(event==true)
          {
            event=false;
            return HappenedAgain;
          }
          else
          {
            return Happened;
          }
  
}

void Output::BuzzerManager(ButtonState  *buttonStatus,eventStatus newEvent,bool fierTrack){
    #define  BLINK_BUZZER_ON_TIME 200
    #define  BLINK_BUZZER_OFF_TIME 1000
    static eventStatus event;
    static eventStatus lastEvent;
    event =newEvent;
    if(event==HappenedAgain){
    buttonStatus->BUZZER = false;
    lastEvent=HappenedAgain;
    }
    if(lastEvent==HappenedAgain) buzzer.SingelOn(BLINK_BUZZER_ON_TIME, BLINK_BUZZER_OFF_TIME); 
    if((event==NormalEvent)&&!(fierTrack))buzzer.TurnOff();
    buzzer.Begin(!buttonStatus->BUZZER);
    if(fierTrack)buzzer.TurnOn(!buttonStatus->BUZZER); 
}


// Function to update the mux position for analog readings
void updateMuxPosition(char &cardSituation) {
  
  if (muxPosition< 8) {

      if (muxPosition == 7) {
        muxPosition = 0;

      } else {
      muxPosition++;
      }
  }
}

// Function to evaluate the status of a line
status evaluateLineStatus(float current , float voltage,double supplyVoltage,int numberLine){

  static status state;
  static unsigned long repeatFireDetection[12]={0};
  static unsigned long repeat= 0;
  static bool shortCircuitLock[12]={false};
  static bool  fierDetectLock[12]={false};
  int fierLouckBit[12] ={0};
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
      else LINE_ON(numberLine);
    break;

    //Handle Normal state
    case NORMAL:
      LINE_STATE_DEBUG(" NORMAL ");
    break;

    //Handle Fier state
    case FIER:
      LINE_STATE_DEBUG(" FIER ");
      //turn the line ON
      LINE_ON(numberLine);
      return FIER;
    break;

    //Handle short Circuit state: when a short connection occurs, the line is turned off,
    // after a period of SHORT_CIRCUIT_TIME time equal to SHORT_CIRCUIT_LINE_ON_TIME, the line is turned on and The line status is checked again
    case SHORT_CIRCUIT:
      LINE_ON(numberLine);
      shortCircuitLock[numberLine] = true;
      if(shortCircuitFlow[numberLine].Delay(SHORT_CIRCUIT_TIME) == false) LINE_OFF(numberLine); 
      if(shortCircuitFlow[numberLine].value>=SHORT_CIRCUIT_LINE_ON_TIME)  LINE_ON(numberLine);
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
      LINE_ON(numberLine);
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

  // Define control values for analogSelectionA, analogSelectionB, and analogSelectionC
  byte controlValues[3] = { LOW, LOW, LOW };

  // // Calculate control values based on the address (add)
  if (address & 0b001) controlValues[0] = HIGH;
  if (address & 0b010) controlValues[1] = HIGH;
  if (address & 0b100) controlValues[2] = HIGH;

  // Set the control values
  digitalWrite(analogSelectionA, controlValues[0]);
  digitalWrite(analogSelectionB, controlValues[1]);
  digitalWrite(analogSelectionC, controlValues[2]);
  // Delay as needed
    delay(10);
  // Read analog values and store them in the mux arrays
   for (int i = 0; i < 4; i++) {
    mux.Values1[address] = ((((analogRead(Analog1)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX)/RSHANT)*1000;
    mux.Values2[address] = ((((analogRead(Analog2)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX)/RSHANT)*1000;
    mux.Values3[address] = ((((analogRead(Analog3)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX)/RSHANT)*1000;
    mux.Values4[address] = ((analogRead(Analog4)*VREF/ADC_RESOLUTION)*VOLTAGE_ATTENUATION)+VOLTAGE_DROP_MUX;
  }

  // Delay as needed
  delay(10);

}

ButtonState* checkButtons() {
  static ButtonState   buttonState ;
  static bool buzzerButtonFlag=false;
  static bool resetButtonFlag=false;
  static bool relayONButtonFlag=false;
  static bool relayOFFButtonFlag=false;
  static bool checkLEDsFlag=false;
  #define PRESS_BUZZER_BUTTON   digitalRead(But5) == 0
  #define PRESS_LED_CHECK_BUTTON    digitalRead(But4) == 0
  #define PRESS_RESET_BUTTON    digitalRead(But3) == 0
  #define CONNECTED_JUMPER    digitalRead(JUMPER) == 0
  #define PRESS_RELEY_ON_BUTTON   digitalRead(But2) == 0
  #define PRESS_RELEY_OFF_BUTTON    digitalRead(But1) == 0
  #define BUZZER_ON_TIME 60

  // Buzzer on/off
  if (PRESS_BUZZER_BUTTON)buzzerButtonFlag =true;
  if((! PRESS_BUZZER_BUTTON )&&( buzzerButtonFlag ==true)){
      buzzer.localBib();
      buzzerButtonFlag =false; 
      buttonState.BUZZER= (!buttonState.BUZZER ); 
  }

  // LED check
  if (PRESS_LED_CHECK_BUTTON) checkLEDsFlag=true;
  if ((!PRESS_LED_CHECK_BUTTON)&&(checkLEDsFlag==true)) {  
      buzzer.localBib();
      buttonState.LED_CHECK=true; 
  }

  // All Line Reset
  if(PRESS_RESET_BUTTON)resetButtonFlag = true;  
  if( (! PRESS_RESET_BUTTON) && (resetButtonFlag == true) ){ 
    buzzer.localBib();
    resetButtonFlag = false;
    if(CONNECTED_JUMPER){
     buttonState.RESET=true;
     resetFunc(); //call reset
    }
  }

  // Alarm rely on
  if (PRESS_RELEY_ON_BUTTON) relayONButtonFlag=true; 
  if((!PRESS_RELEY_ON_BUTTON)&&(relayONButtonFlag==true)){
      buzzer.localBib();
      buttonState.ALARM_RELAY=true;
      relayONButtonFlag=false;
  }

  // Alarm rely off
  if (PRESS_RELEY_OFF_BUTTON)relayOFFButtonFlag=true;
  if((!PRESS_RELEY_OFF_BUTTON) && (relayOFFButtonFlag==true) ) {
    buzzer.localBib();
    buttonState.ALARM_RELAY=false;
    relayOFFButtonFlag=false;
  }
  return &buttonState;
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
    pinMode(BATTERY_CHARGER_PIN, OUTPUT);
    pinMode(CHANGE_VOLTAGE, OUTPUT);
  //digitalWrite(Batcharges, HIGH);
  // digitalWrite(ChangeVolt, LOW);
    // Errors Settings
    pinMode(LEDerror, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LEDerror, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    // Analog settings
    pinMode(analogSelectionA, OUTPUT);
    pinMode(analogSelectionB, OUTPUT);
    pinMode(analogSelectionC, OUTPUT);
    digitalWrite(analogSelectionA, LOW);
    digitalWrite(analogSelectionB, LOW);
    digitalWrite(analogSelectionC, LOW);
    pinMode(Analog1, INPUT_ANALOG);
    pinMode(Analog2, INPUT_ANALOG);
    pinMode(Analog3, INPUT_ANALOG);
    pinMode(Analog4, INPUT_ANALOG);
    pinMode(LEDerror, OUTPUT);
}

void Update_IT_callback1(void) {  // 10hz

  
  buttonFlow.update();
  batteryCheckTime.update();

   
 


}

void Update_IT_callback2(void) { 
 
  for(int i=0;i<=12;i++) {
    shortCircuitFlow[i].update();
    ledBlinkerFlow[i].update();
    }
    fierFlow.update();
    buzzer.buzzerFlow.update();
    buzzer.buzzerRepeadFlow.update();

    
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
//This voltage is always present and is provided through power, battery, or power supply connected to the connector
double readMainVoltage(double VADC) {
 // Define constants
  const float R1=220;// The resistor connected to VCC
  const float R2=22;// The resistor connected to ground 
  const float VDiode=0.48; //220.363kΩ
  const float offset=1.6; //220.363kΩ
  double voltage =(((VADC)*(R1+R2)/R2 )+ VDiode);
  double X1= 18.7;
  double Y1 = 18.7;
  double Y2 = 27.1;
  double X2 = 26.3;

  // // Calculate the conversion factor
  double conversionFactor =((Y2-Y1)/(X2-X1));
  // Calculate the corresponding value of a
  voltage= conversionFactor*(voltage+offset-X1)+Y1;
   POWER_CHECK_DEBUG ("\n readMainVoltage=");
   POWER_CHECK_DEBUG(voltage);
  return voltage;
 }

bool readOutputsAlart(float outputVoltage1,float outputVoltage2)
{
  #define  VOLTAGE_ALART_NORMAL 1.17
  #define  MINIMUM_VOLTAGE_ALART 0.5

  POWER_CHECK_DEBUG ("\n VOUT ALART1=");
  POWER_CHECK_DEBUG( outputVoltage1);
  POWER_CHECK_DEBUG ("\n VOUT ALART2=");
  POWER_CHECK_DEBUG(outputVoltage2);
  return((outputVoltage1<MINIMUM_VOLTAGE_ALART)||(outputVoltage2<MINIMUM_VOLTAGE_ALART))?true:false;
}
bool readEarth(float earthVoltage)
{
POWER_CHECK_DEBUG("\n earthVoltage");
POWER_CHECK_DEBUG(earthVoltage);
return (earthVoltage<0);
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
// void readOutputsAlart(float ADCOutput1,float ADCOutput)
// {



// }

void readInitialMuxValues(void){

  digitalWrite(LEDerror, HIGH);
  digitalWrite(analogSelectionA, LOW);
  digitalWrite(analogSelectionB, LOW);
  digitalWrite(analogSelectionC, LOW);
  delay(25);
  mux.Values4[0] = ((3.3 / 4095.00) * analogRead(Analog4))*100;
  delay(15);
  digitalWrite(analogSelectionA, HIGH);
  digitalWrite(analogSelectionB, HIGH);
  digitalWrite(analogSelectionC, LOW);
  delay(25);
  mux.Values4[3] = ((3.3 / 4095.00) * analogRead(Analog4))*100;

  }
