
# Table of Contents

- [ Fire and Power Distribution Control System](#fire-and-power-distribution-control-system)
  - [Overview](#overview)
  - [Functions](#functions)
  - [ Main Loop ](#9-main-loop)
  - [Usage](#usage)


- [Power State Check Documentation](#power-state-check-documentation)
  - [`checkPower` Function Documentation]( #power-state-check-documentation)
- [Line Status Evaluation Documentation](#line-status-evaluation-documentation)
  - [`evaluateLineStatus` Function Documentation](#line-status-evaluation-documentation)
  - [`handleLineStatusByResistance` Function Documentation](#handle-line-status-by-resistance-documentation)
  - [`handleLineStatusByCurrent` Function Documentation](#handle-line-status-by-current-documentation)
- [Read Multiplexer (Mux) Function Documentation](#read-multiplexer-mux-function-documentation)
  - [`readMux` Function Documentation](#read-multiplexer-mux-function-documentation)
  - [`Distribute Mux Values` Function Documentation](#distribute-mux-values-function-documentation)
  - [`Update Mux Position` Function Documentation](#update-mux-position-function-documentation)
- [Card  Handling Functions](#set-card-situation-function-documentation)
  - [`Set Card Situation` Function Documentation](#set-card-situation-function-documentation)
  - [`allZerosInRange` Function Documentation](#all-zeros-in-range-check-function-documentation)
  - [`handleCardPresentErrors` Function Documentation](#handle-card-present-errors-function-documentation)
- [Timer and Interrupt Callback Functions and classes](#configure-timers-function-documentation)
  - [`configureTimers` Function Documentation](#configure-timers-function-documentation)
  - [`Update_IT_callback1` Function Documentation](#update_it_callback1-function-documentation)
  - [`Update_IT_callback2` Function Documentation](#update_it_callback2-function-documentation)
  - [`timerMS` Class Documentation](#timerms-class-documentation)
  - [`flowDelay` Class Documentation](#flowdelay-class-documentation)
- [Voltage Reading Functions](#read-battery-voltage-function-detection)
  - [`readBatteryVoltage` Function Documentation](#read-battery-voltage-function-detection)
  - [`readPowerSupply` Function Documentation](#readpowersupply-function-documentation)
  - [`readMainVoltage` Function Documentation](#readmainvoltage-function-documentation)
  - [`mainVoltageState` Function Documentation](#mainvoltagestate-function-documentation)
  - [`readEarth` Function Documentation](#readearth-function-documentation)
- [GPIO Initialization Function Documentation](#gpioinit-function-documentation)
  - [`GPIOInit` Function Documentation](#gpioinit-function-documentation)
- [`checkButtons` Functions](#checkbuttons-function-documentation) 
- [`readOutputsAlart` Functions Documentation](#readoutputsalart-function-documentation) 


 - [`newEvent` function Documentation](#newevent-function-documentation)

 - [Output management functions and classes](#output-management-functions-and-classes)
    - [`LED` class Documentation](#led-class-documentation)
   - [`Buzzer` class Documentation](#buzzer-class-documentation)
   - [`Output` management functions Documentation](#output-management-functions)
      -  [`LEDManagement` function Documentation](#ledmanagement-function-documentation)
      -  [`BuzzerManagement` function Documentation](#buzzermanagementfunction-documentation)
      -  [`RelayManagement` function Documentation](#relaymanagemen-function-documentation )


- [Main Header File Documentation](#main-header-file-documentation)
  - [`main.h` Overview](#mainh)
  - [Pins Definitions](#pins-definitions)
  - [Flags and Variables](#flags-and-variables)
  - [Enumerations](#enumerations)
  - [Data Arrays](#data-arrays)
  - [Structures](#structures)
  - [Timer Classes](#timer-classes)
  - [External Libraries](#external-libraries)
  - [Serial Communication](#serial-communication)
  - [Global Instances and Objects](#global-instances-and-objects)
  - [Macros](#macros)
  - [Function-Like Macros](#function-like-macros)
  - [Functi](#functi)

# Fire and Power Distribution Control System

## Overview:

The provided code is designed to control and monitor a system related to fire alarm and power distribution. It utilizes various sensors, algorithms, and peripherals to detect and manage potential fire hazards and power-related issues.
## Functions:

### 1. Reading and Monitoring Sensors:

- **`readMux`**: Reads analog values from sensors connected to a multiplexer.
- **`checkPower`**: Checks the status of the battery and power supply.

### 2. Button Inputs:

- **`checkButtons`**: Monitors button states for user input.

### 3. Line Status Evaluation:

- **`evaluateLineStatus`**: Determines the status of power distribution lines based on sensor readings.

### 4. LED Management:

- **`Output::LEDManagement`**: Manages LED states based on system conditions.
  - **Parameters:**
    - `lineStatus`: Status of power distribution lines.
    - `powerStatus`: Status of the battery and power supply.
    - `buttonStatus`: Status of buttons for user input.
    - `mainVoltageState`: Status of the main voltage.
    - `alatrState`: Status of output alerts.
    - `earthStatus`: Status of the earth connection.

### 5. Buzzer Management:

- **`Output::BuzzerManagement`**: Controls the buzzer for alerting users of critical events.
  - **Parameters:**
    - `buttonStatus`: Status of buttons for user input.
    - `newEvent`: Latest event status indicating system changes.

### 6. Relay Management:

- **`Output::RelayManagement`**: Manages relays to control external devices or circuits.
  - **Parameters:**
    - `buttonStatus`: Status of buttons for user input.
    - `newEvent`: Latest event status indicating system changes.

### 7. Watchdog Timer:

- Utilizes a watchdog timer to ensure proper program execution and prevent long-lasting connections.

### 8. Timer Configuration:

- **`configureTimers`**: Sets up hardware timers for periodic tasks.

### 9. Main Loop :

The main loop continuously performs the following tasks:

- Reads values from analog multiplexers.
- Checks power status.
- Monitors button inputs.
- Distributes sensor values to corresponding lines.
- Evaluates line statuses.
- Controls LED states.
- Manages relays.
- Handles the watchdog timer.
- Updates the multiplexer position.


- **Parameter Structure Definition:**
  ```cpp
  struct outputParametrs{
      powerState *PowerStatus;
      status *LineStatus;
      ButtonState *buttonStatus;
      eventStatus *NewEvent;
      double MainVoltage=0.0;
      bool MainVoltageState=false;
      bool AlatrState=false;
      bool fireTrace=false;
  };
  ```
- output parametrs: Defines a structure to store various parameters and statuses related to system functionality.
  -  Reads values from analog multiplexers.
  -  Checks power status.
  -  Monitors button inputs.
  -  Distributes sensor values to corresponding lines.
  -  Evaluates line statuses.
  -  Controls LED states.
  -  Manages relays.
  -  Handles the watchdog timer.
  -  Updates the multiplexer position.
## Usage:

The code is intended for a fire alarm and power distribution control system. It employs various sensors and algorithms to ensure the safety and proper functioning of the system.





# Review the detailed documentation of functions and classes: #

# Power State Check Documentation

## Function
```cpp
powerState checkPower(float VoltageBattery, float VoltagePowerSupply)
```

## General Overview:
The powerState `checkPower`(float VoltageBattery, float VoltagePowerSupply) function is designed to monitor power supply and battery conditions, determining the system's power state based on predefined voltage thresholds. It covers various states, including normal power, battery, low battery, power supply, battery broken, and power off. The function utilizes static Variables: to maintain state between calls.

Installation and Setup:
No specific installation or setup is required for this code.

## Examples:
```cpp
float batteryVoltage = 20.0;
float supplyVoltage = 25.0;
powerState currentState = checkPower(batteryVoltage, supplyVoltage);
````
## Parameters:
- `VoltageBattery`: Voltage of the battery.

- `VoltagePowerSupply`: Voltage of the power supply.

- `Returns`: The current power state, defined by the powerState enum.



## Parameters and Configurations:

 ### Voltage Thresholds:

- `MINIMUM_VOLTAGE_VALIDE`: Minimum valid voltage value.
- `EXIST_POWER_SUPPLY_THRESHOLD`: Threshold for considering the existence of a power supply.
- `EXIST_BATTERY_THRESHOLD`: Threshold for considering the existence of a battery.
- `NORMAL_BATTERY_THRESHOLD`: Threshold for normal battery voltage.
- `LOW_BATTERY_THRESHOLD`: Threshold for low battery voltage.

### Time Thresholds:
- `POWER_LOW_TIME`: Time threshold for determining low power state.
- `CLEAR_TIME`: Time threshold for clearing checks.
- `MAXIMUM_TIME`: Maximum time for stopping battery check.

## Additional Notes:
- The code includes debug messages denoted by POWER_CHECK_DEBUG.
- Static Variables: are used to maintain state between function calls.












# Line Status Evaluation Documentation
## Function
```cpp
status evaluateLineStatus(float current, float voltage, double supplyVoltage, int numberLine)
``````


## General Overview:
The evaluateLineStatus function assesses the status of a given power line based on current, voltage, and other parameters. It incorporates checks for open circuit, normal operation, fire detection, short circuit, and line damage. The function uses static Variables: to retain state information across calls.

Installation and Setup:


## Examples:
```cpp
// Example Usage:
float current = 10.0;
float voltage = 220.0;
double supplyVoltage = 240.0;
int numberLine = 1;
status lineStatus = evaluateLineStatus(current, voltage, supplyVoltage, numberLine);
````
## Parameters:
- `current`: Current flowing through the power line.
- `voltage`: Voltage of the power line.
- `supplyVoltage`: Supply voltage of the overall system.
- `numberLine`: Identifier for the power line.
## Returns:
- Returns the current power line status, as defined by the `status` enum.
## Parameters and Configurations:

- `MINIMUM_LIMIT_VOLTAGE_SHORT_CIRCUIT`: Minimum voltage threshold for short circuit detection.

- `SHORT_CIRCUIT_TIME`: Time threshold for identifying a short circuit.
- `SHORT_CIRCUIT_LINE_ON_TIME`: Time duration before turning the line back on after a short circuit.
- `FIER_DETECTION_TIME`: Time threshold for fire detection.
- `ACCEPTABLE_NUMBER_OF_REPEAT_FIER`: Maximum number of acceptable repeated fire detections.
- `ACCEPTABLE_NUMBER_OF_REPEAT_FIER_EXTERA_LINES`: Additional acceptable repeated fire detections for extra lines.
### Mode Configuration
- `LINE_STATUS_BY_RESISTANCE`: If defined, line status is determined by resistance; otherwise, it's determined by current.
## Additional Notes:

 Debug information is included with statements denoted by `LINE_STATUS_DEBUG`, `LINE_SC_DEBUG`, `LINE_STATE_DEBUG`, `LINE_FIER_DEBUG`.
- The code uses static Variables: (`shortCircuitLock`, `fierDetectLock`) to maintain state.
- Depending on the line status, actions such as turning the line on or off are performed.
- Fire detection and handling logic is implemented, including a counter (`repeatFireDetection`) for repeated fire detections.
- The code includes specific functions (`lineON`, `lineOFF`) for managing the power line state.



# Handle Line Status by Resistance Documentation
## Function
```cpp
status handleLineStatusByResistance(double lineResistor, double voltage, bool shortCircuit)
```````


## General Overview:
The `handleLineStatusByResistance` function is responsible for determining the line status based on the provided line resistor, voltage, and the presence of a short circuit. It checks for various conditions to categorize the line as open circuit, normal, short circuit, or in need of further fire detection checks.

Installation and Setup:


## Examples:
```cpp
// Example of using handleLineStatusByResistance
double exampleLineResistor = 3000.0;
double exampleVoltage = 220.0;
bool exampleShortCircuit = false;

// Call the function to determine the line status
status exampleLineStatus = handleLineStatusByResistance(exampleLineResistor, exampleVoltage, exampleShortCircuit);

// Print the result
LINE_STATUS_DEBUG("Example Line Status: ");
LINE_STATUS_DEBUG(exampleLineStatus);
````
## Parameters:
- `lineResistor`: Resistance of the power line.
- `voltage`: Voltage of the power line.
- `shortCircuit`: Boolean indicating the presence of a short circuit.
## Parameters and Configurations:
 - `MINIMUM_LIMIT_RES_OPEN_CIRCUIT`: Minimum resistance threshold for open circuit detection.
- `MAXIMUM_LIMIT_RES_OPEN_CIRCUIT`: Maximum resistance threshold for open circuit detection.
- `MINIMUM_LIMIT_RES_NORMAL`: Minimum resistance threshold for normal operating condition.
- `MAXIMUM_LIMIT_RES_NORMAL`: Maximum resistance threshold for normal operating condition.
- `MINIMUM_LIMIT_RES_FIER`: Minimum resistance threshold for fire detection.
- `MAXIMUM_LIMIT_RES_FIER`: Maximum resistance threshold for fire detection.
- `MINIMUM_LIMIT_RES_CURRENT_SHORT_CIRCUIT`: Minimum resistance threshold for short circuit detection.
 ### Conditions and Definitions:
 - `V_R_IS_0`: A condition checking if voltage is 0 and line resistor is 0.
- `OPEN_CIRCUIT_RESISTOR`: A condition checking if the line resistor is greater than the maximum limit for open circuit.
- `NORMAL_RESISTOR`: A condition checking if the line resistor is within the range for normal operation.
- `FIER_RESISTOR`: A condition checking if the line resistor is within the range for fire detection.
- `SHORT_CIRCUIT_RESISTOR`: A condition checking if the line resistor is below the minimum limit for short circuit.

## Additional Notes:
Ensure that extremely low values for `lineResistor` and `voltage` are set to 0 to avoid invalid calculations.
The line status is determined based on the defined conditions.
If a short circuit is detected, the state is set to `SHORT_CIRCUIT`.
If an open circuit is detected, the state is set to `OPEN_CIRCUIT`.
If the line is within the normal resistance range, the state is set to `NORMAL`.
If the line resistance indicates a possible fire, the state is set to `CHECK`.

## Returns:
- Returns the determined line status as defined by the `status` enum.













# Handle Line Status by Current Documentation
## Function
```cpp
status handleLineStatusByCurrent(double current, double voltage, bool shortCircuit)
```


## General Overview:
The `handleLineStatusByCurrent` function determines the line status based on the provided current, voltage, and the presence of a short circuit. It checks for various conditions to categorize the line as open circuit, normal, short circuit, or in need of further fire detection checks.
Installation and Setup:


## Examples:
```cpp
// Example of using handleLineStatusByCurrent
double exampleCurrent = 15.0;
double exampleVoltage = 220.0;
bool exampleShortCircuit = false;

// Call the function to determine the line status
status exampleLineStatus = handleLineStatusByCurrent(exampleCurrent, exampleVoltage, exampleShortCircuit);

// Print the result
LINE_STATUS_DEBUG("Example Line Status: ");
LINE_STATUS_DEBUG(exampleLineStatus);
````
## Parameters:
- `current`: Current flowing through the power line.
- `voltage`: Voltage of the power line.
- `shortCircuit`: Boolean indicating the presence of a short circuit
## Parameters and Configurations:
 - `MINIMUM_LIMIT_OPEN_CIRCUIT`: Minimum current threshold for open circuit detection.
- `MAXIMUM_LIMIT_OPEN_CIRCUIT`: Maximum current threshold for open circuit detection.
- `MINIMUM_LIMIT_NORMAL`: Minimum current threshold for normal operating condition.
- `MAXIMUM_LIMIT_NORMAL`: Maximum current threshold for normal operating condition.
- `MINIMUM_LIMIT_FIER`: Minimum current threshold for fire detection.
- `MAXIMUM_LIMIT_FIER`: Maximum current threshold for fire detection.
- `MINIMUM_LIMIT_CURRENT_SHORT_CIRCUIT`: Minimum current threshold for short circuit detection.short circuit detection.

 ### Conditions and Definitions:
- `V_I_IS_0`: A condition checking if voltage is 0 and current is 0.
- `OPEN_CIRCUIT_CURRENT`: A condition checking if the current is within the range for open circuit.
- `NORMAL_CURRENT`: A condition checking if the current is within the range for normal operation.
- `FIER_CURRENT`: A condition checking if the current is within the range for fire detection.
- `SHORT_CIRCUIT_VOLTAGE`: A condition checking if the voltage is above the minimum threshold for short circuit.
- `SHORT_CIRCUIT_CURRENT`: A condition checking if the current is above the minimum threshold for short circuit.

## Additional Notes:
EEnsure that extremely low values for current and voltage are set to 0 to avoid invalid calculations.
The line status is determined based on the defined conditions.
If a short circuit is detected, the state is set to SHORT_CIRCUIT.
If an open circuit is detected, the state is set to OPEN_CIRCUIT.
If the line is within the normal current range, the state is set to NORMAL.
If the line current indicates a possible fire, the state is set to CHECK.

## Returns:
- Returns the determined line status as defined by the `status` enum.


















# Read Multiplexer (Mux) Function Documentation
## Function
```cpp
void readMux(byte address, Mux &mux)
```


## General Overview:
The `readMux` function is designed to read analog values from a multiplexer (Mux) based on the provided address. It utilizes control signals (Sela, Selb, Selc) to select the desired channel on the multiplexer and reads analog values from multiple channels. The read values are stored in the respective arrays of the `Mux` structure.




## Examples:
```cpp
// Example Usage:
Mux myMux; // Assuming Mux structure is defined
byte exampleAddress = 3; // Example address
readMux(exampleAddress, myMux);

// Accessing read values:
float valueOnChannel1 = myMux.Values1[exampleAddress];
float valueOnChannel2 = myMux.Values2[exampleAddress];
float valueOnChannel3 = myMux.Values3[exampleAddress];
float valueOnChannel4 = myMux.Values4[exampleAddress];

````
## Parameters:
- `address`: Byte representing the address of the multiplexer channel to be read.
- `mux`: Reference to a `Mux` structure where the read values will be stored.

## Parameters and Configurations:
 - `ADC_RESOLUTION`: The resolution of the ADC (Analog-to-Digital Converter).
- `VREF`: Reference voltage for the ADC.
- `VOLTAGE_ATTENUATION`: Attenuation factor for the ADC voltage due to resistive division.
- `VOLTAGE_DROP_MUX`: Voltage drop on the multiplexer.
- `RSHANT`: Shunt resistor value for current calculation.
.
### Control Values:
- `controlValues`: An array of three values (LOW or HIGH) representing the control signals Sela, Selb, and Selc for the multiplexer.

### Control Signal Calculation:
- The function calculates control values based on the provided address.

### Channel Selection
- The function sets control values using digitalWrite to select the desired channel on the multiplexer.
- A delay of 5 milliseconds is applied after setting control values.
### Analog Reading
- The function reads analog values from each channel of the multiplexer and converts them to corresponding physical quantities.
- The read values are stored in the respective arrays of the `Mux` structure.
- A delay of 5 milliseconds is applied after each analog read operation.




## Additional Notes:
- It is important to consider the accuracy of the ADC and the impact of delays on the overall performance.
- Ensure that the provided `Mux` structure has sufficient storage capacity for the read values.

## Returns:
- The function does not return a value directly. The read values are stored in the provided `Mux` structure via reference.

















# Distribute Mux Values Function Documentation
## Function
```cpp
void distributionMuxValues(char cardSituation, Mux &mux)
```


## General Overview:
The `distributionMuxValues` function is designed to distribute values obtained from multiple multiplexers (Mux) to their corresponding arrays based on the current card situation. It uses a switch statement to handle different configurations and assigns the values from the Mux arrays to specific arrays representing line voltages and currents




## Examples:
```cpp
// Example Usage:
Mux myMux; // Assuming Mux structure is defined
char exampleCardSituation = 1; // Example card situation
distributionMuxValues(exampleCardSituation, myMux);

// Accessing distributed values:
float voltageOnLine2 = lineVoltage[2];
float currentOnLine3 = lineCurrent[3];
// Continue for other lines as needed...


````
## Parameters:
- `cardSituation`: Character representing the current card configuration (e.g., 0, 1).
- `mux`: Reference to a `Mux` structure containing the read values from multiple multiplexers.

### Arrays:
- `lineVoltage`: An array to store line voltage values for different lines and configurations.
- `lineCurrent`: An array to store line current values for different lines and configurations.

## Parameters and Configurations:
### Cases :
- Case 0 (Main Lines Configuration):
  - Assign values from Mux1 to corresponding arrays for the main lines.

- Case 1 (Main Lines & Single Card Configuration):
  - Assign values from Mux1 to corresponding arrays for the main lines.
  - Assign values from Mux2 to corresponding arrays for the single card.

- Case 2 (Two-Card & Main Lines Configuration):
  - Assign values from Mux1 to corresponding arrays for the main lines.
  - Assign values from Mux2 to corresponding arrays for the first card.
  - Assign values from Mux3 to corresponding arrays for the second card.




## Additional Notes:
- Ensure that the `Mux` structure has sufficient storage capacity for the read values.
- The function assumes the existence of arrays (`lineVoltage` and `lineCurrent`) for storing distributed values.

## Returns:
- The function does not explicitly return a value. The distributed values are stored in the provided `lineVoltage` and `lineCurrent` arrays via reference.




# Update Mux Position Function Documentation

## Function
```cpp
void updateMuxPosition(char &cardSituation)
```
## General Overview:

The `updateMuxPosition` function is designed to update the multiplexer (Mux) position for analog readings based on the current `cardSituation`. It increments the `muxPosition` variable, and when it reaches a specific value, it resets to 0 and sets a flag (`readAnalogs`) to indicate that analog readings can be performed.




## Examples:
```cpp
// Example Usage:
char currentCardSituation = 1; // Example card situation
updateMuxPosition(currentCardSituation);

// Accessing updated Variables:
int updatedMuxPosition = muxPosition;
bool canReadAnalogs = readAnalogs;
// Continue with other parts of the program...

````
## Parameters:
- `cardSituation`: A reference to a character variable indicating the current card configuration (0 (`main`), 1, 2).



### Arrays:
- `lineVoltage`: An array to store line voltage values for different lines and configurations.
- `lineCurrent`: An array to store line current values for different lines and configurations.

## Parameters and Configurations:

- The function increments the `muxPosition` variable and checks if it has reached a specific value (7).
- If the condition is met, the `muxPosition` is reset to 0, and the `readAnalogs` flag is set to true.
- Otherwise, the `muxPosition` is incremented.


## Variables:
- `muxPosition`: A variable representing the current position of the multiplexer for analog readings.
- `readAnalogs`: A flag indicating whether analog readings can be performed.

## Control Flow:
- The function increments the `muxPosition` variable and checks if it has reached a specific value (7).
- If the condition is met, the `muxPosition` is reset to 0, and the `readAnalogs` flag is set to true.
- Otherwise, the `muxPosition` is incremented.

## Additional Notes:

- The function assumes that there is a valid use case for updating the Mux position based on the `cardSituation`.
- The `readAnalogs` flag can be used to trigger analog readings at the appropriate time in the program.
- Ensure that the control flow and logic align with the requirements of your specific application.


## Returns:

- The `updateMuxPosition` function does not explicitly return a value.
- It updates the `muxPosition` variable and sets the `readAnalogs` flag as needed.
- The updated values can be accessed directly from the relevant Variables: (`muxPosition` and `readAnalogs`) after calling the function.







# Set Card Situation Function Documentation

## Function
```cpp
char setCardSituation(void)
``````
## General Overview:

The `distributionMuxValues` function is designed to distribute values obtained from multiple multiplexers (Mux) to their corresponding arrays based on the current card situation. It uses a switch statement to handle different configurations and assigns the values from the Mux arrays to specific arrays representing line voltages and currents




## Examples:
```cpp
// Example Usage:
char detectedCardSituation = setCardSituation();

// Accessing the detected card situation:
if (detectedCardSituation == '0') {
  // Main Lines Configuration
} else if (detectedCardSituation == '1') {
  // Main Lines & Single Card Configuration
} else if (detectedCardSituation == '2') {
  // Two-Card & Main Lines Configuration
}

````
## Parameters:
- No parameters are required for this function.


### Arrays:
- `lineVoltage`: An array to store line voltage values for different lines and configurations.
- `lineCurrent`: An array to store line current values for different lines and configurations.

## Parameters and Configurations:

- `CS1` and `CS2`: Pins for detecting the presence of Card 1 and Card 2.
- `Line1` to `Line12`: Pins representing individual lines.

## Variables:
- `cardSituation`: A character variable indicating the current card configuration (0, 1, 2).
- `card1Present` and `card2Present`: Boolean Variables: indicating the presence of Card 1 and Card 2.

## Additional Notes:
- The function assumes the existence of specific pins (`CS1`, `CS2`, `Line1` to `Line12`) for detecting card presence and activating lines.
- Boolean Variables: (`card1Present` and `card2Present`) are used to track the presence of Card 1 and Card 2.
- The function activates specific lines based on the detected card configuration using digitalWrite.
- Ensure that the pin configurations are compatible with the hardware setup.
- It is recommended to call this function periodically to update the card situation based on real-time changes in card presence.
- The `setCardSituation` function does not explicitly handle error cases if both cards are absent simultaneously.

## Returns:
- The function returns the determined `cardSituation` character.








# All Zeros in Range Check Function Documentation
## General Overview:
The allZerosInRange function checks whether all values in a specified range (inclusive) of the lineCurrent array are zero. It iterates through the specified range and returns true if all values are zero; otherwise, it returns false.

## Function
```cpp
bool allZerosInRange(int start, int end)
``````



## Examples:
```cpp
// Example Usage:
bool areAllZeros = allZerosInRange(0, 3); // Assuming a range for the array indices
if (areAllZeros) {
  // Perform actions when all values in the specified range are zero.
} else {
  // Perform actions when at least one value in the specified range is non-zero.
}

````
## Parameters:
- `start`: The starting index of the range to be checked.
- `end`: The ending index of the range to be checked.


## Variables:
- `lineCurrent`: An array containing current values for different power lines.
##  Control Flow:
- The function iterates through the specified range (from `start` to `end`).
- If any value in the `lineCurrent` array within the specified range is non-zero, the function returns false.
- If all values in the range are zero, the function returns true.


## Additional Notes:
- The function assumes the existence of the `lineCurrent` array and its appropriate initialization.
- Ensure that the range specified by `start` and `end` is valid for the array indices.



## Returns:

- Returns a boolean value:
  - `true` if all values in the specified range are zero.
  - `false` if at least one value in the specified range is non-zero.



# Handle Card Present Errors Function Documentation

## Function
```cpp
void handleCardPresentErrors(char &cardSituation)
``````
## General Overview:
The `handleCardPresentErrors` function is designed to handle card present errors based on the current 'cardSituation'. It evaluates specific conditions using the allZerosInRange function and sets the CardPresentError variable accordingly.
## Parameters:
- `cardSituation`: A reference to a character variable indicating the current card configuration (0, 1, 2).

## Examples:
```cpp
// Example Usage:
char currentCardSituation = 2; // Example card situation
handleCardPresentErrors(currentCardSituation);

// Accessing the card present error code:
int errorType = CardPresentError;
// Responding to different error types...


````



## Variables:
- `CardPresentError`: An integer variable indicating the type of card present error (0 for no error).

##  Control Flow:
- If `cardSituation` is 1, it checks whether all values in the range [4, 7] are zero and sets `CardPresentError` accordingly.
- If `cardSituation` is 2, it checks whether all values in the ranges [4, 7] and [8, 11] are zero and sets `CardPresentError` accordingly.
- Different error codes are assigned based on the conditions met.


## Additional Notes:
- The function relies on the `allZerosInRange` function for checking if all values in a specified range are zero.
- The `CardPresentError` variable indicates the type of card present error (0 for no error, 1-3 for specific error types).
- Ensure that the ranges specified for array indices align with the structure of your data.

## Returns

- The `handleCardPresentErrors` function does not explicitly return a value.
- It updates the `CardPresentError` variable based on the specified conditions and card situation.
- The updated value can be accessed directly from the `CardPresentError` variable after calling the function.


# All Zeros in Range Check Function Documentation
## General Overview:
The allZerosInRange function checks whether all values in a specified range (inclusive) of the lineCurrent array are zero. It iterates through the specified range and returns true if all values are zero; otherwise, it returns false.

## Function
```cpp
bool allZerosInRange(int start, int end)
``````



## Examples:
```cpp
// Example Usage:
bool areAllZeros = allZerosInRange(0, 3); // Assuming a range for the array indices
if (areAllZeros) {
  // Perform actions when all values in the specified range are zero.
} else {
  // Perform actions when at least one value in the specified range is non-zero.
}

````
## Parameters:
- `start`: The starting index of the range to be checked.
- `end`: The ending index of the range to be checked.

## Parameters and Configurations:
## Variables:
- `lineCurrent`: An array containing current values for different power lines.
##  Control Flow:
- The function iterates through the specified range (from `start` to `end`).
- If any value in the `lineCurrent` array within the specified range is non-zero, the function returns false.
- If all values in the range are zero, the function returns true.


## Additional Notes:
- The function assumes the existence of the `lineCurrent` array and its appropriate initialization.
- Ensure that the range specified by `start` and `end` is valid for the array indices.



## Returns:

- Returns a boolean value:
  - `true` if all values in the specified range are zero.
  - `false` if at least one value in the specified range is non-zero.




# Configure Timers Function Documentation

## Function
```cpp
void configureTimers(void)
``````
## General Overview:
The `configureTimers` function is responsible for configuring timers for periodic tasks in the system. It creates and configures two hardware timers (`TIM1` or `TIM2` and `TIM3`) and attaches corresponding interrupt callbacks (`Update_IT_callback1` and `Update_IT_callback2`).

## Parameters:
- No parameters.
## Examples:
```cpp
//initialize Timer
 configureTimers();
``````

## Parameters and Configurations:
### functions
- `Update_IT_callback1`: Placeholder for the callback function associated with the first timer.
- `Update_IT_callback2`: Placeholder for the callback function associated with the second timer.

## Variables:
- `Instance`: The timer instance (either `TIM1` or `TIM2`).
- `Instance1`: The timer instance for the second timer (`TIM3`).
- `MyTim`, `MyTim2`: Pointers to hardware timers.

##  Control Flow:
- Create and configure the first hardware timer (`MyTim`), set its overflow period to 100 milliseconds (10 Hz), and attach `Update_IT_callback1` as the interrupt callback.
- Create and configure the second hardware timer (`MyTim2`), set its overflow period to 1 second (1 Hz), and attach `Update_IT_callback2` as the interrupt callback.
- Start both timers.


## Additional Notes:
- Modify the overflow periods and callback functions based on the specific timing requirements of your application.
- Ensure that the interrupt callback functions (`Update_IT_callback1` and `Update_IT_callback2`) are defined elsewhere in your code.





# Update_IT_callback1 Function Documentation

## Function
```cpp
void Update_IT_callback1(void)
``````
## General Overview:
The `Update_IT_callback1` function is an interrupt callback associated with the first hardware timer in the system. It performs various tasks at a rate of 10 Hz. These tasks include updating timers (`currentTime` and `fultSencetimer`), checking battery status, and managing LED indicators based on card present errors and fault conditions.
## Parameters:
- No parameters.
## Examples:
```cpp
// Example Usage:
Update_IT_callback1();
// Perform tasks associated with the Update_IT_callback1 function.
``````

## Variables:
- `currentTime`: Counter for tracking time.
- `fultSencetimer`: Counter for a fault sensitivity timer.
- `batteryCheckTime`: Timer object for battery checking.
- `ledBlinker2`: Variable controlling LED blinking.
- `CardPresentError`: Error code indicating card present issues.
- `sr`: ShiftRegister object for managing LED indicators.
- `ledErrorsPins`: Array of pins associated with LED indicators for errors.
- `fultCounter`: Counter for fault conditions.
- `faultFlag`: Flag indicating the presence of a fault condition.

 ##  Control Flow:
- Increment `currentTime` to track time.
- Increment `fultSencetimer` for fault sensitivity.
- Update the `batteryCheckTime` timer.
- Toggle `ledBlinker2` for LED blinking.
- Check and manage LED indicators based on card present errors.
- Check and update the fault condition flag (`faultFlag`).

## Additional Notes:
- Adjust the tasks performed in the callback based on specific application requirements.
- Ensure that the error indicator LEDs and corresponding pins are correctly configured.
- Modify the fault sensitivity and handling logic as needed.

## Returns:
- No return value.



# Update_IT_callback2 Function Documentation

## Function
```cpp
void Update_IT_callback2(void)
``````

## General Overview:
The `Update_IT_callback2` function is an interrupt callback associated with the second hardware timer in the system. It updates timers related to fire detection (`fierFlow`) and short circuit handling (`shortCircuitFlow`) for each power line.

## Parameters:
- No parameters.

## Examples:
```cpp
// Example Usage:
Update_IT_callback2();
// Perform tasks associated with the Update_IT_callback2 function.
``````

## Variables:
- `fierFlow`: Timer object for fire detection.
- `shortCircuitFlow`: Array of timer objects for short circuit handling for each power line.
##  Control Flow:
- Update the fire detection timer (`fierFlow`).
- Iterate through each power line and update the corresponding short circuit handling timer (`shortCircuitFlow`).

## Additional Notes:
- Adjust the update frequency and logic based on specific application requirements.
- Ensure that the timers associated with fire detection and short circuit handling are configured appropriately.
- Modify the function as needed for your specific use case.

## Returns:- No return value.


# timerMS Class Documentation

## Class
```cpp
class timerMS {
public:
    STATE status;
    unsigned long value = 0;
    void update();
};
``````
## General Overview:
The `timerMS` class represents a timer in milliseconds with functionalities for starting, stopping, and pausing the timer. It includes an internal state (`status`) and a counter (`value`) to keep track of elapsed time.

## Function 
- `update()`Method: Updates the timer based on its current status (START, STOP, PAUSE).

- If the status is `START`, increments the timer value.
- If the status is `STOP`, resets the timer value to 0.
- If the status is `PAUSE`, does nothing.
```cpp
void timerMS::update() {
    if (status == START) {
        value++;
    } else if (status == STOP) {
        value = 0;
    } else if (status == PAUSE) {
        // Do nothing for PAUSE state.
    }
}
``````
## Parameters
- `time_ms`: The duration of the delay in milliseconds.

## Examples
```cpp
// Example Usage:
timerMS myTimer;
myTimer.status = START;
myTimer.update();
// The timer value will be incremented.

myTimer.status = STOP;
myTimer.update();
// The timer value will be set to 0.

myTimer.status = PAUSE;
myTimer.update();
// The timer value remains unchanged.
``````
## Variables:
- `status`: Enum representing the timer status (START, STOP, PAUSE).
- `value`: Counter for elapsed time in milliseconds.

## Control Flow
- In the `update()` function, the timer is incremented if the status is START, set to 0 if the status is STOP, and remains unchanged if the status is PAUSE.
## Additional Notes:

- The `timerMS` class provides a generic timer functionality with states (START, STOP, PAUSE) and an elapsed time counter (`value`).
- The `flowDelay` class extends the `timerMS` class to offer a delay mechanism. It can be used to introduce delays in the program execution.
- Ensure that the `update` function of the `timerMS` class is regularly called to keep the timer updated based on its status.
- Example usage and control flow explanations are provided in the documentation for better understanding.
- Feel free to integrate these classes into your program and adapt them according to your specific requirements.

## Returns
- `true`: If the delay duration has elapsed.
- `false`: If the delay is still in progress.




# flowDelay Class Documentation

## Class
```cpp
class flowDelay: public timerMS {
public:
    bool Delay(unsigned long time_ms);
};
``````
## General Overview:
The `flowDelay` class is derived from the `timerMS` class and extends its functionality by providing a delay mechanism. It allows you to initiate a delay for a specified duration in milliseconds.
## Functions
- `Delay(unsigned long time_ms)`: Initiates a delay for the specified duration in milliseconds.

## Parameters
- `time_ms`: Duration of the delay in milliseconds.
## Examples
```cpp
// Example Usage:
flowDelay myDelay;
if (myDelay.Delay(1000)) {
    // Delay of 1000 milliseconds has elapsed.
    // Perform actions here.
} else {
    // Delay is still in progress.
    // Continue with other tasks.
}
``````


## Control Flow
- The `Delay` function sets the timer status to START and checks whether the elapsed time has reached the specified duration (`time_ms`). If true, it sets the status to STOP and returns `true`; otherwise, it returns `false`.

# Additional Notes

- The `flowDelay` class is designed to provide a straightforward delay mechanism by extending the functionality of the `timerMS` class.
- When using the `Delay` function, it's important to consider that it operates in a blocking manner, meaning the program execution will pause until the specified delay time elapses.
- Ensure that the `update` function of the `timerMS` class is regularly called to keep the timer updated based on its status.
- Example usage is provided in the documentation for a better understanding of how to integrate and utilize the `flowDelay` class in your program.
- Feel free to adapt and customize the class to suit your specific application requirements.

## Returns
- `true`: If the delay duration has elapsed.
- `false`: If the delay is still in progress.








# Read Battery Voltage Function detection
```cpp
float readBattery(float VADC)
``````
## General Overview:
The `readBattery` function is designed to calculate and return the battery voltage based on the provided analog-to-digital converter (ADC) reading (`VADC`). The function incorporates scaling, multiplication, and adjustments to derive the battery voltage. Additionally, conditional statements are used to fine-tune the calculated voltage within specific ranges.


## Parameters:
- `VADC`: Voltage read from the analog-to-digital converter (ADC).

## Examples:

```cpp
float adcReading = 123.45; // Replace with the actual ADC reading
float batteryVoltage = readBattery(adcReading);
````````
## Additional Notes:
- The `readBattery` function calculates the battery voltage based on the provided ADC reading (`VADC`).
- The function utilizes a formula that involves scaling, multiplication, and adjustments to derive the battery voltage.
- Conditional statements are used to fine-tune the calculated voltage within specific ranges.
- Ensure that the `VADC` value passed to the function is appropriate for the desired voltage calculation.
- The calculated battery voltage is then returned by the function.
## Returns:
- `float`: The calculated battery voltage.


# readPowerSupply Function Documentation

## Function
```cpp
float readPowerSupply(float VADC);
```````
## General Overview:
The readPowerSupply function calculates the power supply voltage based on the given analog voltage (`VADC`). It utilizes a specific formula to convert the analog voltage to the corresponding power supply voltage.

Parameters:
`VADC`: Analog voltage value obtained from the power supply.

## Examples:
```cpp
// Example of using readPowerSupply function
float analogVoltage = 2.5;  // Replace with the actual analog voltage value
float powerSupplyVoltage = readPowerSupply(analogVoltage);
// Now, powerSupplyVoltage contains the calculated power supply voltage
// Use powerSupplyVoltage as needed in your application
``````````````
## Returns:
Returns the calculated power supply voltage


# readMainVoltage Function Documentation

## Function
```cpp
double readMainVoltage(double VADC);
````````
## General Overview:
The readMainVoltage function calculates the main voltage based on the given analog voltage (VADC). It uses specified resistor values and a diode voltage drop to convert the analog voltage into the corresponding main voltage. Additionally, a conversion factor is applied for further calibration.

## Parameters:
`VADC`: Analog voltage value obtained from the main voltage.


## Parameters and Configurations:
- `R1`: The resistor connected to VCC.
- `R2`: The resistor connected to ground.
`VDiode`: Diode voltage drop.
## Calibration Constants
- `X1`: Calibration constant.
- `X2`: Calibration constant.
- `Y1`: Calibration constant.
- `Y2`: Calibration constant.
## Examples:
```cpp
float analogVoltage = 3.0;  // Replace with the actual analog voltage value
double mainVoltage = readMainVoltage(analogVoltage);
// mainVoltage is now the calculated main voltage based on the given analog voltage.
``````
## Returns:
Returns the calculated main voltage.


# mainVoltageState Function Documentation

## Function
```cpp
bool mainVoltageState(double mainVoltage)
````````
## General Overview:
The function compares the provided main voltage with a predefined minimum voltage threshold.

## Parameters:
`mainVoltage`: `A` double value representing the main voltage to be checked.

## Parameters and Configurations:

`MINIMUM_VOLTAGE_MAIN`: Predefined minimum voltage threshold for the main power.

## Examples:
```cpp
double mainVoltage = 20.5; // Main voltage value to be checked
bool voltageStatus = mainVoltageState(mainVoltage);
// The function will return true if the main voltage is greater than or equal to 19V, otherwise false.
`````
## Returns:
`true` if the main voltage meets or exceeds the minimum threshold.
`false` if the main voltage is below the minimum threshold.




# readEarth Function Documentation
## Function: 
```cpp
bool readEarth(float earthVoltage)
```
## General Overview:
The `readEarth` function is responsible for determining whether the measured earth voltage is within acceptable limits.

## Parameters:
- **earthVoltage**: A floating-point value representing the measured earth voltage.

## Control Flow:
- The function compares the measured earth voltage with a predefined threshold (0 in this case).
- If the earth voltage is less than 0, indicating a negative voltage or an invalid measurement, the function returns false.
- Otherwise, if the earth voltage is greater than or equal to 0, the function returns true.

## Additional Notes:
- Ensure proper calibration and error handling mechanisms are in place for accurate earth voltage measurements.
- Adjust the threshold value according to system specifications and requirements.

## Returns:
A boolean value indicating whether the measured earth voltage is within acceptable limits.

## Examples:
```cpp
float earthVoltage = // measured earth voltage value
bool isEarthValid = readEarth(earthVoltage);
````




# GPIOInit Function Documentation

## Function
This function initializes the GPIO pins for various components and settings.

## General Overview:
The GPIOInit function configures pins for relay control, line control, button and jumper inputs, battery charger control, error indication, and analog readings.

## Parameters:
None

## Examples:
```cpp
GPIOInit();
``````

### Relay Control Pins

- `rel1`, `rel2`: Output pins for relay control.
- `relo1`, `relo2`: Output pins for another set of relays.
### Line Control Pins

`Line1` to `Line12`: Output pins for controlling different lines.
`Button` and `Jumper` Pins

`CS1`, `CS2`: Input pins for checking the presence of cards.
`JUMPER`: Input pin for jumper configuration.
`But1` to `But5`: Input pins for button inputs.
### Battery Charger Pins

- `Batcharges`: Output pin for controlling battery charges.
- `ChangeVolt`: Output pin for changing voltage.
- `Error` Indication Pins

`LEDerror`: Output pin for indicating errors with an LED.
`MCUbuzz`: Output pin for buzzer control.
Analog Pins

`Sela`, `Selb`, `Selc`: Output pins for selecting different analog channels.
`Analog1` to `Analog4`: Input pins for reading analog values.

##  Control Flow:
- Relay control pins are set to LOW.
- Line control pins are set to LOW.
- Button, Jumper, and Analog pins are configured accordingly.
- Battery charger pins are not explicitly set in the provided code.
- Error indication pins are set to LOW.
## Returns:
- None


# readOutputsAlart Function Documentation

## Function:
```cpp
bool readOutputsAlart(float outputVoltage1,float outputVoltage2)
```
## General Overview:
The `readOutputsAlart` function checks if the measured output voltages are below the minimum threshold, indicating an alarm condition.

## Parameters:
- **outputVoltage1**: A floating-point value representing the voltage of the first output.
- **outputVoltage2**: A floating-point value representing the voltage of the second output.

## Variables:
- **VOLTAGE_ALART_NORMAL**: A constant representing the normal voltage threshold for the outputs.
- **MINIMUM_VOLTAGE_ALART**: A constant representing the minimum voltage threshold for triggering an alarm.

## Control Flow:
- The function compares the measured voltages of both outputs with the minimum voltage threshold for triggering an alarm.
- If either of the output voltages is less than the minimum threshold (`MINIMUM_VOLTAGE_ALART`), the function returns true, indicating an alarm condition.
- Otherwise, if both output voltages are greater than or equal to the minimum threshold, the function returns false, indicating normal operation.

## Additional Notes:
- Ensure proper calibration and error handling mechanisms are in place for accurate voltage measurements.
- Adjust the threshold values (`VOLTAGE_ALART_NORMAL` and `MINIMUM_VOLTAGE_ALART`) according to system specifications and requirements.

## Returns:
A boolean value indicating whether an alarm condition is present based on the measured output voltages.

## Examples:
```cpp
float outputVoltage1 = // measured voltage of the first output
float outputVoltage2 = // measured voltage of the second output
bool isAlartTriggered = readOutputsAlart(outputVoltage1, outputVoltage2);
````





# checkButtons Function Documentation
## Function
```cpp
void checkButtons(bool &resetFier) 
``````
## General Overview:
The `checkButtons` function monitors the status of various buttons and responds to specific button presses. It controls functionalities such as toggling the general fault, checking LEDs, resetting all lines, enabling/disabling the alarm relay, and turning off the buzzer.


## Examples:
```cpp
// Example: Using checkButtons
checkButtons();
`````````

## Variables:
Several static and global variables, such as buttonPressTime, generalFault, ledBlinker1, and others, are used to control the behavior of the function.

##  Control Flow:

#### 1. Buzzer Off Button (But5):

- If the button is pressed, toggle the generalFault status after a debounce period.
 ####  2. LED Check Button (But4):

- Turn off all LEDs, check for card present errors, and blink the LEDerror. Restore the LED state after the check.
 #### 3. All Line Reset Button (But3 and JUMPER):

- If both buttons are pressed, reset various system variables, line parameters

 ####  4. Alarm Relay On Button (But2):

- If the button is pressed, turn on the alarm relay (relayControl), set the ledesounder, and update related flags.
####  5. Alarm Relay Off Button (But1):

- If the button is pressed, turn off the alarm relay (relayControl) and update related flags.

## Additional Notes:

- Ensure proper debouncing for button presses.
- The function plays a crucial role in user interaction and system control.

## Returns:
- None


# newEvent Function Documentation
 ##Function:
```cpp
eventStatus *newEvent(status *lineStatus,powerState *powerStatus,bool mainVoltageState,bool outputAlartState)
```
## General Overview:
The `newEvent` function is responsible for generating and updating event statuses based on changes in system parameters such as line status, power state, main voltage state, and output alert state.

## Parameters:
- **lineStatus**: An array of status enums representing the current status of each line.
- **powerStatus**: A pointer to a powerState enum representing the current power state.
- **mainVoltageState**: A boolean indicating whether the main voltage is within acceptable limits.
- **outputAlartState**: A boolean indicating the state of output alerts.

## Variables:
- **last**: A static event struct representing the last known event.
- **event**: A boolean indicating whether an event has occurred.
- **result**: A static eventStatus struct representing the current event status.
- **power**: A powerState enum representing the current power state.
- **lineNormalLatch**: A static boolean indicating whether the line status is normal.
- **lastNumberLine**: A static character storing the last line number.
- Various macro definitions for detecting new events and abnormal states.

## Control Flow:
- The function iterates through each line and checks for new events or abnormal states.
- If a new event or abnormal state is detected, the appropriate event type and details are updated in the result struct.
- The function returns a pointer to the result struct indicating the event status.

## Additional Notes:
- Ensure proper initialization of variables and macros before calling this function.
- Adjust the macro definitions and control flow according to specific system requirements and configurations.
## Examples:
```cpp
status lineStatus[12];
powerState powerStatus;
bool mainVoltageState, outputAlartState;
// Example call:
eventStatus *eventResult = newEvent(lineStatus, &powerStatus, mainVoltageState, outputAlartState);
`````
## Returns:
A pointer to the `eventStatus` struct representing the current event status.


# Output management functions and classes

# LED Class Documentation

## Overview:
The `LED` class provides functionality to manage LEDs, including turning them on or off and blinking them with custom patterns.

## Constructors:
- **LED(unsigned char ID)**: Initializes an LED object with the specified ID.

## Methods:
- **LEDBegin()**: Initializes the LED object.
- **turnOn(char numerPin)**: Turns on the LED connected to the specified pin.
- **turnOff(char numerPin)**: Turns off the LED connected to the specified pin.
- **blink(char numerPin, unsigned int timeBlinking)**: Blinks the LED connected to the specified pin with the given duration.
- **blinkCustum(char numerPin, unsigned int timeOn, unsigned int timeOff)**: Blinks the LED connected to the specified pin with a custom on-off pattern.
- **turnOnArry(char *arry, unsigned int length)**: Turns on multiple LEDs specified in the array.
- **turnOffArry(char *arry, unsigned int length)**: Turns off multiple LEDs specified in the array.
- **blinkCustumArry(char *arry, unsigned int length, unsigned int timeOn, unsigned int timeOff)**: Blinks multiple LEDs with a custom on-off pattern specified in the array.
- **blinkArry(char *arry, unsigned int length, unsigned int timeBlinking)**: Blinks multiple LEDs with the same duration specified in the array.

## Variables:
- **id**: An unsigned char representing the ID of the LED.
- **ActivityState**: A boolean indicating the activity state of the LED.

## Control Flow:
- The `LED` class provides methods to control individual LEDs or arrays of LEDs based on specific timing patterns.
- Each method handles the behavior of LEDs according to the provided parameters.

## Additional Notes:
- Ensure proper initialization of LED pins before using the methods.
- Adjust timing parameters according to specific requirements and hardware configurations.

## Returns:
The methods do not return values; they directly control the state of LEDs based on the provided parameters.

## Example:
```cpp
// Example for LED::LEDBegin()
LED led1(1);
led1.LEDBegin(); // Initializes LED with ID 1

// Example for LED::turnOn()
led1.turnOn(13); // Turns on LED connected to pin 13

// Example for LED::turnOff()
led1.turnOff(13); // Turns off LED connected to pin 13

// Example for LED::blink()
led1.blink(13, 1000); // Blinks LED connected to pin 13 for 1 second

// Example for LED::blinkCustum()
led1.blinkCustum(13, 500, 300); // Blinks LED connected to pin 13 with a custom pattern

// Example for LED::turnOnArry()
char pins[] = {10, 11, 12};
led1.turnOnArry(pins, sizeof(pins)/sizeof(pins[0])); // Turns on LEDs connected to pins 10, 11, and 12

// Example for LED::turnOffArry()
led1.turnOffArry(pins, sizeof(pins)/sizeof(pins[0])); // Turns off LEDs connected to pins 10, 11, and 12

// Example for LED::blinkCustumArry()
led1.blinkCustumArry(pins, sizeof(pins)/sizeof(pins[0]), 500, 300); // Blinks LEDs with a custom pattern

// Example for LED::blinkArry()
led1.blinkArry(pins, sizeof(pins)/sizeof(pins[0]), 1000); // Blinks LEDs with a common pattern

```


# Buzzer Class Documentation

## Overview:
The `Buzzer` class manages the behavior of a buzzer, including turning it on or off, producing single-tone alarms, and repeating tone patterns.

## Variables:
- **buzzerFlow**: A `flowDelay` object to control the duration of single-tone buzzers.
- **buzzerRepeadFlow**: A `flowDelay` object to control the duration of repeated tone patterns.
- **buzzerTimeON**: An unsigned integer representing the duration of the buzzer being on.
- **buzzerTimeOFF**: An unsigned integer representing the duration of the buzzer being off.
- **buzzerRepedTimeOFF**: An unsigned integer representing the duration of the repeated tone pattern being off.
- **buzzerRepedTimeON**: An unsigned integer representing the duration of the repeated tone pattern being on.
- **numberRepead**: A pointer to an integer representing the number of times the repeated tone pattern should be played.
- **AlarmActive**: A boolean indicating the activation status of the buzzer.

## Methods:
- **Begin(bool ActivityState)**: Initializes the buzzer and controls its behavior based on the activity state.
- **Repead(int *NumberRepead, unsigned int timeON, unsigned int timeOFF)**: Plays a repeated tone pattern specified by the number of repeats and the durations of on and off states.
- **TurnOn(bool ActivityState)**: Turns on the buzzer if the activity state is true.
- **TurnOff()**: Turns off the buzzer.
- **SingelOn(unsigned int timeON, unsigned int timeOFF)**: Plays a single-tone buzzer with the specified durations of on and off states.
- **localBib()**: Produces a short beep sound locally.

## Control Flow:
- The `Buzzer` class provides methods to control the behavior of the buzzer based on specific timing patterns and activity states.
- The buzzer can be turned on or off, produce single-tone alarms, or repeat tone patterns.

## Additional Notes:
- Ensure proper initialization of the buzzer pins before using the methods.
- Adjust timing parameters and tone patterns according to specific requirements and hardware configurations.

## Example:
```cpp
Buzzer buzzer;
buzzer.Begin(true); // Initializes the buzzer and sets it active
buzzer.SingelOn(1000, 500); // Plays a single-tone buzzer for 1 second with a 0.5-second pause
int repeats[] = {3};
buzzer.Repead(repeats, 200, 300); // Repeats a tone pattern 3 times with a 0.2-second tone and a 0.3-second pause
```





newEven

# Output Management Functions

## General Overview:
This document describes a namespace called `Output` containing functions for managing LEDs, relays, and buzzers based on various conditions.

## Parameters:
- `lineStatus`: An array of `status` representing the status of each line.
- `powerStatus`: A `powerState` indicating the status of the power supply.
- `buttonStatus`: A pointer to `ButtonState` representing the status of buttons.
- `mainVoltageState`: A boolean indicating whether the main voltage meets a certain threshold.
- `outputAlart`: A boolean indicating the occurrence of an output alert.
- `existenceEarth`: A boolean indicating the existence of an earth connection.

## Examples:
```cpp
// Example usage of LED management function
Output::LEDManagement(lineStatus, powerStatus, &buttonStatus, mainVoltageState, outputAlart, existenceEarth);

// Example usage of buzzer management function
Output::BuzzerManagement(&buttonStatus, &newEvent);

// Example usage of relay management function
Output::RelayManagement(&buttonStatus, &newEvent);


```

# LEDManagement Function Documentation
 Function: 
```cpp
Output::LEDManagement(lineStatus, powerStatus, &buttonStatus, mainVoltageState, outputAlart, existenceEarth);
```
## General Overview:
The `LEDManagement` function, defined within the Output namespace, controls the behavior of LEDs based on various system statuses and conditions.

## Parameters:
- **lineStatus[12]**: An array of status enums representing the status of each power line.
- **powerStatus**: A powerState enum indicating the overall power status of the system.
- **buttonStatus**: A pointer to a ButtonState struct representing the status of buttons.
- **mainVoltageState**: A boolean indicating whether the main voltage is within the acceptable range.
- **outputAlart**: A boolean indicating the occurrence of an output alert.
- **existenceEarth**: A boolean indicating the existence of an earth connection.

## Variables:
- **BLINK_LEDS_ON_TIME**: Constant representing the duration for LED blinking (ON state).
- **BLINK_LEDS_OFF_TIME**: Constant representing the duration for LED blinking (OFF state).
- **FAST_BLINK_LEDS_ON_TIME**: Constant representing the fast duration for LED blinking (ON state).
- **FAST_BLINK_LEDS_OFF_TIME**: Constant representing the fast duration for LED blinking (OFF state).
- **fireTrace[12]**: Array of booleans representing the trace of fire on each line.
- **normalLine**: Boolean indicating the presence of normal lines without fire.



## Control Flow:
- Initializes LED objects.
- Controls LED behavior based on line status, power status, button status, and other conditions.
- Blinks LEDs for open circuit or short circuit statuses.
- Turns on or off fire LEDs based on fire status.
- Handles power status LEDs based on different power states.
- Handles alarm and buzzer LEDs based on button status.
- Enters a loop to indicate LED check if the LED check button is pressed.

## Additional Notes:
- Ensure proper initialization of LED pins before calling this function.
- Adjust blinking durations and other parameters as needed for specific requirements.
## Examples:
```cpp
ButtonState button;
status lineStatus[12];
powerState power;
bool mainVoltageOK = true;
bool alert = false;
bool earthConnected = true;

// Example call:
 Output::LEDManagement(lineStatus, power, &button, mainVoltageOK, alert, earthConnected);
```````

## Returns:
This function does not return any value.



#  BuzzerManagementFunction Documentation
Function:
```cpp
Output::BuzzerManagement(&buttonStatus, &newEvent);
```
## General Overview:
The `BuzzerManagement` function, defined within the Output namespace, controls the behavior of the buzzer based on various events and system states.

## Parameters:
- **buttonStatus**: A pointer to a ButtonState struct representing the state of buttons, including the buzzer button.
- **newEvent**: A pointer to an eventStatus struct representing the latest detected event.

## Variables:
- **BLINK_BUZZER_ON_TIME**: Macro defining the duration for the buzzer to be turned on during blinking.
- **BLINK_BUZZER_OFF_TIME**: Macro defining the duration for the buzzer to be turned off during blinking.
- **UNMUTE**: Macro to set the buzzer status to unmute (buttonStatus->BUZZER = false).
- **MUTE**: Macro to set the buzzer status to mute (buttonStatus->BUZZER = true).
- **lastEvent**: A static eventStatus struct storing the last event processed by the function.
- **RunOnce**: A boolean flag indicating whether the function has run at least once.

## Control Flow:
- Mutes the buzzer if the buzzer button is pressed and the function has not run yet.
- Unmutes the buzzer when the buzzer button is released.
- Handles different types of events:
  - If a line-related event occurs, unmutes the buzzer if the event is different from the last event.
  - If a power-related event occurs, unmutes the buzzer under specific conditions.
  - If a main voltage-related event occurs, unmutes the buzzer.
  - If an output-related event occurs, unmutes the buzzer.
- Turns on the buzzer if there is a fire track detected.
- Sets the buzzer to blink or turn off based on the event state.
- Calls the buzzer's Begin method based on the state of the buzzer button.

## Additional Notes:
- Ensure proper initialization of the lastEvent struct before calling this function.
- Adjust the debug output statements as needed for debugging purposes.

## Examples:
```cpp
ButtonState buttonStatus;
eventStatus newEvent;
// Example call:
Output::BuzzerManagement(&buttonStatus, &newEvent);
``````
## Returns:
- None



# RelayManagemen Function Documentation
 Function:
```cpp 
Output::RelayManagement(&buttonStatus, &newEvent);
```
## Overview:
The `RelayManagement` function, part of the Output namespace, manages the behavior of relays based on the current system state and events.

## Parameters:
- **buttonStatus**: A pointer to a ButtonState struct representing the state of buttons, including the alarm relay button.
- **newEvent**: A pointer to an eventStatus struct representing the latest detected event.

## Variables:
- **rel1**: Pin used to control relay 1.
- **rel2**: Pin used to control relay 2.
- **relo1**: Pin used to control relay output 1.
- **relo2**: Pin used to control relay output 2.

## Control Flow:
- If the system is not in the normal event state, relay 2 is turned on (HIGH); otherwise, it is turned off (LOW).
- If there is a fire track detected and the alarm relay button is not pressed, relay 1 and the relay outputs are turned on (HIGH).
- If the alarm relay button is pressed, relay 1 and the relay outputs are turned on (HIGH).
- If neither of the above conditions is met, relay 1 and the relay outputs are turned off (LOW).

## Additional Notes:
- Ensure proper initialization of the relay pins and other necessary components before calling this function.
- Adjust the behavior according to specific system requirements and configurations.

## Examples:
```cpp
ButtonState buttonStatus;
eventStatus newEvent;
// Example call:
Output::RelayManagement(&buttonStatus, &newEvent);
````````
## Returns:
- None







***********
***********
# `Main Header File Documentation`

# main.h

This C++ header file is part of an Arduino project. It defines various constants, pins, flags, and data structures used in the program.

## `Pins Definitions`:

- Defines pins for relays, LEDs, buttons, analog channels, battery charges, error indicators, and more.

## `Flags and Variables`:

- Several boolean flags indicating the status of different components and conditions in the system.

## `Enumerations`:

- `status`: Represents the status of a power line, such as open circuit, normal, fire, etc.
- `powerState`: Represents the overall power state of the system.

## `Data Arrays`:

- `lineCurrent` and `lineVoltage`: Arrays storing current and voltage values for different power lines.
- `lockFier`: Array of boolean values indicating the lock status of the fire alarm for each line.

## `Structures`:

- `Mux`: Structure to store analog values read from multiple multiplexers.

## Timer Classes:

- `timerMS` and `flowDelay`: Classes for handling timer-related functionalities.

## `External Libraries`:

- Includes various Arduino libraries such as `IWatchdog`, `ShiftRegister74HC595`, and `SoftwareSerial`.

## `Serial Communication`:

- Defines a `SoftwareSerial` instance (`mySerial`) for communication.

## `Global Instances and Objects`:

- Global instances of timers (`batteryCheckTime`, `fierTimer`, `shortCircuitFlow`, `fierFlow`).
- Global variables for managing time, counters, and card-related information.

## `Macros`:

- Debug macros for conditional compilation based on defined debug flags.
- In normal mode, all debug macros are in comment mode. If you need to debug the code, just remove the debug part from the comment and monitor the serial output of the code behavior.
## Examples:
```cpp
//The power debug line has been removed from the comment, you can see the power status and specifications on your serial port.
 #define POWER_CHECK_DEBUG 
// #define LINE_STATUS_DEBUG 
// #define LINE_STATE_DEBUG 
// #define LINE_FIER_DEBUG
// #define LINE_SC_DEBUG
``````


## `Function-Like Macros`:

- Macros for turning on/off power relays and changing supply voltage.

## `Function Declarations`:

- Declarations for functions that might be defined elsewhere.

## `Conditional Compilation`:

- Various `#ifdef` and `#define` statements for conditional compilation based on debug flags.

