#include <IWatchdog.h>
#include <ShiftRegister74HC595.h>
#include <Main.h>
#include <SoftwareSerial.h>
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

  if (digitalRead(CS1) == 0)
    card1 = true;
  if (digitalRead(CS2) == 0)
    card2 = true;
  if (card1 & card2) {
    cardsit = 2;
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
  } else if (card1) {
    cardsit = 1;
    digitalWrite(Line1, HIGH);
    digitalWrite(Line2, HIGH);
    digitalWrite(Line3, HIGH);
    digitalWrite(Line4, HIGH);
    digitalWrite(Line5, HIGH);
    digitalWrite(Line6, HIGH);
    digitalWrite(Line7, HIGH);
    digitalWrite(Line8, HIGH);
  } else {
    cardsit = 0;
    digitalWrite(Line1, HIGH);
    digitalWrite(Line2, HIGH);
    digitalWrite(Line3, HIGH);
    digitalWrite(Line4, HIGH);
  }


  IWatchdog.begin(5000000);  // 5000ms
  sctime = time1;
  
  digitalWrite(LEDerror, HIGH);
  digitalWrite(Sela, LOW);
  digitalWrite(Selb, LOW);
  digitalWrite(Selc, LOW);
  delay(25);
  mux4[0] = (3.3 / 1023.00) * analogRead(Analog4);
  delay(15);
  digitalWrite(Sela, HIGH);
  digitalWrite(Selb, HIGH);
  digitalWrite(Selc, LOW);
  delay(25);
  mux4[3] = (3.3 / 1023.00) * analogRead(Analog4);
}

void loop() {

  digitalWrite(LEDerror, HIGH);
  batpowerchek1();
  buttonchek();
  Muxread(muxpos);
  Linechek();

  for (byte i = 0; i < ((cardsit + 1) * 4); i++) {

    if ((lvoltage[i] < sctreshold) && (time1 - sctime > 1)) {

      printVoltageAlert(i, lvoltage[i]);

    } else {
      processCurrentConditions(i);
    }
  }
  handleThresholdFaults();
  handleSupplyAndPowerFailures();
  handleCardPresentErrors();

  if (timeForLedBlink()) {
    toggleLedState();
    Ledcontrol();
  }
  Relaycont();
  IWatchdog.reload();
  updateMuxPosition();
  checkAndEnableBeeper();
}



// Function to check if a threshold fault is detected
bool thresholdFaultDetected() {
  return ((mux4[4] > uperthresholdout) || (mux4[4] < lowerthresholdout) || (mux4[5] > uperthresholdout) || (mux4[5] < lowerthresholdout));
}

// Function to handle threshold faults
void handleThresholdFaults() {
  if (thresholdFaultDetected() && !firetrac && !relycontroll) {
    if (genfault) genfault = false;
    relo = true;
    buzcont = true;
  } else {
    relo = false;
  }
}
// Function to check if a supply failure is detected
bool supplyFailureDetected() {
  return (0.55 < mux4[0]);
}
// Function to check if a power failure is detected

bool powerFailureDetected() {
  return (mux4[0] < 0.1);
}

// Function to handle supply and power failures
void handleSupplyAndPowerFailures() {
  if (supplyFailureDetected()) {
    supplyfault = false;
    buzcont = true;
    batcheking = false;
  } else {
    if (!batcheking)
      supplyfault = true;
    else
      batcheking = false;
  }

  if (powerFailureDetected()) {
    powerfail = true;
    buzcont = true;
  } else {
    powerfail = false;
  }
}
// Function to check if it's time to toggle the LED
bool timeForLedBlink() {
  return (time1 - ledblink) > 6;
}

void batpowerchek1() {
  batcheking = true;
  float vp[4] = { 0, 0, 0, 0 };
  byte cont = 0;
  if (!relycharge) {
    if ((mux4[3] * 24) > 18.6) {
      relycharge = true;
      digitalWrite(Batcharges, relycharge);
    }
  }
  if (relycharge) {
    if (time1 - batscan > 50) {
      if ((mux4[0] * 41) > 16) {
        digitalWrite(Sela, LOW);
        digitalWrite(Selb, LOW);
        digitalWrite(Selc, LOW);
        batscan = time1;
        digitalWrite(ChangeVolt, HIGH);
        delay(55);
        do {
          mux4[0] = (3.3 / 1023.00) * analogRead(Analog4);
          vp[cont] = mux4[0];
          cont++;
          delay(13);
          if (cont > 2) {
            if (((vp[0] - vp[1]) < 0.02) && ((vp[1] - vp[2]) < 0.02))
              break;
            cont = 0;
          }
        } while ((time1 - batscan) > 21);
        digitalWrite(Sela, HIGH);
        digitalWrite(Selb, HIGH);
        digitalWrite(Selc, LOW);
        delay(21);
        mux4[3] = (3.3 / 1023.00) * analogRead(Analog4);
        batfail = false;
        delay(13);
        if ((mux4[3] * 24) < 18.6) {
          relycharge = false;
          batfail = true;
          digitalWrite(Batcharges, relycharge);
        }
        digitalWrite(ChangeVolt, LOW);
      }
    }
    if ((mux4[0] * 41) < 15.6) {
      if ((mux4[3] * 24) < 15.6) {
        relycharge = false;
        batfail = true;
        digitalWrite(Batcharges, relycharge);
      } else if ((mux4[3] * 24) < 16.6) {
        batlowvolt = true;
        digitalWrite(MCUbuzz, HIGH);
      } else {
        batlowvolt = false;
      }
    }
  }
  delay(10);
  batfail = !relycharge;
  powerfail = false;
}
// Function to print voltage alert
void printVoltageAlert(byte line, float voltage) {
  mySerial.print("line");
  mySerial.print(line);
  mySerial.print(",");
  mySerial.println(voltage);
}

void Ledcontrol() {
  for (byte i = 0; i < 12; i++) {
    if ((linesituation[i] == 1) || (linesituation[i] == 4)) {  // Fault mode
      sr.set(lederrors[i], blinkerl);
      if (buzcont && !genfault && !firetrac)
        digitalWrite(MCUbuzz, blinkerl);
      else
        digitalWrite(MCUbuzz, LOW);
    } else if (linesituation[i] == 3) {  // Fire mode
      sr.set(ledfire[i], blinkerl);
      sr.set(ledefiremode, LOW);
      if (firetrac)
        digitalWrite(MCUbuzz, HIGH);
      else
        digitalWrite(MCUbuzz, LOW);
    } else {
      sr.set(lederrors[i], HIGH);
      sr.set(ledfire[i], HIGH);
      if ((batfail || powerfail || supplyfault) && !genfault)
        digitalWrite(MCUbuzz, blinkerl);
      else
        digitalWrite(MCUbuzz, LOW);
    }
    if (firetrac && !genfault)
      digitalWrite(MCUbuzz, HIGH);
  }
  if ((genfault && firetrac) || (batfail || powerfail || supplyfault) && genfault)
    sr.set(ledebuz, LOW);
  else
    sr.set(ledebuz, HIGH);
  if (supplyfault && !powerfail)
    sr.set(ledepower, LOW);
  else
    sr.set(ledepower, HIGH);
  if (batfail)
    sr.set(ledebat, LOW);
  else
    sr.set(ledebat, HIGH);
  if (batlowvolt) {
    sr.set(ledebat, blinkerl);
    sr.set(ledemainpower, blinkerl);
  }
  if (powerfail)
    sr.set(ledemainpower, LOW);
  else
    sr.set(ledemainpower, HIGH);
  if (relo)
    sr.set(ledeearth, LOW);
  else
    sr.set(ledeearth, HIGH);
  if (firetrac)
    sr.set(ledefiremode, LOW);
  else
    sr.set(ledefiremode, HIGH);
  sr.set(panelon, LOW);
  if (relo) {
    // mySerial.println("relo ok");
    if (!genfault)
      digitalWrite(MCUbuzz, relo);
  }
  sr.set(generalfault, !(supplyfault || batfail || powerfail || relo));
}


// Function to toggle the LED state
void toggleLedState() {
  ledblink = time1;
  blinkerl = !blinkerl;
}

// Function to update the mux position for analog readings
void updateMuxPosition() {
  if (muxpos < 8) {
    if (muxpos == 7) {
      muxpos = 0;
      freadanalogs = true;
    } else {
      muxpos++;
    }
  }
}

// Function to check and enable the beeper
void checkAndEnableBeeper() {
  if (enableBeeper()) {
    buzzready = time1;
    beeper = true;
  }
}

// Function to check if the beeper should be enabled
bool enableBeeper() {
  return (buzcont && genfault && !firetrac && (time1 - buzzready > 300));
}








// Function to process current conditions
void processCurrentConditions(byte line) {
  // Process the current conditions for the line
  if ((lcurrent[line] < opentreshold) && firstsence[line] == 0) {
    linesituation[line] = 1;
  } else if ((opentreshold < lcurrent[line]) && (lcurrent[line] < normaltreshold) && firstsence[line] == 0) {
    linesituation[line] = 2;
  } else if ((normaltreshold < lcurrent[line]) && (lcurrent[line] < firetreshold)) {
    // Handle fire detection conditions
    fsencetimer = 0;  // fire alarming
    delay(55);
    if (firstsence[line] == 1) {
      linesituation[line] = 0;
      digitalWrite(linecontrol[line], LOW);
      delay(55);
    } else if (firstsence[line] == 2) {
      linesituation[line] = 0;
      digitalWrite(linecontrol[line], HIGH);
    } else if (firstsence[line] == 3) {
      linesituation[line] = 3;
      firetrac = true;
      fireflag = true;
      relycontroll = false;
      relycustomon = false;
      sr.set(ledebuz, HIGH);
      sr.set(ledesounder, HIGH);
    }
    firstsence[line] = firstsence[line] + 1;
    delay(55);
  } else {
    if (firstsence[line] == 0)
      linesituation[line] = 4;
  }
}

// Function to check if all values in a range are zero
bool allZerosInRange(int start, int end) {
  for (int i = start; i <= end; i++) {
    if (lcurrent[i] != 0) {
      return false;
    }
  }
  return true;
}



// Function to handle card present errors
void handleCardPresentErrors() {
  if (cardsit == 1) {
    if (allZerosInRange(4, 7)) {
      cardpresenterror = 1;
    } else {
      cardpresenterror = 0;
    }
  }
  if (cardsit == 2) {
    if (allZerosInRange(4, 7) && allZerosInRange(8, 11)) {
      cardpresenterror = 3;
    } else if (allZerosInRange(8, 11)) {
      cardpresenterror = 2;
    } else {
      cardpresenterror = 0;
    }
  }
}



void Linechek() {

  switch (cardsit) {
    case 0:
      lvoltage[3] = mux1[0];
      lcurrent[3] = mux1[3];
      lcurrent[2] = mux1[2];
      lvoltage[2] = mux1[1];
      lcurrent[1] = mux1[4];
      lvoltage[1] = mux1[6];
      lvoltage[0] = mux1[5];
      lcurrent[0] = mux1[7];
      // mySerial.print(lcurrent[i]);
      break;
    case 1:
      lvoltage[3] = mux1[0];
      lcurrent[3] = mux1[3];
      lcurrent[2] = mux1[2];
      lvoltage[2] = mux1[1];
      lcurrent[1] = mux1[4];
      lvoltage[1] = mux1[6];
      lvoltage[0] = mux1[5];
      lcurrent[0] = mux1[7];

      lvoltage[7] = mux2[0];
      lcurrent[7] = mux2[3];
      lcurrent[6] = mux2[1];
      lvoltage[6] = mux2[2];
      lcurrent[5] = mux2[6];
      lvoltage[5] = mux2[4];
      lvoltage[4] = mux2[5];
      lcurrent[4] = mux2[7];
      break;
    case 2:
      lvoltage[3] = mux1[0];
      lcurrent[3] = mux1[3];
      lcurrent[2] = mux1[2];
      lvoltage[2] = mux1[1];
      lcurrent[1] = mux1[4];
      lvoltage[1] = mux1[6];
      lvoltage[0] = mux1[5];
      lcurrent[0] = mux1[7];

      lvoltage[7] = mux2[0];
      lcurrent[7] = mux2[3];
      lcurrent[6] = mux2[1];
      lvoltage[6] = mux2[2];
      lcurrent[5] = mux2[6];
      lvoltage[5] = mux2[4];
      lvoltage[4] = mux2[5];
      lcurrent[4] = mux2[7];

      lvoltage[11] = mux3[0];
      lcurrent[11] = mux3[3];
      lcurrent[10] = mux3[1];
      lvoltage[10] = mux3[2];
      lcurrent[9] = mux3[6];
      lvoltage[9] = mux3[4];
      lvoltage[8] = mux3[5];
      lcurrent[8] = mux3[7];
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
    mux1[add] = (3.3 / 1023.00) * analogRead(Analog1);
    mux2[add] = (3.3 / 1023.00) * analogRead(Analog2);
    mux3[add] = (3.3 / 1023.00) * analogRead(Analog3);
    mux4[add] = (3.3 / 1023.00) * analogRead(Analog4);
  }

  // Delay as needed
  delay(15);
}


void buttonchek() {
  if (digitalRead(But5) == 0) {  // Buzzer off
    if (time1 - buttontime > 10) {
      buttontime = time1;
      genfault = !genfault;
    }
  }
  if (digitalRead(But4) == 0) {  // LED chek
    sr.setAllLow();
    delay(550);
    byte initi = 0;
    initi = cardpresenterror;
    cardpresenterror = 0;
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
    cardpresenterror = initi;
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
      mux1[i] = 0;
      mux2[i] = 0;
      mux3[i] = 0;
      mux4[i] = 0;
    }
    for (byte i = 0; i < 12; i++) {
      lcurrent[i] = 0;
      lvoltage[i] = 0;
      linesituation[i] = 0;      // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
      lastlinesituation[i] = 0;  // 1=open line error, 2=normal line, 3=fire line, 4=short circut line
      firstsence[i] = 0;
      scdetected[i] = 0;
    }
    limittimesc = 3;
    muxpos = 0;
    // cardsit = 0;
    time1 = 0;
    ledblink = 0;
    buttontime = 0;
    sctime = 0;
    buzzready = 0;
    fsencetimer = 0;
    blinkerl = true;
    blinkerl2 = true;
    buzcont = false;
    sounderled = false;
    supplyfault = false;
    batfail = false;
    powerfail = false;
    eartfail = false;
    genfault = false;
    firetrac = false;
    relycontroll = false;
    freadanalogs = false;
    buzzerz = false;
    beeper = false;
    relo = false;
    fireflag = false;
    faultflag = false;
    fcounter = 0;
    digitalWrite(MCUbuzz, LOW);
    IWatchdog.reload();
    delay(900);
    IWatchdog.reload();
    if (cardsit == 2) {
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
    } else if (cardsit == 1) {
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
    Muxread(muxpos);
  }
  if (digitalRead(But2) == 0) {  // Alarm rely on
    if (time1 - buttontime > 13) {
      buttontime = time1;
      relycontroll = true;
      sr.set(ledesounder, HIGH);
      //  mySerial.println("alarm on");
      if (sounderled) {
        sounderled = !sounderled;
        relycustomon = false;
      }
    }
  }
  if (digitalRead(But1) == 0) {  // Alarm rely off

    if (time1 - buttontime > 13) {
      buttontime = time1;
      relycontroll = false;
      // mySerial.println("alarm off");
      if (fireflag) {
        fireflag = false;
        sr.set(ledesounder, LOW);
      }
      if (!sounderled)
        sounderled = !sounderled;
      relycustomon = true;
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
  time1++;
  fsencetimer++;
  blinkerl2 = !blinkerl2;
  if (cardpresenterror > 0) {
    if (cardpresenterror == 1) {
      sr.set(lederrors[4], blinkerl2);
      sr.set(lederrors[5], blinkerl2);
      sr.set(lederrors[6], blinkerl2);
      sr.set(lederrors[7], blinkerl2);
    } else if (cardpresenterror == 2) {
      sr.set(lederrors[8], blinkerl2);
      sr.set(lederrors[9], blinkerl2);
      sr.set(lederrors[10], blinkerl2);
      sr.set(lederrors[11], blinkerl2);
    } else if (cardpresenterror == 3) {
      sr.set(lederrors[4], blinkerl2);
      sr.set(lederrors[5], blinkerl2);
      sr.set(lederrors[6], blinkerl2);
      sr.set(lederrors[7], blinkerl2);
      sr.set(lederrors[8], blinkerl2);
      sr.set(lederrors[9], blinkerl2);
      sr.set(lederrors[10], blinkerl2);
      sr.set(lederrors[11], blinkerl2);
    }
  }
  if (fcounter > 0) {
    fcounter = fcounter - 1;
    faultflag = true;
  } else
    faultflag = false;
}

void Relaycont() {
  digitalWrite(rel2, faultflag);
  // mySerial.println(faultflag);
  if (fireflag && !relycustomon) {
    digitalWrite(rel1, HIGH);
    digitalWrite(relo1, HIGH);
    digitalWrite(relo2, HIGH);
  } else {
    if (relycontroll) {
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