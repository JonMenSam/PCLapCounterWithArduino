/*****************************************************************************************
   Slotcar Race Controller for PCLapCounter Software

   (C) Copyright 2016-2018 el.Dude - www.eldude.nl

   Arduino MEGA 2560 based slotcar race controller. Capture start/finish signals,
   controls the power relays as well as any signal LEDs and manages external buttons.

   See http://pclapcounter.be/arduino.html for the input/output protocol.
   Minimum PC Lap Counter version: 5.43

   Author: Gabriel Inäbnit
   Date  : 2016-10-14

   TODO:
   - Multi heat race proper false start and heat end detection
   - disable track call button when race is not active (or change button behaviour)
   - aborting start/restart is bogus

   Revision History
   __________ ____________________ _______________________________________________________
   2025-07-18 JMS                  Six lane pit detection and power control only - no lap counting, no external buttons, no lights
   2019-09-28 Gabrile Inäbnit      Six lanes version
   2019-05-15 Gabriel Inäbnit      PCLC 5.43 - two Arduino modules mode: lap counting only
   2017-05-20 Gabriel Inäbnit      Slimming down functionality and reduce to four lanes
   2017-01-25 Gabriel Inäbnit      Light show pattern functionality
   2017-01-22 Gabriel Inäbnit      LEDs and Relay code refactored with classes
   2017-01-21 Gabriel Inäbnit      Lane detection blackout period added
   2017-01-17 Gabriel Inäbnit      Interrupt to Lane mapping also configured with array
   2017-01-16 Gabriel Inäbnit      Relays NC, r/g/y racer's stand lights, lane mappings
   2016-10-31 Gabriel Inäbnit      Race Clock - Race Finished status (RC2) PCLC v5.40
   2016-10-28 Gabriel Inäbnit      Start/Finish lights on/off/blink depending race status
   2016-10-25 Gabriel Inäbnit      Removed false start init button - no longer needed
   2016-10-24 Gabriel Inäbnit      Fix false start GO command with HW false start enabled
   2016-10-22 Gabriel Inäbnit      HW false start enable/disable, penalty, reset
   2016-10-21 Gabriel Inäbnit      false start detection and penalty procedure
   2016-10-18 Gabriel Inäbnit      external buttons handling added
   2016-10-14 Gabriel Inäbnit      initial version
 *****************************************************************************************/

/*****************************************************************************************
   Do not use pins:
   Serial1: 18 & 19 - used for interrupts
   Serial2: 16 & 17
   Serial3: 14 & 15
   BuiltIn: 13 - try to avoid it
 *****************************************************************************************/

/*****************************************************************************************
   Global variables
 *****************************************************************************************/
const long serialSpeed = 19200;                  // 19200;
const unsigned long laneProtectionTime = 3000L;  // 3 seconds protection time

#define PITENTRYTIME (500)
#define DETECTIONPOLLING (150)  // 150 ms

const byte laneToInterrupMapping[] = { 2, 3, 20, 21, 18, 19 };
const byte laneToRelayMapping[] = { 10, 11, 12, 13, 14, 15 };

const char lapTime[][7] = { "[SF01$", "[SF02$", "[SF03$", "[SF04$", "[SF05$", "[SF06$" };
const char pitEntry[][7] = { "[PI01]", "[PI02]", "[PI03]", "[PI04]", "[PI05]", "[PI06]" };
const char pitExit[][7] = { "[PO01]", "[PO02]", "[PO03]", "[PO04]", "[PO05]", "[PO06]" };


/*****************************************************************************************
   Pin Naming
 *****************************************************************************************/
// lane to interrup pin mapping
#define LANE_1 laneToInterrupMapping[0]
#define LANE_2 laneToInterrupMapping[1]
#define LANE_3 laneToInterrupMapping[2]
#define LANE_4 laneToInterrupMapping[3]
#define LANE_5 laneToInterrupMapping[4]
#define LANE_6 laneToInterrupMapping[5]

/*****************************************************************************************
   PC Lap Counter Messages
 *****************************************************************************************/
#define PWR_ON "PW001"
#define PWR_OFF "PW000"
#define PWR_1_ON "PW011"
#define PWR_1_OFF "PW010"
#define PWR_2_ON "PW021"
#define PWR_2_OFF "PW020"
#define PWR_3_ON "PW031"
#define PWR_3_OFF "PW030"
#define PWR_4_ON "PW041"
#define PWR_4_OFF "PW040"
#define PWR_5_ON "PW051"
#define PWR_5_OFF "PW050"
#define PWR_6_ON "PW061"
#define PWR_6_OFF "PW060"

/*****************************************************************************************
   Class Lane
 *****************************************************************************************/
class Lane {
protected:
  volatile unsigned long start;
  volatile unsigned long finish;
  volatile unsigned long now;
  volatile long count;
  volatile bool reported;
  volatile bool reportedPitEntry;
  volatile bool reportedPitExit;
  byte lane;
  byte pin;
  bool falseStart;

  bool isInPit;
  bool laneDetect;
  volatile bool detectionStart;
  volatile unsigned long detectionStartTime;
  volatile unsigned long detectionTime;
  //volatile unsigned long laptime;
public:
  Lane(byte setLane) {
    //start = 0L;
    //finish = 0L;
    lane = setLane - 1;
    pin = laneToRelayMapping[lane];
    //reported = true;
    reportedPitEntry = true;
    reportedPitExit = true;
    //falseStart = false;
    detectionStart = false;
  }

  void startDetection() {
    detectionStartTime = millis();
    if (false == isInPit) {
      if (!detectionStart) {
        detectionStart = true;
      }
    }
  }

  void stopDetection() {
    if (true == isInPit) {
      reportedPitExit = false;
    }
    isInPit = false;
    detectionStart = false;
  }

  void checkLapOrPit() {
    if (true == detectionStart) {  // si se inicio una detección
      now = millis();
      //Serial.print("now: " + String(now) + "detectionStart: " + String(detectionStartTime)+ "\n");
      detectionTime = (now - detectionStartTime);
      if (detectionTime < DETECTIONPOLLING) {
        return;
      }
      if (true == isInPit) {
        //Serial.print("sigeu dentro\n");
      } else if (detectionTime > PITENTRYTIME) {  // Ya ha pasado el tiempo de detección de entrada
                                                  /*Serial.print("St: ");
      Serial.println(detectionStartTime);
      Serial.print(">now: ");
      Serial.println(now);
      Serial.print(">Dt: ");
      Serial.println(detectionTime);*/
        reportedPitEntry = false;
        isInPit = true;
        detectionStart = false;
      }
    }
  }

  void lapDetected() {  // called by ISR, short and sweet
    now = millis();
    if ((now - finish) < laneProtectionTime) {
      return;
    }
    start = finish;
    finish = now;
    count++;
    reported = false;
  }
  void reset() {
    reported = true;
    count = -1L;
  }

  void reportPitEntry() {
    if (!reportedPitEntry) {
      Serial.println(pitEntry[lane]);
      reportedPitEntry = true;
    }
    // Serial.print(finish - start);
    // Serial.println(']');
  }

  void reportPitExit() {
    if (!reportedPitExit) {
      Serial.println(pitExit[lane]);
      reportedPitExit = true;
    }
    // Serial.print(finish - start);
    // Serial.println(']');
  }

  /* void reportLap() {
    //if (!reported) {
    //  Serial.print(lapTime[lane]);
    //  Serial.print(finish - start);
    //  Serial.println(']');
    //  reported = true;
    //}

    if (race.isFalseStartEnabled()) {
      if (race.isInit() && !falseStart && (count == 0)) {
        // false start detected,
        // switching lane off immediately
        powerOff();
        falseStart = true;
        race.setFalseStartDetected();  // burn the race fuse
      }
      // switch power back on after false start penalty served
      if (falseStart && race.isFalseStartPenaltyServed()) {
        falseStart = false;  // reset false start lane "fuse"
        powerOn();
      }
    }
  }*/

  void
  powerOn() {
    if (!falseStart) {
      digitalWrite(pin, HIGH);
    }
  }
  void powerOff() {
    digitalWrite(pin, LOW);
  }

  bool isFalseStart() {
    return falseStart;
  }
};

/*****************************************************************************************
   Class Lane instantiations
 *****************************************************************************************/
Lane lane1(1);
Lane lane2(2);
Lane lane3(3);
Lane lane4(4);
Lane lane5(5);
Lane lane6(6);


/*****************************************************************************************
   enable interrupts
 *****************************************************************************************/
#define ISRDETECTIONSIDE (CHANGE)  //(RISING)  //(FALLING)

void attachAllInterrupts() {
  attachInterrupt(digitalPinToInterrupt(LANE_1), lapDetected1, ISRDETECTIONSIDE);
  attachInterrupt(digitalPinToInterrupt(LANE_2), lapDetected2, ISRDETECTIONSIDE);
  attachInterrupt(digitalPinToInterrupt(LANE_3), lapDetected3, ISRDETECTIONSIDE);
  attachInterrupt(digitalPinToInterrupt(LANE_4), lapDetected4, ISRDETECTIONSIDE);
  attachInterrupt(digitalPinToInterrupt(LANE_5), lapDetected5, ISRDETECTIONSIDE);
  attachInterrupt(digitalPinToInterrupt(LANE_6), lapDetected6, ISRDETECTIONSIDE);
}

/*****************************************************************************************
   disable interrupts
 *****************************************************************************************/
void detachAllInterrupts() {
  detachInterrupt(digitalPinToInterrupt(LANE_1));
  detachInterrupt(digitalPinToInterrupt(LANE_2));
  detachInterrupt(digitalPinToInterrupt(LANE_3));
  detachInterrupt(digitalPinToInterrupt(LANE_4));
  detachInterrupt(digitalPinToInterrupt(LANE_5));
  detachInterrupt(digitalPinToInterrupt(LANE_6));
}

/*****************************************************************************************
   initializations and configurations of I/O pins
 *****************************************************************************************/
void setup() {
  // initialize serial communication
  Serial.begin(serialSpeed);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }
  // interrup pins
  pinMode(LANE_1, INPUT_PULLUP);
  pinMode(LANE_2, INPUT_PULLUP);
  pinMode(LANE_3, INPUT_PULLUP);
  pinMode(LANE_4, INPUT_PULLUP);
  pinMode(LANE_5, INPUT_PULLUP);
  pinMode(LANE_6, INPUT_PULLUP);

  // shake the dust off the relays
  jiggleRelays();
  delay(333);
  setPowerOn();  // switch all power relays on

  lapDetected1();
}

/*****************************************************************************************
   relays initialization - shake the dust off the contacts
 *****************************************************************************************/
#define CLICK 20

void jiggleRelays() {
  allRelaysOn();
  delay(CLICK);
  allRelaysOff();
  delay(222);
  allRelaysOn();
  delay(CLICK);
  allRelaysOff();
  delay(111);
  allRelaysOn();
  delay(CLICK);
  allRelaysOff();
  delay(111);
  allRelaysOn();
  delay(CLICK);
  allRelaysOff();
  delay(222);
  allRelaysOn();
  delay(CLICK);
  allRelaysOff();
  delay(444);
  allRelaysOn();
  delay(CLICK);
  allRelaysOff();
  delay(222);
  allRelaysOn();
  delay(CLICK);
  allRelaysOff();
}

/*****************************************************************************************
   Class Relay
 *****************************************************************************************/
class Relay {
protected:
  byte pin;
public:
  Relay(byte lane) {
    pin = laneToRelayMapping[lane - 1];
    pinMode(pin, OUTPUT);
  }
  void on() {
    digitalWrite(pin, HIGH);
  }
  void off() {
    digitalWrite(pin, LOW);
  }
};

Relay relay1(1);
Relay relay2(2);
Relay relay3(3);
Relay relay4(4);
Relay relay5(5);
Relay relay6(6);

/*****************************************************************************************
   engage/disengage relays
 *****************************************************************************************/
void allRelaysOn() {
  relay1.on();
  relay2.on();
  relay3.on();
  relay4.on();
  relay5.on();
  relay6.on();
}

void allRelaysOff() {
  relay1.off();
  relay2.off();
  relay3.off();
  relay4.off();
  relay5.off();
  relay6.off();
}

void setPowerOn() {
  // ledPowerAll.on();
  allRelaysOn();
  // setLEDsPowerOn();
}

void setPowerOff() {
  // ledPowerAll.off();
  allRelaysOff();
  // setLEDsPowerOff();
}

/*****************************************************************************************
   Interrup Service Routines (ISR) definitions
 *****************************************************************************************/
void lapDetected1() {
  if (HIGH == digitalRead(LANE_1)) {
    //Serial.println("1H");
    lane1.startDetection();
  } else {
    //Serial.println("1L");
    lane1.stopDetection();
  }
}
void lapDetected2() {
  if (HIGH == digitalRead(LANE_2)) {
    //Serial.println("2H");
    lane2.startDetection();
  } else {
    //Serial.println("2L");
    lane2.stopDetection();
  }
}
void lapDetected3() {
  if (HIGH == digitalRead(LANE_3)) {
    //Serial.println("3H");
    lane3.startDetection();
  } else {
    //Serial.println("3L");
    lane3.stopDetection();
  }
}
void lapDetected4() {
  if (HIGH == digitalRead(LANE_4)) {
    //Serial.println("4H");
    lane4.startDetection();
  } else {
    //Serial.println("4L");
    lane4.stopDetection();
  }
}
void lapDetected5() {
  if (HIGH == digitalRead(LANE_5)) {
    //Serial.println("5H");
    lane5.startDetection();
  } else {
    //Serial.println("5L");
    lane5.stopDetection();
  }
}
void lapDetected6() {
  if (HIGH == digitalRead(LANE_6)) {
    //Serial.println("6H");
    lane6.startDetection();
  } else {
    //Serial.println("6L");
    lane6.stopDetection();
  }
}

/*****************************************************************************************
   Main loop
 *****************************************************************************************/
void loop() {
  detachAllInterrupts();
  while (Serial.available()) {
    Serial.readStringUntil('[');
    {
      String output = Serial.readStringUntil(']');
      String raceClockState = output.substring(0, 3);  // RC#

      if (output == PWR_OFF) {
        //ledPowerAll.off();
        //if (race.isFinished()) {
        setPowerOff();
        //}
        //if (race.isPaused()) {
        //ledCaution.on();
        //}
      } else if (output == PWR_1_ON) {
        lane1.powerOn();
      } else if (output == PWR_1_OFF) {
        lane1.powerOff();
      } else if (output == PWR_2_ON) {
        lane2.powerOn();
      } else if (output == PWR_2_OFF) {
        lane2.powerOff();
      } else if (output == PWR_3_ON) {
        lane3.powerOn();
      } else if (output == PWR_3_OFF) {
        lane3.powerOff();
      } else if (output == PWR_4_ON) {
        lane4.powerOn();
      } else if (output == PWR_4_OFF) {
        lane4.powerOff();
      } else if (output == PWR_5_ON) {
        lane5.powerOn();
      } else if (output == PWR_5_OFF) {
        lane5.powerOff();
      } else if (output == PWR_6_ON) {
        lane6.powerOn();
      } else if (output == PWR_6_OFF) {
        lane6.powerOff();
      }
    }
  }
  lane1.checkLapOrPit();
  lane2.checkLapOrPit();
  lane3.checkLapOrPit();
  lane4.checkLapOrPit();
  lane5.checkLapOrPit();
  lane6.checkLapOrPit();

  lane1.reportPitEntry();
  lane2.reportPitEntry();
  lane3.reportPitEntry();
  lane4.reportPitEntry();
  lane5.reportPitEntry();
  lane6.reportPitEntry();

  lane1.reportPitExit();
  lane2.reportPitExit();
  lane3.reportPitExit();
  lane4.reportPitExit();
  lane5.reportPitExit();
  lane6.reportPitExit();

  attachAllInterrupts();
}
