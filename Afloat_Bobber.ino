
/*
Fabriek der dingen - Afloat bobber v1 Arduino sketch
Author: Freek Bossong
Sketch desciption: Sends signal to Sigfox backend to trigger callback if water is detected form l
iquid level sensor. Normal state is asleep. The device can be woken up from two interrupt pins: 
INT1 is the interrupt from the liquid level sensor, INT0 is the interrupt from the RTC. If woken up 
from the liquid level sensor it can be due to two states: water detected or no water detected anymore. 
Water detected is HIGH and no water detected anymore is LOW. To error out false positives when water is detected the
liquid level sensor needs to be at least 10 seconds on HIGH tot trigger the signal to Sigfox.
When water is detected and the signal is send, the MCU goes back to sleep. If the water level
drops again to cause a LOW on the liquid level sensor the MCU sends a different signal.
Uses a TD1207 Sigfox transceiver via UART (hardware) serial communication.
Codes:
1 = no water detected (anymore)
2 = water detected
3 = one time startup coverage signal
4 = daily alive signal
5 = OK to pair (not yet implemented in version v1)
Data consist of code (int) and battery voltage * 100 (hex). I.e. 1171 = code 1 and 369 = 3.69 volt (171).
Daiy alive message is send at 17.00 using RTC alarm trigger via INT0.

Pairing
To pair the device the user needs to place a small magnet on top of the casing marked with pairing icon. 
This triggers the reed switch for pairing on INT0. To pair the device sends his ID code and tells the server that 
its ok to pair this device with a user.
*/

//LIBRARIES

#include <Wire.h>
#include <DS3231.h>
#include <avr/sleep.h>
#include <EEPROM.h>
#include <TheAirBoard.h>

//GLOBALS

#define BLUE 6              // Blue LED for debugging
int reedPin = 3;            // Pin for reed switch - Pin 3 is INT1
int reedState = 0;          // Default state
int counter = 0;            // Counter value for reducing false positives
int sensor;                 // Input value water level sensor for takeReading
char buffer[9];             // Buffer with value for sigfox modem
int addr = 0;               // EEPROM memory position
byte value;                 // EEPROM value

//DS3231 RTC
DS3231 clock;
RTCDateTime dt;

//TheAirboard: Only for battery voltage
TheAirBoard board;

void setup() {
  Serial.begin(9600);

  //RTC initialize and setup Alarm1
  Serial.println("Initialize DS3231");
  delay(100);
  clock.begin();
  clock.armAlarm2(false);
  clock.clearAlarm2();
  // Set Alarm1 - Every 17.00 on each day
  // Format: setAlarm1(Date or Day, Hour, Minute, Second, Mode, Armed = true)
  clock.setAlarm2(0, 15, 54, DS3231_MATCH_H_M);
  // Check alarm settings
  checkAlarms();

  //Show last reading form EEPROM memory
  value = EEPROM.read(addr);

  pinMode(reedPin, INPUT_PULLUP);               // Input reed switch with input pullup
  pinMode(BLUE, OUTPUT);                        // Blue led for debugging

  attachInterrupt(1, wakeUpNow, CHANGE);        // Attach INT1 to the reed switch to wake MCU
  attachInterrupt(0, wakeUpNow, RISING);        // Attach INT0 to the pairing magnet switch
  
  //Send one time start signal to Sigfox to check coverage
  //  digitalWrite(BLUE, 1);                    // set communication indicator
  String batString = String(board.batteryChk() * 100, 0);
  Serial.print("AT$SS=");                       // Send battery state and indicator '1' as HEX
  Serial.print(3);
  Serial.println(batString.toInt(), HEX);
  delay(6000);                                  // wait until end of transmission
  digitalWrite(BLUE, 0);                        // reset communication indicator
  //

}
//Checks for alarms
void checkAlarms()
{
  //RTCAlarmTime a1;
  RTCAlarmTime a2;

}
void wakeUpNow() {
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
  //Serial.println("Woke up");
  //delay(100);
}
void sleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // sleep mode is set here
  sleep_enable();                         // enables the sleep bit in the mcucr register
  attachInterrupt(1, wakeUpNow, CHANGE); // Liquid level interrupt. Use interrupt 1 (pin 3) and run function
  attachInterrupt(0, wakeUpNow, RISING); // Pairing interrupt. Use interrupt 0 (pin 2) and run function
  sleep_mode();                           // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep: disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the wakeUpNow code will not be executed during normal running time.
  detachInterrupt(1);      // disables interrupt 1 on pin 3 so the wakeUpNow code will not be executed during normal running time.
}

void loop() {
  // Send daily alive message when alarm2 sends interrupt
  if (clock.isAlarm2()) {

    //Start communication steps
    Serial.println("Send daily alive message");
    analogWrite(BLUE, 1);                                      // Set communication indicator
    String batString = String(board.batteryChk() * 100, 0);    // Get battery voltage and set value without decimal
    Serial.print("AT$SS=");                                    // Send battery state and indicator '3' as HEX
    Serial.print(4);
    Serial.println(batString.toInt(), HEX);
    delay(6000);                                               // wait until end of transmission
    digitalWrite(BLUE, 0);                                     // reset communication indicator
  }

  //Pairing
  //Check state of pairSwitch. If HIGH more than 10 seconds, send 'OK to pair' signal to backend
  //....[code]
  

  
  // If INT1 has an interrupt check if it is due to water detection on no water detected anymore.
  // reedState give the state of the reed switch. HIGH is water detected, LOW is no water detected
  reedState = digitalRead(reedPin);
  if (reedState == LOW) {
    //Pin state is low >> no water detected (anymore)
    digitalWrite(BLUE, LOW);                    //Debug
    Serial.println("Woke from LOW on INT1");    //Debug

    //Check for false positives
    //Measure for at least 10 seconds to prevent false positive
    for (int i = 0; i < 10; i++) {
      sensor =  digitalRead(reedPin); //Read input value
      if (sensor == LOW) {
        counter ++;
      }
      delay(1000);
    }

    //Serial.println(counter);      //Debug

    //Send signal to Sigfox backend for callback trigger to service
    //when previously water detected
    if ((counter == 10) && (value == 2) || (sensor == LOW)) {

      //Reset counter
      Serial.println("Send to Sigfox backend (No water detected (anymore))");

      //Send value
      digitalWrite(BLUE, 1);                                    // set communication indicator
      String batString = String(board.batteryChk() * 100, 0);   // Get battery voltage and set value without decimal
      Serial.print("AT$SS=");                                   // Send battery state and indicator '1' as HEX
      Serial.print(1);
      Serial.println(batString.toInt(), HEX);
      delay(6000);                                              // wait until end of transmission
      digitalWrite(BLUE, 0);                                    // reset communication indicator

      //Put value in EEPROM
      EEPROM.write(addr, 1);
      delay(1000);
    }
    //Previously already no water detected. No need to send extra signal
    else if ((counter == 10) && (value == 1))  {
      Serial.println("No change in water level detection");
    }
    //False reading
    else {
      Serial.println("False reading");
    }
    //Reset counter
    counter = 0;
  }
  else {
    //Pin state is high >> water detected
    //digitalWrite(BLUE, HIGH);
    Serial.println("Woke from HIGH on INT1");

    //Check for false positives
    //Measure for at least 10 seconds
    for (int i = 0; i < 10; i++) {
      sensor =  digitalRead(reedPin); //Read input value
      if (sensor == HIGH) {
        counter ++;
      }
      delay(1000);
    }

    Serial.println(counter);

    //Send signal to Sigfox backend for callback trigger to service
    if ((counter == 10) && (value == 1) || (sensor == HIGH) ) {
      //Reset counter
      counter = 0;
      Serial.println("Send to Sigfox backend (Water detected)");

      //Send value
      digitalWrite(BLUE, 1);                                   // set communication indicator
      String batString = String(board.batteryChk() * 100, 0);   // Get battery voltage and set value without decimal
      Serial.print("AT$SS=");                                  // Send battery state and indicator '1' as HEX
      Serial.print(2);
      Serial.println(batString.toInt(), HEX);
      delay(6000);                                            // wait until end of transmission
      digitalWrite(BLUE, 0);                                  // reset communication indicator

      //Put value in EEPROM
      EEPROM.write(addr, 2);
      delay(1000);

    }
    //Previously already water detected. No need to send extra signal
    else if ((counter == 10) && (value == 2)) {
      Serial.println("No change in water level detection");
    }
    //False reading
    else {
      Serial.println("False reading");
    }
    //Reset counter
    counter = 0;

  }

  //Go back to sleep mode
  Serial.println("Go back to sleep");
  delay(100);
  sleepNow();
}

