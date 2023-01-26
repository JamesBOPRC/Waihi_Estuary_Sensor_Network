/*****************************************************************************
simple_logging.ino
Written By:  Sara Damiano (sdamiano@stroudcenter.org)
Development Environment: PlatformIO
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team

This sketch is an example of logging data to an SD card

DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
*****************************************************************************/

// ==========================================================================
//    Defines for the Arduino IDE
//    In PlatformIO, set these build flags in your platformio.ini
// ==========================================================================

// ==========================================================================
//    Include the base required libraries
// ==========================================================================
#include <Arduino.h>  // The base Arduino library
#include <EnableInterrupt.h>  // for external and pin change interrupts
#include <LoggerBase.h>  // The modular sensors library
//#include <Adafruit_ADS1015.h>
//Adafruit_ADS1115 ads;    /* Use this for the Mayfly because of the onboard 16-bit ADS1115  */


// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The name of this file
const char *sketchName = "simple_logging.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "JD_Omatata_Stream_Deploy";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 5;
// Your logger's timezone.
const int8_t timeZone = 12;  // Eastern Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!


// ==========================================================================
//    Primary Arduino-Based Board and Processor
// ==========================================================================
#include <sensors/ProcessorStats.h>

const long serialBaud = 115200;   // Baud rate for the primary serial port for debugging
const int8_t greenLED = 8;        // MCU pin for the green LED (-1 if not applicable)
const int8_t redLED = 9;          // MCU pin for the red LED (-1 if not applicable)
const int8_t buttonPin = 21;      // MCU pin for a button to use to enter debugging mode  (-1 if not applicable)
const int8_t wakePin = A7;        // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin = -1;     // MCU SD card power pin (-1 if not applicable)
const int8_t sdCardSSPin = 12;      // MCU SD card chip select/slave select pin (must be given!)
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power (-1 if not applicable)

// Create the main processor chip "sensor" - for general metadata
const char *mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);

// ==========================================================================
//    Turbidity sensor settings
// ==========================================================================

//Turbidity sensor pins
//const int ledPin = 4;
//const int ldr_VCC = 18;
const int ldrPinGreen = A0;//this is opposite the led (180 degrees)
const int ldrPinYellow = A2; //this is at 90 degrees from the led
const int LDR_90_Stop = 100; // in NTU
 //of the form mx + c
const float green_coeff_m = -7.137;
const float green_coeff_c = 6390.4;
//linear up to LDR 90 Stop
const float yellow_coeff_m = 1.1578;
const float yellow_coeff_c = -457.5;




// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>  // Includes wrapper functions for Maxim DS3231 RTC

// Create a DS3231 sensor object, using this constructor function:
MaximDS3231 ds3231(1);


// ==========================================================================
//    Settings for Additional Sensors
// ==========================================================================

//=============================================================
// Turbidity sensor as a calculated variable
//===========================================================


float calculateTurbidity180(void)
{



  //char Turb_final = "err";  // Always safest to start with a bad value
  //float Turb_180 = -9999;
  //float Turb_90 = -9999;
    //Serial.print(F(" Calculating Turbidity... "));

    digitalWrite(sensorPowerPin, HIGH); // turn on voltage to LDR sensors
    //digitalWrite(ledPin, HIGH); // turn on LED
    delay(5000); // give the LDR's 5 seconds to adapt to the LED

    //begin data gathering process
    //Replicate 1
    //int ldrStatus_Led1 = ads.readADC_SingleEnded(ldrPinGreen);
    int ldrStatus_Led1 = analogRead(ldrPinGreen);
    //int ldrStatus2_Led1 = ads.readADC_SingleEnded(ldrPinYellow);
    //int ldrStatus2_Led1 = analogRead(ldrPinYellow);
    delay(1000);
    //Replicate 2
    int ldrStatus_Led2 = analogRead(ldrPinGreen);

  //  int ldrStatus2_Led2 = analogRead(ldrPinYellow);
    delay(1000);
    //Replicate 3
    int ldrStatus_Led3 = analogRead(ldrPinGreen);
  //  int ldrStatus2_Led3 = analogRead(ldrPinYellow);

    delay(1000);
    //Replicate 4
    int ldrStatus_Led4 = analogRead(ldrPinGreen);
  //  int ldrStatus2_Led4 = analogRead(ldrPinYellow);

    delay(1000);
    //Replicate 5
    int ldrStatus_Led5 = analogRead(ldrPinGreen);
//    int ldrStatus2_Led5 = analogRead(ldrPinYellow);
    delay(1000);

    //digitalWrite(ledPin, LOW);
    digitalWrite(sensorPowerPin, LOW);
    float LDR_Green_180 = (ldrStatus_Led1 + ldrStatus_Led2 + ldrStatus_Led3 + ldrStatus_Led4 + ldrStatus_Led5)/5;
  //  float LDR_Yellow_90 = (ldrStatus2_Led1 + ldrStatus2_Led2 + ldrStatus2_Led3 + ldrStatus2_Led4 + ldrStatus2_Led5)/5;

    // Serial.print(F(" Sensor 180 value is ... "));
    // Serial.println(LDR_Green_180);
    //
    // Serial.print(F(" Sensor 90 value is ... "));
    // Serial.println(LDR_Yellow_90);

    // float gm = green_coeff_m;
    // float gc = green_coeff_c;
    // float ym = yellow_coeff_m;
    // float yc = yellow_coeff_c;
    //
    // if (LDR_Green_180 != -9999 && LDR_Yellow_90 != -9999) // make sure input is good
    // {
    //   Turb_180 = (gm*LDR_Green_180) + gc;
    //   Turb_90 = (ym*LDR_Yellow_90) + yc;
    // }
    //
    // if (Turb_90 < LDR_90_Stop){
    //   Turb_final = Turb_90;
    // } else if(Turb_90 > LDR_90_Stop && Turb_180 < LDR_90_Stop ){// weird situation - taking average
    //   Turb_final = (Turb_90 + Turb_180)/2 ;
    // } else {//if 90 and 180 greater than threshold, take 180
    //   Turb_final = Turb_180;
    // }

     // if (Turb_final != -9999)  // make sure input is good
     // {
     //        Serial.print(F(" Calculated result is ... "));
     //        Serial.println(Turb_final);
     // }

    //return Turb_final;


    return LDR_Green_180;


}

// Properties of the calculated variable
const uint8_t calculatedVarResolution180 = 2;  // The number of digits after the decimal place
const char *calculatedVarName180 = "Turbidity";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarUnit180 = "NTU";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarCode180 = "Turb";  // A short code for the variable
const char *calculatedVarUUID180 = "12345678-abcd-1234-ef00-1234567890ab";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
Variable *TurbidityVariable180 = new Variable(calculateTurbidity180, calculatedVarResolution180,
                                       calculatedVarName180, calculatedVarUnit180,
                                       calculatedVarCode180, calculatedVarUUID180);





// ==========================================================================
//    Maxim DS18 One Wire Temperature Sensor
// ==========================================================================
#include <sensors/MaximDS18.h>

// OneWire Address [array of 8 hex characters]
// If only using a single sensor on the OneWire bus, you may omit the address
// DeviceAddress OneWireAddress1 = {0x28, 0xFF, 0xBD, 0xBA, 0x81, 0x16, 0x03, 0x0C};
const int8_t OneWirePower = sensorPowerPin;  // Pin to switch power on and off (-1 if unconnected)
const int8_t OneWireBus = 7;  // Pin attached to the OneWire Bus (-1 if unconnected) (D24 = A0)

// Create a Maxim DS18 sensor objects (use this form for a known address)
// MaximDS18 ds18(OneWireAddress1, OneWirePower, OneWireBus);

// Create a Maxim DS18 sensor object (use this form for a single sensor on bus with an unknown address)
MaximDS18 ds18(OneWirePower, OneWireBus);

//=============================================================
// Turbidity sensor as a calculated variable
//===========================================================


float calculateTurbidity90(void)
{



  //char Turb_final = "err";  // Always safest to start with a bad value
  //float Turb_180 = -9999;
  //float Turb_90 = -9999;
    //Serial.print(F(" Calculating Turbidity... "));

    digitalWrite(sensorPowerPin, HIGH); // turn on voltage to LDR sensors
    //digitalWrite(ledPin, HIGH); // turn on LED
    delay(5000); // give the LDR's 5 seconds to adapt to the LED

    //begin data gathering process
    //Replicate 1
    //int ldrStatus_Led1 = ads.readADC_SingleEnded(ldrPinGreen);
    //int ldrStatus_Led1 = analogRead(ldrPinGreen);
    //int ldrStatus2_Led1 = ads.readADC_SingleEnded(ldrPinYellow);
    int ldrStatus2_Led1 = analogRead(ldrPinYellow);
    delay(1000);
    //Replicate 2
    //int ldrStatus_Led2 = analogRead(ldrPinGreen);

    int ldrStatus2_Led2 = analogRead(ldrPinYellow);
    delay(1000);
    //Replicate 3
    //int ldrStatus_Led3 = analogRead(ldrPinGreen);
    int ldrStatus2_Led3 = analogRead(ldrPinYellow);

    delay(1000);
    //Replicate 4
  //  int ldrStatus_Led4 = analogRead(ldrPinGreen);
    int ldrStatus2_Led4 = analogRead(ldrPinYellow);

    delay(1000);
    //Replicate 5
  //  int ldrStatus_Led5 = analogRead(ldrPinGreen);
    int ldrStatus2_Led5 = analogRead(ldrPinYellow);
    delay(1000);

    //digitalWrite(ledPin, LOW);
    digitalWrite(sensorPowerPin, LOW);
    //float LDR_Green_180 = (ldrStatus_Led1 + ldrStatus_Led2 + ldrStatus_Led3 + ldrStatus_Led4 + ldrStatus_Led5)/5;
    float LDR_Yellow_90 = (ldrStatus2_Led1 + ldrStatus2_Led2 + ldrStatus2_Led3 + ldrStatus2_Led4 + ldrStatus2_Led5)/5;

    // Serial.print(F(" Sensor 180 value is ... "));
    // Serial.println(LDR_Green_180);
    //
    // Serial.print(F(" Sensor 90 value is ... "));
    // Serial.println(LDR_Yellow_90);

    // float gm = green_coeff_m;
    // float gc = green_coeff_c;
    // float ym = yellow_coeff_m;
    // float yc = yellow_coeff_c;
    //
    // if (LDR_Green_180 != -9999 && LDR_Yellow_90 != -9999) // make sure input is good
    // {
    //   Turb_180 = (gm*LDR_Green_180) + gc;
    //   Turb_90 = (ym*LDR_Yellow_90) + yc;
    // }
    //
    // if (Turb_90 < LDR_90_Stop){
    //   Turb_final = Turb_90;
    // } else if(Turb_90 > LDR_90_Stop && Turb_180 < LDR_90_Stop ){// weird situation - taking average
    //   Turb_final = (Turb_90 + Turb_180)/2 ;
    // } else {//if 90 and 180 greater than threshold, take 180
    //   Turb_final = Turb_180;
    // }

     // if (Turb_final != -9999)  // make sure input is good
     // {
     //        Serial.print(F(" Calculated result is ... "));
     //        Serial.println(Turb_final);
     // }

    //return Turb_final;


    return LDR_Yellow_90;


}

// Properties of the calculated variable
const uint8_t calculatedVarResolution90 = 2;  // The number of digits after the decimal place
const char *calculatedVarName90 = "Turbidity";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarUnit90 = "NTU";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarCode90 = "Turb";  // A short code for the variable
const char *calculatedVarUUID90 = "12345678-abcd-1234-ef00-1234567890ab";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
Variable *TurbidityVariable90 = new Variable(calculateTurbidity90, calculatedVarResolution90,
                                       calculatedVarName90, calculatedVarUnit90,
                                       calculatedVarCode90, calculatedVarUUID90);







// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

Variable *variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard),
    new ProcessorStats_FreeRam(&mcuBoard),
    new ProcessorStats_Battery(&mcuBoard),
    new MaximDS3231_Temp(&ds3231),
    new MaximDS18_Temp(&ds18),
    TurbidityVariable180,
    TurbidityVariable90
    // Additional sensor variables can be added here, by copying the syntax
    //   for creating the variable pointer (FORM1) from the `menu_a_la_carte.ino` example
    // The example code snippets in the wiki are primarily FORM2.
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray;


// ==========================================================================
//     The Logger Object[s]
// ==========================================================================

// Create a logger instance
Logger dataLogger;


// ==========================================================================
//    Working Functions
// ==========================================================================

// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75)
{
  for (uint8_t i = 0; i < numFlash; i++) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
    delay(rate);
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
    delay(rate);
  }
  digitalWrite(redLED, LOW);
}


// ==========================================================================
// Main setup function
// ==========================================================================
void setup()
{
    // Start the primary serial connection
    Serial.begin(serialBaud);
    //ads.begin();//check
    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Set information pins
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin, greenLED);

    // Begin the variable array[s], logger[s], and publisher[s]
    varArray.begin(variableCount, variableList);
    dataLogger.begin(LoggerID, loggingInterval, &varArray);

    // Set up the sensors
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    dataLogger.createLogFile(true);  // true = write a new header

    // Call the processor sleep
    dataLogger.systemSleep();
}


// ==========================================================================
// Main loop function
// ==========================================================================
void loop()
{
    dataLogger.logData();
}
