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
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 240
#endif

// ==========================================================================
//    Include the base required libraries
// ==========================================================================
#include <Arduino.h>  // The base Arduino library
#include <EnableInterrupt.h>  // for external and pin change interrupts
#include <LoggerBase.h>  // The modular sensors library

#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;    /* Use this for the Mayfly because of the onboard 16-bit ADS1115  */

#include <EEPROM.h> // use this to get the K value from the EEPROM

//EEPROM READ KVALUE
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
#define KVALUEADDR 0x0F

#include <OneWire.h>
#include <DallasTemperature.h>


/********************************************************************/
// Data wire is plugged into pin 7 on the Arduino
#define ONE_WIRE_BUS 5 // For EnviroDIY Mayfly, I am using digital pin 7.
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The name of this file
const char *sketchName = "Lite_1.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Lite_1_2021";
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
//    Wifi/Cellular Modem Settings
// ==========================================================================

// Create a reference to the serial port for the modem
HardwareSerial &modemSerial = Serial1;  // Use hardware serial if possible

// Modem Pins - Describe the physical pin connection of your modem to your board
const int8_t modemVccPin = -1;      // MCU pin controlling modem power (-1 if not applicable)
//const bool useCTSforStatus = false;  // Flag to use the XBee CTS pin for status
const int8_t modemStatusPin = 19;   // MCU pin used to read modem status (-1 if not applicable)
const int8_t modemResetPin = 20;    // MCU pin connected to modem reset pin (-1 if unconnected)
const int8_t modemSleepRqPin = 23;  // MCU pin used for modem sleep/wake request (-1 if not applicable)
const int8_t modemLEDPin = redLED;  // MCU pin connected an LED to show modem status (-1 if unconnected)

// // Network connection information

// Network connection information
const char *apn = "m2m";  // The APN for the gprs connection

// const char *wifiId = "Dare_Family";  // The WiFi access point, unnecessary for gprs
// const char *wifiPwd = "119HarveyStreet";  // The password for connecting to WiFi, unnecessary for gprs

// ==========================================================================
//    The modem object
//    Note:  Don't use more than one!
// ==========================================================================
//#elif defined MS_BUILD_TESTING && defined MS_BUILD_TEST_XBEE_LTE_B
// For the u-blox SARA R410M based Digi LTE-M XBee3
// NOTE:  According to the manual, this should be less stable than transparent
// mode, but my experience is the complete reverse.
#include <modems/DigiXBeeLTEBypass.h>
//#include <modems/SodaqDigiXBeeLTEBypass.h>

const long modemBaud = 9600;  // All XBee's use 9600 by default
const bool useCTSforStatus = false;   // Flag to use the XBee CTS pin for status
// NOTE:  If possible, use the STATUS/SLEEP_not (XBee pin 13) for status, but
// the CTS pin can also be used if necessary
DigiXBeeLTEBypass modemXBLTEB(&modemSerial,
                              modemVccPin, modemStatusPin, useCTSforStatus,
                              modemResetPin, modemSleepRqPin,
                              apn);
// Create an extra reference to the modem by a generic name (not necessary)
DigiXBeeLTEBypass modem = modemXBLTEB;
// ==========================================================================

// ==========================================================================
//    Conductvity sensor settings
// ==========================================================================
// //Cond sensor pins
const int SpCSensorPin = 3; // checked
//const float Kvalue = -9999;//updated 05/11/2020
const float Kvalue = 1;
//EEPROM_read(KVALUEADDR, Kvalue); //1.0 means no change to raw readings

 // ==========================================================================
 //    eTape sensor settings
 // ==========================================================================
 //
const int eTapeSensorPin = 1;

// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>  // Includes wrapper functions for Maxim DS3231 RTC

// Create a DS3231 sensor object, using this constructor function:
MaximDS3231 ds3231(1);


// ==========================================================================
//    Maxim DS18 One Wire Temperature Sensor
// ==========================================================================
#include <sensors/MaximDS18.h>
// OneWire Address [array of 8 hex characters]
// If only using a single sensor on the OneWire bus, you may omit the address
// DeviceAddress OneWireAddress1 = {0x28, 0xFF, 0xBD, 0xBA, 0x81, 0x16, 0x03, 0x0C};
const int8_t OneWirePower = sensorPowerPin;  // Pin to switch power on and off (-1 if unconnected)
const int8_t OneWireBus = 5;  // Pin attached to the OneWire Bus (-1 if unconnected) (D24 = A0)

// Create a Maxim DS18 sensor objects (use this form for a known address)
// MaximDS18 ds18(OneWireAddress1, OneWirePower, OneWireBus);

// Create a Maxim DS18 sensor object (use this form for a single sensor on bus with an unknown address)
MaximDS18 ds18(OneWirePower, OneWireBus);

// Create a temperature variable pointer for the DS18

//float ds18Temp = -9999;
//Variable* ds18Temp =
//        new MaximDS18_Temp(&ds18, "de727a28-dcf9-4502-97e1-40811172ccbf");

// ==========================================================================
// //  External Voltage via TI ADS1115
// // ==========================================================================
// /** Start [ext_volt] */
// #include <sensors/ExternalVoltage.h>
//
// const int8_t  ADSPower       = sensorPowerPin;  // Power pin (-1 if unconnected)
// const int8_t  ADSChannel     = 2;               // The ADS channel of interest
// const float   dividerGain    = 10;  //  Gain setting if using a voltage divider
// const uint8_t evADSi2c_addr  = 0x48;  // The I2C address of the ADS1115 ADC
// const uint8_t VoltReadsToAvg = 1;     // Only read one sample
//
//         // Create an External Voltage sensor object
// ExternalVoltage extvolt(ADSPower, ADSChannel, dividerGain, evADSi2c_addr,
//                         VoltReadsToAvg);
//
// // Create a voltage variable pointer
// Variable* extvoltV =
//     new ExternalVoltage_Volt(&extvolt, "12345678-abcd-1234-ef00-1234567890ab");
// /** End [ext_volt] */
//

// ==========================================================================
//    eTape Water Level Sensor
// ==========================================================================
float getWL(int eTapePin)
{
  #define VREF 3.3      // analog reference voltage(Volt) of the ADC 3.3 or 5???
  #define SCOUNT 50           // number of sample points to collect for averaging
  float voltage, depth;
  int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
  int analogBufferTemp[SCOUNT];
  int analogBufferIndex = 0,  copyIndex = 0;
  digitalWrite(22, HIGH);

  while (analogBufferIndex < SCOUNT)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
    {
       analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(eTapePin);    //read the analog value and store into the buffer
       analogBufferIndex++;
  //         if(analogBufferIndex == SCOUNT)
        delay(50u);  //delay 50 milliseconds between taking sample
    }
    analogBufferIndex = 0;

    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)  // for coppyIndex = 0 to SCOUNT-1
               analogBufferTemp[copyIndex]= analogBuffer[copyIndex]; // copy analogBuffer to analogBufferTemp

  voltage = (getMedianNum(analogBufferTemp,SCOUNT)*VREF)/17585.0; // read the analog value,

  //depth = (voltage - 1.23)/0.0092; //formula calibrated from eTape
  depth = (154.98*(voltage*voltage*voltage))-(801.57*(voltage*voltage))+(1449.4*voltage)-857.39;

  digitalWrite(22, LOW);
  return depth;
}

//=============================================================
// eTape as a calculated variable
//===========================================================

float calculateeTape(void)
{
float calculatedResult = -9999;  // Always safest to start with a bad value
//sensors.requestTemperatures();

//float temperature = sensors.getTempCByIndex(0);
//float temperature = MaximDS18_Temp(&ds18);
calculatedResult = getWL(eTapeSensorPin);
//alculatedResult = getSpc(Kvalue, SpCSensorPin);

  return calculatedResult;
}

// Properties of the calculated variable
const uint8_t calculatedVareTapeResolution = 2;  // The number of digits after the decimal place
const char *calculatedVareTapeName = "Gage height";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVareTapeUnit = "cm";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVareTapeCode = "WL";  // A short code for the variable
const char *calculatedVareTapeUUID = "46796135-0f44-400f-9421-2d4d1e08e32d";  // The (optional) universallly unique identifier

// // Finally, Create a calculated variable pointer and return a variable pointer to it
// Variable* eTapeVariable = new Variable(calculateeTape, calculatedVareTapeResolution,
//                                        calculatedVareTapeName, calculatedVareTapeUnit,
//                                        calculatedVareTapeCode, calculatedVareTapeUUID);

// ==========================================================================
//    Function for Specific Conductivity using the DFROBOT TDS sensor
// ==========================================================================
//Eventually I would like to add this to a library
// get SpC value from sensor
float getSpC(float Kvalue, int SpCSensorPin)
 {
//    --------------------------------------------------------
      #define VREF 3.3      // analog reference voltage(Volt) of the ADC
      #define SCOUNT 50           // number of sample points to collect for averaging
      int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
      int analogBufferTemp[SCOUNT];
      int analogBufferIndex = 0,  copyIndex = 0;
      EEPROM_read(KVALUEADDR, Kvalue);
      float averageVoltage = 0,  K = Kvalue;  // K is a crude calibration factor that can be used to tune the readings
      float SpC = -1.1;

      //float ds18TempVal = ds18Temp ->getValue();
      int ds18TempVal = sensors.getTempCByIndex(0);  //add your temperature sensor and read it


      digitalWrite(22, HIGH);

      while (analogBufferIndex < SCOUNT)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
        {
           analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(SpCSensorPin);    //read the analog value and store into the buffer
           analogBufferIndex++;
            delay(50u);  //delay 50 milliseconds between taking sample
        }
      analogBufferIndex = 0;

      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)  // for coppyIndex = 0 to SCOUNT-1
                 analogBufferTemp[copyIndex]= analogBuffer[copyIndex]; // copy analogBuffer to analogBufferTemp
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * VREF /17585.0; // read the analog value,


      float compensationCoefficient=1.0+0.019*(ds18TempVal-25.0);    //temperature compensation formula: 0.019 used by YSI

            SpC= ( 133.42*averageVoltage*averageVoltage*averageVoltage
                 - 255.86*averageVoltage*averageVoltage
                 + 857.39*averageVoltage) /compensationCoefficient * K; //convert voltage value to SpC value, then correct for temp

  digitalWrite(22, LOW);
            return SpC;
            //return ds18TempVal;

}  // end of getSpC
//=============================================================
// Conductivity/TDS as a calculated variable
//===========================================================


float calculateConductivity(void)
{
float calculatedResult = -9999;  // Always safest to start with a bad value
//sensors.requestTemperatures();

calculatedResult = getSpC(Kvalue, SpCSensorPin);
//alculatedResult = getSpc(Kvalue, SpCSensorPin);

  return calculatedResult;
}

// Properties of the calculated variable
const uint8_t calculatedVarCondResolution = 2;  // The number of digits after the decimal place
const char *calculatedVarCondName = "Conductivity";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarCondUnit = "uS/cm";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarCondCode = "Cond";  // A short code for the variable
const char *calculatedVarCondUUID = "52a03834-f15f-40aa-9e4e-4947eca0560e";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
// Variable *ConductivityVariable = new Variable(calculateConductivity, calculatedVarCondResolution,
//                                        calculatedVarCondName, calculatedVarCondUnit,
//                                        calculatedVarCondCode, calculatedVarCondUUID);


// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

Variable *variableList[] = {
    //new ProcessorStats_SampleNumber(&mcuBoard),
    //new ProcessorStats_FreeRam(&mcuBoard),
    new ProcessorStats_Battery(&mcuBoard,"ad1e7a6a-414b-4cb5-86c6-69a7c54080bd"),
    //new MaximDS3231_Temp(&ds3231),
    new MaximDS18_Temp(&ds18,"0bc19c50-67d8-4012-9e17-fb32db82f1ca"),
    new Variable(calculateeTape, calculatedVareTapeResolution,
                 calculatedVareTapeName, calculatedVareTapeUnit,
                 calculatedVareTapeCode, calculatedVareTapeUUID),
    new Variable(calculateConductivity, calculatedVarCondResolution,
                 calculatedVarCondName, calculatedVarCondUnit,
                 calculatedVarCondCode, calculatedVarCondUUID),


    // Additional sensor variables can be added here, by copying the syntax
    //   for creating the variable pointer (FORM1) from the `menu_a_la_carte.ino` example
    // The example code snippets in the wiki are primarily FORM2.
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray(variableCount, variableList);


// ==========================================================================
//     The Logger Object[s]
// ==========================================================================

// Create a logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);

// ==========================================================================
//    A Publisher to Monitor My Watershed / EnviroDIY Data Sharing Portal
// ==========================================================================
// Device registration and sampling feature information can be obtained after
// registration at https://monitormywatershed.org or https://data.envirodiy.org
const char *registrationToken = "bb4be033-adea-475c-a4ad-17ce67fc0c58";   // Device registration token
const char *samplingFeature = "03b56716-e87d-47cf-9952-10aec25d886f";     // Sampling feature UUID

// const char *UUIDs[] =                                                      // UUID array for device sensors
// {
//     "0bc19c50-67d8-4012-9e17-fb32db82f1ca",   // Temperature (Maxim_DS18B20_Temp)
//     "846f84dc-4455-47f3-bd38-51a21e20fa50"    // Temperature (Maxim_DS3231_Temp)
// };



// Create a data publisher for the EnviroDIY/WikiWatershed POST endpoint
#include <publishers/EnviroDIYPublisher.h>
EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modem.gsmClient, registrationToken, samplingFeature);


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

// Read's the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage()
{
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}


//Median Funtion
// calculate a median for set of values in buffer
int getMedianNum(int bArray[], int iFilterLen)
{     int bTab[iFilterLen];
   for (byte i = 0; i<iFilterLen; i++)
 bTab[i] = bArray[i];                  // copy input array into BTab[] array
   int i, j, bTemp;
   for (j = 0; j < iFilterLen - 1; j++)        // put array in ascending order
   {  for (i = 0; i < iFilterLen - j - 1; i++)
       {  if (bTab[i] > bTab[i + 1])
           {  bTemp = bTab[i];
              bTab[i] = bTab[i + 1];
              bTab[i + 1] = bTemp;
            }
        }
   }
if ((iFilterLen & 1) > 0)  // check to see if iFilterlen is odd or even using & (bitwise AND) i.e if length &AND 1 is TRUE (>0)
     bTemp = bTab[(iFilterLen - 1) / 2];     // then then it is odd, and should take the central value
 else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;  // if even then take aveage of two central values
return bTemp;
} //end getmedianNum


// ==========================================================================
// Main setup function
// ==========================================================================

void setup()
{
    // Wait for USB connection to be established by PC
    // NOTE:  Only use this when debugging - if not connected to a PC, this
    // could prevent the script from starting
    #if defined SERIAL_PORT_USBVIRTUAL
      while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000)){}
    #endif

    ads.begin();//check
    sensors.begin();

    //just checking the code for this
    float kvaluecheck = -9999;
    EEPROM_read(KVALUEADDR, kvaluecheck);
    Serial.print("EEPROM KVALUE:");
    Serial.println(kvaluecheck);


    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);
    Serial.print(F("TinyGSM Library version "));
    Serial.println(TINYGSM_VERSION);
    Serial.println();

    // Allow interrupts for software serial
    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt, CHANGE);
    #endif
    #if defined NeoSWSerial_h
        enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
    #endif

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

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

    // Attach the modem and information pins to the logger
    dataLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin, greenLED);

    // Begin the logger
    dataLogger.begin();

    // Note:  Please change these battery voltages to match your battery
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4)
    {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }

    // Sync the clock if it isn't valid or we have battery to spare
    if (getBatteryVoltage() > 3.55 || !dataLogger.isRTCSane())
    {
        // Synchronize the RTC with NIST
        // This will also set up the modem
        dataLogger.syncRTC();
    }

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4)
    {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);  // true = wait for card to settle after power up
        dataLogger.createLogFile(true); // true = write a new header
        dataLogger.turnOffSDcard(true); // true = wait for internal housekeeping after write
    }

    // Set up some of the power pins so the board boots up with them off
        // NOTE:  This isn't necessary at all.  The logger begin() function
        // should leave all power pins off when it finishes.
        if (modemVccPin >= 0)
        {
            pinMode(modemVccPin, OUTPUT);
            digitalWrite(modemVccPin, LOW);
            pinMode(modemSleepRqPin, OUTPUT);  // <- Added
            digitalWrite(modemSleepRqPin, HIGH);  // <- Added
        }


    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
}



// ==========================================================================
// Main loop function
// ==========================================================================
// Use this short loop for simple data logging and sending
void loop()
{
    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
    if (getBatteryVoltage() < 3.4)
    {
        dataLogger.systemSleep();
    }
    // At moderate voltage, log data but don't send it over the modem
    else if (getBatteryVoltage() < 3.55)
    {
        dataLogger.logData();
    }
    // If the battery is good, send the data to the world
    else
    {
        dataLogger.logDataAndPublish();
    }
}
