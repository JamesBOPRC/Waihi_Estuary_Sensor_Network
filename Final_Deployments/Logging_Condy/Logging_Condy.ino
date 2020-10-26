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

#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;    /* Use this for the Mayfly because of the onboard 16-bit ADS1115  */

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
const char *sketchName = "simple_logging.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Conductivity Logger Test";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
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
//    Conductvity sensor settings
// ==========================================================================
//
// //Cond sensor pins
const int SpCSensorPin = 3;
const float Kvalue = 1.700519059; //1.0 means no change to raw readings

// ==========================================================================
//    Depth sensor settings
// ==========================================================================
//
// //Cond sensor pins
const int DepthSensorPin = 2;
const float CURRENT_INIT = 0.004416667; //1.0 means no change to raw readings


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
Variable* ds18Temp =
        new MaximDS18_Temp(&ds18, "12345678-abcd-1234-ef00-1234567890ab");

// ==========================================================================
//    Function for Specific Conductivity using the DFROBOT TDS sensor
// ==========================================================================
//Eventually I would like to add this to a library
// get SpC value from sensor
float getSpC(float Kvalue, int SpCSensorPin)
 {
//    --------------------------------------------------------
      #define VREF 3.3      // analog reference voltage(Volt) of the ADC
      #define SCOUNT  50           // number of sample points to collect for averaging
      int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
      int analogBufferTemp[SCOUNT];
      int analogBufferIndex = 0,  copyIndex = 0;
      float averageVoltage = 0,  K = Kvalue;  // K is a crude calibration factor that can be used to tune the readings
      //int SpCSensorPin  = 3;
      float SpC = -1.1;
      //sensors.requestTemperatures();
      float ds18TempVal = ds18Temp ->getValue();
      //Serial.print("CONDTEMP =  ");Serial.println(ds18TempVal);

      //int temperature = sensors.getTempCByIndex(0);  //add your temperature sensor and read it

      digitalWrite(22, HIGH);

      while (analogBufferIndex < SCOUNT)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
        {
           analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(SpCSensorPin);    //read the analog value and store into the buffer
           analogBufferIndex++;
  //         if(analogBufferIndex == SCOUNT)
            delay(50u);  //delay 50 milliseconds between taking sample
        }
      analogBufferIndex = 0;

      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)  // for coppyIndex = 0 to SCOUNT-1
                 analogBufferTemp[copyIndex]= analogBuffer[copyIndex]; // copy analogBuffer to analogBufferTemp
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * 3.3 /17585.0; // read the analog value,
                                            // remember particle board has analog resolution of 4095
                                            //made more stable by the median filtering algorithm, and convert to voltage value
      //Serial.print(temperature);   // temperature comes from a different sensor, outside this function.
      //Serial.println(" deg.C at start");
      //Serial.print("Median condy voltage= "); Serial.println(averageVoltage);
      //Serial.print("Temperature= ");Serial.println(ds18TempVal);
      //Serial.print("averageVoltage= "); Serial.println(averageVoltage);
      float compensationCoefficient=1.0+0.019*(ds18TempVal-25.0);    //temperature compensation formula: 0.019 used by YSI
                //fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
// coefficients given by DFROBOT on their webpage.  Error in that the temp. compensation should be after using the equation
/* TDS=(133.42*compensationVolatge*compensationVolatge*compensationVolatge
            - 255.86*compensationVolatge*compensationVolatge
            + 857.39*compensationVolatge)*0.5*K; //convert voltage value to tds value and multiply by calibration K.
*/
// coefficients for the following equation derived from calibration to
 // hundreds of specific conductance readings taken by an Onset logger running in parallel with the Spudnik
       SpC= ( 18.835*averageVoltage*averageVoltage*averageVoltage
            + 24.823*averageVoltage*averageVoltage
            + 624.194*averageVoltage) /compensationCoefficient * K; //convert voltage value to SpC value, then correct for temp

  digitalWrite(22, LOW);
            //Serial.print("SpC Value: ");
            //Serial.println(SpC,2);
            return SpC;

}  // end of getSpC
//=============================================================
// Conductivity/TDS as a calculated variable
//===========================================================


float calculateConductivity(void)
{
float calculatedResult = -9999;  // Always safest to start with a bad value
//sensors.requestTemperatures();

//float temperature = sensors.getTempCByIndex(0);
//float temperature = MaximDS18_Temp(&ds18);
calculatedResult = getSpC(Kvalue, SpCSensorPin);
//alculatedResult = getSpc(Kvalue, SpCSensorPin);

  return calculatedResult;
}

// Properties of the calculated variable
const uint8_t calculatedVarCondResolution = 2;  // The number of digits after the decimal place
const char *calculatedVarCondName = "Conductivity";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarCondUnit = "uS/cm";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarCondCode = "Cond";  // A short code for the variable
const char *calculatedVarCondUUID = "12345678-abcd-1234-ef00-1234567890ab";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
Variable *ConductivityVariable = new Variable(calculateConductivity, calculatedVarCondResolution,
                                       calculatedVarCondName, calculatedVarCondUnit,
                                       calculatedVarCondCode, calculatedVarCondUUID);



// // ==========================================================================
// //    Function for Depth Sensor
// // ==========================================================================

float getDepth(int DepthSensorPin, float CURRENT_INIT)
 {

//Serial.print("DepthsensorPin ");Serial.println(DepthSensorPin);
   #define RANGE 5000 // Depth measuring range 5000mm (for water)
      // Current @ 0mm (uint: mA)
   #define DENSITY_WATER 1  // Pure water density normalized to 1
   #define DENSITY_GASOLINE 0.74  // Gasoline density
   #define PRINT_INTERVAL 1000
   #define SCOUNT_DEPTH 50

  //int ads2;// averageadDepth;
  float averageVoltageDepth;
  float dataCurrent, depth;//unit:mA
   //unsigned long timepoint_measure;

   int analogBufferDepth[SCOUNT_DEPTH];    // store the analog value in the array, read from ADC
   int analogBufferTempDepth[SCOUNT_DEPTH];
   int analogBufferIndexDepth = 0,copyIndexDepth = 0;

   digitalWrite(22, HIGH);
   while (analogBufferIndexDepth < SCOUNT_DEPTH)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
     {
        analogBufferDepth[analogBufferIndexDepth] = ads.readADC_SingleEnded(DepthSensorPin);    //read the analog value and store into the buffer
        analogBufferIndexDepth++;
//         if(analogBufferIndex == SCOUNT)
         delay(50u);  //delay 50 milliseconds between taking sample
     }
   analogBufferIndexDepth = 0;

   for(copyIndexDepth=0;copyIndexDepth<SCOUNT_DEPTH;copyIndexDepth++)  // for coppyIndex = 0 to SCOUNT-1
              analogBufferTempDepth[copyIndexDepth]= analogBufferDepth[copyIndexDepth]; // copy analogBuffer to analogBufferTemp

   averageVoltageDepth = getMedianNum(analogBufferTempDepth,SCOUNT_DEPTH) * 3.3 /17585.0; // read the analog value,

   //Serial.print("Median Voltage Depth: ");
   //Serial.print(averageVoltageDepth);

   dataCurrent = (averageVoltageDepth / 120); //Sense Resistor:120ohm
   depth = (dataCurrent - CURRENT_INIT) * (RANGE/ DENSITY_WATER / 0.016); //Calculate depth from current readings

  digitalWrite(22, LOW);

  //Serial.print("Depth Value: ");
  //Serial.println(depth);

  return depth;

}






//
float calculateDepth(void)
{

float calculatedResultDepth = -6666;  // Always safest to start with a bad value
//sensors.requestTemperatures();
//float temperature = sensors.getTempCByIndex(0);
calculatedResultDepth = getDepth(DepthSensorPin, CURRENT_INIT);
    return calculatedResultDepth;
}

// Properties of the calculated variable
const uint8_t calculatedVarDepthResolution = 1;  // The number of digits after the decimal place
const char *calculatedVarDepthName = "waterLevel";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarDepthUnit = "mm";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarDepthCode = "Depth";  // A short code for the variable
const char *calculatedVarDepthUUID = "12345678-abcd-1234-ef00-1234567890ab";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
Variable *DepthVariable = new Variable(calculateDepth, calculatedVarDepthResolution,
                                       calculatedVarDepthName, calculatedVarDepthUnit,
                                       calculatedVarDepthCode, calculatedVarDepthUUID);


// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

Variable *variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard),
    new ProcessorStats_FreeRam(&mcuBoard),
    new ProcessorStats_Battery(&mcuBoard),
    new MaximDS3231_Temp(&ds3231),
    //new MaximDS18_Temp(&ds18),
    ds18Temp,
    ConductivityVariable,
    DepthVariable



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
    // Start the primary serial connection
    Serial.begin(serialBaud);
    ads.begin();//check
    //sensors.begin();

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
