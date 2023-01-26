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

// Include the main header for ModularSensors
#include <ModularSensors.h>
/** End [includes] */

//#include <Adafruit_ADS1015.h>
//Adafruit_ADS1115 ads;    /* Use this for the Mayfly because of the onboard 16-bit ADS1115  */


//#include <AltSoftSerial.h>
//#include <YosemitechModbus.h>

// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The name of this file
const char *sketchName = "03_Kokopu.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Kokopu";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 15;
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
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>  // Includes wrapper functions for Maxim DS3231 RTC

// Create a DS3231 sensor object, using this constructor function:
MaximDS3231 ds3231(1);

// ==========================================================================
//  Meter Hydros 21 Conductivity, Temperature, and Depth Sensor
// ==========================================================================
/** Start [hydros21] */
#include <sensors/DecagonCTD.h>

const char*   CTDSDI12address   = "1";    // The SDI-12 Address of the CTD
const uint8_t CTDNumberReadings = 6;      // The number of readings to average
const int8_t  CTDPower = sensorPowerPin;  // Power pin (-1 if unconnected)
const int8_t  CTDData  = 11;               // The SDI12 data pin

// Create a Decagon CTD sensor object
DecagonCTD ctd(*CTDSDI12address, CTDPower, CTDData, CTDNumberReadings);

// Create conductivity, temperature, and depth variable pointers for the CTD
//Variable* ctdCond = new DecagonCTD_Cond(&ctd,
//                                        "12345678-abcd-1234-ef00-1234567890ab");
//Variable* ctdTemp = new DecagonCTD_Temp(&ctd,
//                                        "12345678-abcd-1234-ef00-1234567890ab");
//Variable* ctdDepth =
//    new DecagonCTD_Depth(&ctd, "12345678-abcd-1234-ef00-1234567890ab");
/** End [hydros21] */

// ==========================================================================
//  Yosemitech Y511 Turbidity Sensor with Wiper
// ==========================================================================
/** Start [y511] */
#include <sensors/YosemitechY511.h>
#include <AltSoftSerial.h>
AltSoftSerial altSoftSerial;
// Create a reference to the serial port for modbus
// Extra hardware and software serial ports are created in the "Settings for
// Additional Serial Ports" section
#if defined ARDUINO_ARCH_SAMD || defined ATMEGA2560
HardwareSerial& y511modbusSerial = Serial2;  // Use hardware serial if possible
#else
AltSoftSerial& y511modbusSerial = altSoftSerial;  // For software serial
// NeoSWSerial& y511modbusSerial = neoSSerial1;  // For software serial
#endif

byte         y511ModbusAddress = 0x01;  // The modbus address of the Y511
const int8_t y511AdapterPower =
    sensorPowerPin;  // RS485 adapter power pin (-1 if unconnected)
const int8_t  y511SensorPower = -1;  // Sensor power pin
const int8_t  y511EnablePin   = -1;  // Adapter RE/DE pin (-1 if not applicable)
const uint8_t y511NumberReadings = 5;
// The manufacturer recommends averaging 10 readings, but we take 5 to minimize
// power consumption

// Create a Y511-A Turbidity sensor object
YosemitechY511 y511(y511ModbusAddress, y511modbusSerial, y511AdapterPower,
                    y511SensorPower, y511EnablePin, y511NumberReadings);

// Create turbidity and temperature variable pointers for the Y511
//Variable* y511Turb =
//    new YosemitechY511_Turbidity(&y511, "12345678-abcd-1234-ef00-1234567890ab");
//Variable* y511Temp =
//    new YosemitechY511_Temp(&y511, "12345678-abcd-1234-ef00-1234567890ab");
/** End [y511] */



// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

Variable *variableList[] = {
    //new ProcessorStats_SampleNumber(&mcuBoard),
    //new ProcessorStats_FreeRam(&mcuBoard),
    new ProcessorStats_Battery(&mcuBoard,"f6126868-dd30-4ad4-84b3-10c334336e9a"),
    new MaximDS3231_Temp(&ds3231,"780cd9c8-de8b-4aa3-af4a-08fb421631ca"),
    new DecagonCTD_Cond(&ctd, "8dcb28ec-3320-49ad-81c9-177e5a3b144a"),
    new DecagonCTD_Temp(&ctd, "9846ef2a-39ba-4cd1-a108-8b44dfe88429"),
        new DecagonCTD_Depth(&ctd, "4356b101-de61-47fb-bbfd-34241ab074b3"),
        new YosemitechY511_Turbidity(&y511, "7c374dfb-18ae-4db7-8aeb-245023c928db"),
        new YosemitechY511_Temp(&y511, "e685a494-abc6-4419-a8ec-8a577d8da673")


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

const char *registrationToken = "ee3172df-fbb7-45ab-8a7c-3e82de9d94d4";   // Device registration token
const char *samplingFeature = "02e38178-901c-4ade-8744-1778f01a9c70";     // Sampling feature UUID

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

    // Start the stream for the modbus sensors; all currently supported modbus sensors use 9600 baud
    y511modbusSerial.begin(9600);

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

/** Start [complex_loop] */
// Use this long loop when you want to do something special
// Because of the way alarms work on the RTC, it will wake the processor and
// start the loop every minute exactly on the minute.
// The processor may also be woken up by another interrupt or level change on a
// pin - from a button or some other input.
// The "if" statements in the loop determine what will happen - whether the
// sensors update, testing mode starts, or it goes back to sleep.
void loop() {
    // Reset the watchdog
    dataLogger.watchDogTimer.resetWatchDog();

    // Assuming we were woken up by the clock, check if the current time is an
    // even interval of the logging interval
    // We're only doing anything at all if the battery is above 3.4V
    if (dataLogger.checkInterval() && getBatteryVoltage() > 3.4) {

        // Flag to notify that we're in already awake and logging a point
        Logger::isLoggingNow = true;
        dataLogger.watchDogTimer.resetWatchDog();

        // Print a line to show new reading
        Serial.println(F("------------------------------------------"));
        // Turn on the LED to show we're taking a reading
        dataLogger.alertOn();
        // Power up the SD Card, but skip any waits after power up
        dataLogger.turnOnSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();

        // Turn on the modem to let it start searching for the network
        // Only turn the modem on if the battery at the last interval was high
        // enough
        // NOTE:  if the modemPowerUp function is not run before the
        // completeUpdate
        // function is run, the modem will not be powered and will not
        // return a signal strength reading.
        if (getBatteryVoltage() > 3.6) modem.modemPowerUp();

        // Start the stream for the modbus sensors, if your RS485 adapter bleeds
        // current from data pins when powered off & you stop modbus serial
        // connection with digitalWrite(5, LOW), below.
        // https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
        altSoftSerial.begin(9600);

        // Do a complete update on the variable array.
        // This this includes powering all of the sensors, getting updated
        // values, and turing them back off.
        // NOTE:  The wake function for each sensor should force sensor setup
        // to run if the sensor was not previously set up.
        varArray.completeUpdate();

        dataLogger.watchDogTimer.resetWatchDog();

        // Reset modbus serial pins to LOW, if your RS485 adapter bleeds power
        // on sleep, because Modbus Stop bit leaves these pins HIGH.
        // https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
        digitalWrite(5, LOW);  // Reset AltSoftSerial Tx pin to LOW
        digitalWrite(6, LOW);  // Reset AltSoftSerial Rx pin to LOW

        // Create a csv data record and save it to the log file
        dataLogger.logToSD();
        dataLogger.watchDogTimer.resetWatchDog();

        // Connect to the network
        // Again, we're only doing this if the battery is doing well
        if (getBatteryVoltage() > 3.55) {
          //Serial.println(F("Waking up modem-------"));
        //Serial.println(F("Waking up") + modem.getModemName() + F("..."));


            modem.modemWake();
            dataLogger.watchDogTimer.resetWatchDog();
            if (modem.connectInternet()) {
                dataLogger.watchDogTimer.resetWatchDog();
                // Publish data to remotes
                Serial.println(F("Modem connected to internet."));
                dataLogger.publishDataToRemotes();

                // Sync the clock at midnight
                dataLogger.watchDogTimer.resetWatchDog();
                if (Logger::markedLocalEpochTime != 0 &&
                    Logger::markedLocalEpochTime % 86400 == 0) {
                    Serial.println(F("Running a daily clock sync..."));
                    dataLogger.setRTClock(modem.getNISTTime());
                    dataLogger.watchDogTimer.resetWatchDog();
                    modem.updateModemMetadata();
                    dataLogger.watchDogTimer.resetWatchDog();
                }

                // Disconnect from the network
                modem.disconnectInternet();
                dataLogger.watchDogTimer.resetWatchDog();
            }
            // Turn the modem off
            modem.modemSleepPowerDown();
            dataLogger.watchDogTimer.resetWatchDog();
        }

        // Cut power from the SD card - without additional housekeeping wait
        dataLogger.turnOffSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();
        // Turn off the LED
        dataLogger.alertOff();
        // Print a line to show reading ended
        Serial.println(F("------------------------------------------\n"));

        // Unset flag
        Logger::isLoggingNow = false;
    }

    // Check if it was instead the testing interrupt that woke us up
    if (Logger::startTesting) {
        // Start the stream for the modbus sensors, if your RS485 adapter bleeds
        // current from data pins when powered off & you stop modbus serial
        // connection with digitalWrite(5, LOW), below.
        // https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
        altSoftSerial.begin(9600);

        dataLogger.testingMode();
    }

    // Reset modbus serial pins to LOW, if your RS485 adapter bleeds power
    // on sleep, because Modbus Stop bit leaves these pins HIGH.
    // https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
    digitalWrite(5, LOW);  // Reset AltSoftSerial Tx pin to LOW
    digitalWrite(6, LOW);  // Reset AltSoftSerial Rx pin to LOW

    // Call the processor sleep
    dataLogger.systemSleep();
}
