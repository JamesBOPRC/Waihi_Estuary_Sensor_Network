/*
 * file DFRobot_EC10.ino
 * @ https://github.com/DFRobot/DFRobot_EC10
 *
 * This is the sample code for Gravity: Analog Electrical Conductivity Sensor / Meter Kit(K=10), SKU: DFR0300_H.
 * In order to guarantee precision, a temperature sensor such as DS18B20 is needed, to execute automatic temperature compensation.
 * You can send commands in the serial monitor to execute the calibration.
 * Serial Commands:
 *   enterec -> enter the calibration mode
 *   calec -> calibrate with the standard buffer solution, one buffer solutions(12.88ms/cm) will be automaticlly recognized
 *   exitec -> save the calibrated parameters and exit from calibration mode
 *
 * Copyright   [DFRobot](https://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  2018-11
 */

//#include "DFRobot_EC10.h"
#include <EEPROM.h>
#include <DFRobot_EC10.h>

// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1015.h>

#define EC_PIN 1
float voltage,ecValue,temperature = 25,ad3 = 9999;
DFRobot_EC10 ec;

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 5

#define RES2 (7500.0/0.66)
#define ECREF 20.0
#define KVAL 1.0

#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
#define KVALUEADDR 0x0F

// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
/********************************************************************/

#define SENS_ON 22
Adafruit_ADS1115 ads;
#define SCOUNT  40           // number of sample points to collect for averaging

void setup()
{
  float kvaluecheck = -9999;

  Serial.begin(115200);
  //Serial.begin(9600);
  digitalWrite(SENS_ON, HIGH);
  ec.begin();
  sensors.begin();
  ads.begin();

  EEPROM_read(KVALUEADDR, kvaluecheck);
  Serial.print("EEPROM KVALUE:");

  Serial.println(kvaluecheck);

}

void loop()
{
  float value = 0, ecvalueRaw = -9999;
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();

      float value = 0, ecvalueRaw = -9999, rawECsolution = -9999;
      float KValueTemp = -9999, temperature = -9999, voltage2 = -9999;
      int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
      int analogBufferTemp[SCOUNT];
      int analogBufferIndex = 0,  copyIndex = 0;

      while (analogBufferIndex < SCOUNT)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
        {
           analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(EC_PIN);    //read the analog value and store into the buffer
           analogBufferIndex++;
      //         if(analogBufferIndex == SCOUNT)
            delay(50u);  //delay 50 milliseconds between taking sample
        }
        analogBufferIndex = 0;

        for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)  // for coppyIndex = 0 to SCOUNT-1
                   analogBufferTemp[copyIndex]= analogBuffer[copyIndex]; // copy analogBuffer to analogBufferTemp

          voltage = (getMedianNum(analogBufferTemp,SCOUNT) *3300)/17585.0; // read the analog value,


//      ad3 = ads.readADC_SingleEnded(EC_PIN);
//      voltage = (ad3*3300)/17585.0;//make sure voltage is in milivolts

      //voltage = analogRead(EC_PIN)/1024.0*5000;  // read the voltage
      Serial.print("voltage:");
      Serial.print(voltage);
      temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
      //ecvalueRaw = 1000*voltage/RES2/ECREF*KVAL*10.0;
      ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation

      //ecValue = ecvalueRaw / (1.0+0.0185*(temperature-25.0));  //temperature compensation
      //ecValue =  readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
      Serial.print("  temperature:");
      Serial.print(temperature,1);
      Serial.print("^C  EC:");
      Serial.print(ecValue,3);
      Serial.println("ms/cm");
    }
    ec.calibration(voltage,temperature);  // calibration process by Serail CMD
}

float readTemperature()
{
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);//add your code here to get the temperature from your temperature sensor
  return temperature;
}

// calculate a median for set of values in buffer
float getMedianNum(int bArray[], int iFilterLen)
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
