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
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}


// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1015.h>

#define EC_PIN 1
float voltage,ecValue,temperature = 25,ad3 = 9999;

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 5

#define KVALUEADDR 0x0F    //the start address of the K value stored in the EEPROM
#define RES2 (7500.0/0.66)
#define ECREF 20.0
#define KVAL 1.0

// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
/********************************************************************/

#define SENS_ON 22
Adafruit_ADS1115 ads;

#define ECSOLUTION 12.88

#define SCOUNT  40           // number of sample points to collect for averaging
void setup()
{
  Serial.begin(115200);

  digitalWrite(SENS_ON, HIGH);
  //ec.begin();
  sensors.begin();
  ads.begin();

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

      //ad3 = ads.readADC_SingleEnded(EC_PIN);
      //voltage2 = (ad3*3300)/17585.0;//make sure voltage is in milivolts

      //Serial.println(voltage2);

      //getMedianNum(analogBufferTemp,SCOUNT)

      voltage = (getMedianNum(analogBufferTemp,SCOUNT) *3300)/17585.0; // read the analog value,

      temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
      rawECsolution = ECSOLUTION*(1.0+0.0185*(temperature-25.0));  //temperature compensation

      KValueTemp = RES2*ECREF*rawECsolution/1000.0/voltage/10.0;  //calibrate the k value

      //voltage = analogRead(EC_PIN)/1024.0*5000;  // read the voltage

      Serial.println("Reading Sensors:");
      Serial.println();
      Serial.print("voltage:");
      Serial.print(voltage);

      //ecValue = ecvalueRaw / (1.0+0.0185*(temperature-25.0));  //temperature compensation
      //ecValue =  readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
      Serial.print("  temperature:");
      Serial.print(temperature,1);


      Serial.print("^C  rawECsolution:");
      Serial.println(rawECsolution,3);

      if((KValueTemp>0.5) && (KValueTemp<1.8))
      {
          Serial.println();
          Serial.print(F(">>>Successful,K:"));
          Serial.println(KValueTemp);
          EEPROM_write(KVALUEADDR, KValueTemp);
          Serial.print(F(">>>Calibration Successful"));

      }
      else{
        Serial.println();
        Serial.println(F(">>>Failed,Try Again<<<"));
        Serial.println();

      }

      Serial.print(" KValueTemp:");
      Serial.println(KValueTemp,3);





}

void loop()
{
   //do nothing
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
