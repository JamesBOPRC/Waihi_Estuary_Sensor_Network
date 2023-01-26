/***************************************************
 DFRobot Gravity: Analog TDS Sensor/Meter
 <https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244>

 ***************************************************
 This sample code shows how to read the tds value and calibrate it with the standard buffer solution.
 707ppm(1413us/cm)@25^c standard buffer solution is recommended.

 Created 2018-1-3
 By Jason <jason.ling@dfrobot.com@dfrobot.com>

 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution.
 ****************************************************/

 /***********Notice and Trouble shooting***************
 1. This code is tested on Arduino Uno with Arduino IDE 1.0.5 r2 and 1.8.2.
 2. Calibration CMD:
     enter -> enter the calibration mode
     cal:tds value -> calibrate with the known tds value(25^c). e.g.cal:707
     exit -> save the parameters and exit the calibration mode
 ****************************************************/

#include <EEPROM.h>
//#include "GravityTDS.h"

//#define TdsSensorPin 3
//GravityTDS gravityTds;

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
/********************************************************************/
 #include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;

float temperature = 25,ecValue = 0;

//EEPROM READ KVALUE
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
#define KVALUEADDR 0x0F


void setup()
{
  float kvaluecheck = -9999;

    Serial.begin(115200);
    //gravityTds.setPin(TdsSensorPin);
    digitalWrite(22, HIGH); //turn on sensor power
    //gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
    //gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
    //gravityTds.begin();  //initialization
    sensors.begin();
    ads.begin();

    EEPROM_read(KVALUEADDR, kvaluecheck);
    Serial.print("EEPROM KVALUE:");

    Serial.println(kvaluecheck);


}

void loop()
{

    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);  //add your temperature sensor and read it

    //gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    //gravityTds.update();  //sample and calculate
    ecValue = getSpC(temperature);  // then get the value
    Serial.print(ecValue,0);
    Serial.println("uS/cm");
    Serial.print(temperature);
    Serial.println("degC");
    delay(1000);
}

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

// get SpC value from sensor
float getSpC(float temperature)
 {
      #define VREF 3.3      // analog reference voltage(Volt) of the ADC
      #define SCOUNT  40           // number of sample points to collect for averaging
      int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
      int analogBufferTemp[SCOUNT];
      int analogBufferIndex = 0,  copyIndex = 0;
      float averageVoltage = 0,  K = -9999;  // K is a crude calibration factor that can be used to tune the readings
      int SpCSensorPin  = 3;
      float SpC = -1.1;

      EEPROM_read(KVALUEADDR, K);

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
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF /17585.0; // read the analog value,
                                            // remember particle board has analog resolution of 4095
                                            //made more stable by the median filtering algorithm, and convert to voltage value
      Serial.print(temperature);   // temperature comes from a different sensor, outside this function.
      Serial.println(" deg.C at start");
      Serial.print("median analog reading= "); Serial.println(getMedianNum(analogBufferTemp,SCOUNT));
      Serial.print("averageVoltage= "); Serial.println(averageVoltage);
      float compensationCoefficient=1.0+0.019*(temperature-25.0);    //temperature compensation formula: 0.019 used by YSI
                //fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
// coefficients given by DFROBOT on their webpage.  Error in that the temp. compensation should be after using the equation
/* TDS=(133.42*compensationVolatge*compensationVolatge*compensationVolatge
            - 255.86*compensationVolatge*compensationVolatge
            + 857.39*compensationVolatge)*0.5*K; //convert voltage value to tds value and multiply by calibration K.
*/
// coefficients for the following equation derived from calibration to
 // hundreds of specific conductance readings taken by an Onset logger running in parallel with the Spudnik
       // SpC= ( 18.835*averageVoltage*averageVoltage*averageVoltage
       //      + 24.823*averageVoltage*averageVoltage
       //      + 624.194*averageVoltage) /compensationCoefficient * K; //convert voltage value to SpC value, then correct for temp

            SpC= ( 133.42*averageVoltage*averageVoltage*averageVoltage
                 - 255.86*averageVoltage*averageVoltage
                 + 857.39*averageVoltage) /compensationCoefficient * K; //convert voltage value to SpC value, then correct for temp


          //  KValueTemp = rawECsolution/(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage);  //calibrate in the  buffer solution, such as 707ppm(1413us/cm)@25^c



            Serial.print("SpC Value: ");
            Serial.println(SpC,2);
            return SpC;

}  // end of getSpC
