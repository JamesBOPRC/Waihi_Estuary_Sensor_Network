/***********************************************************
  DFRobot Gravity: Analog Current to Voltage Converter(For 4~20mA Application)
  SKU:SEN0262

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
 ****************************************************/

#define ANALOG_PIN 2
#define RANGE 5000 // Depth measuring range 5000mm (for water)
#define CURRENT_INIT 0.004416667
 // Current @ 0mm (uint: mA)
#define DENSITY_WATER 1  // Pure water density normalized to 1
#define DENSITY_GASOLINE 0.74  // Gasoline density
#define PRINT_INTERVAL 1000
#define SCOUNT  30

int ad2, averagead2;
float dataVoltage, averageVoltage;
float dataCurrent, depth;//unit:mA
unsigned long timepoint_measure;

int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;

void setup()
{
  Serial.begin(115200);
  pinMode(ANALOG_PIN, INPUT);
  timepoint_measure = millis();
  digitalWrite(22, HIGH); //turn on sensor power
  ads.begin();
}

void loop()
{
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
  {

    analogSampleTimepoint = millis();
    ad2 = ads.readADC_SingleEnded(ANALOG_PIN);
    //dataVoltage = (ad2*3.3)/17585.0;
    //Serial.print("Raw Voltage: ");
    //Serial.println(voltage);
    analogBuffer[analogBufferIndex] = ad2;   //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT)
        analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U)
  {
     printTimepoint = millis();
     for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
       analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
       //need better median function here... somehow losing voltage values...
     averagead2 = getMedianNum(analogBufferTemp,SCOUNT);
     averageVoltage = (averagead2 * 3.3)/17585.0;// read the analog value more stable by the median filtering algorithm, and convert to voltage value
     //float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
     //float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
     //tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value

     dataCurrent = (averageVoltage / 120); //Sense Resistor:120ohm
     depth = (dataCurrent - CURRENT_INIT) * (RANGE/ DENSITY_WATER / 0.016); //Calculate depth from current readings


     //Serial print results
     Serial.print("depth:");
     Serial.print(depth);
     Serial.println("mm");
     Serial.println(averageVoltage);
   }

  }


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
