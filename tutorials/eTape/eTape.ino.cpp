# 1 "C:\\Users\\User\\AppData\\Local\\Temp\\tmpsxu79cdf"
#include <Arduino.h>
# 1 "C:/Users/User/Documents/Arduino/Mayfly/deployments/tutorials/eTape/eTape.ino"
# 34 "C:/Users/User/Documents/Arduino/Mayfly/deployments/tutorials/eTape/eTape.ino"
#include <EnableInterrupt.h>
#include <LoggerBase.h>
#include <Adafruit_ADS1015.h>

#define eTapePin 2

#define SENS_ON 22
Adafruit_ADS1115 ads;
#define SCOUNT 40
void setup(void);
void loop(void);
float getMedianNum(int bArray[], int iFilterLen);
#line 49 "C:/Users/User/Documents/Arduino/Mayfly/deployments/tutorials/eTape/eTape.ino"
void setup(void) {
  Serial.begin(115200);
  ads.begin();
  pinMode(eTapePin,INPUT);
  digitalWrite(SENS_ON, HIGH);
}

void loop(void) {
  float voltage; float depth;
  int analogBuffer[SCOUNT];
  int analogBufferTemp[SCOUNT];
  int analogBufferIndex = 0, copyIndex = 0;

  while (analogBufferIndex < SCOUNT)
    {
       analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(eTapePin);
       analogBufferIndex++;

        delay(50u);
    }
    analogBufferIndex = 0;

    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
               analogBufferTemp[copyIndex]= analogBuffer[copyIndex];


  voltage = (getMedianNum(analogBufferTemp,SCOUNT) *3.3)/17585.0;
  depth = (voltage - 1.23)/0.0092;






  Serial.print("Sensor voltage: ");
   Serial.print(voltage,5);
   Serial.print("   Water level:");
   Serial.println(depth,3);


  delay(1000);
}






float getMedianNum(int bArray[], int iFilterLen)
{ int bTab[iFilterLen];
   for (byte i = 0; i<iFilterLen; i++)
 bTab[i] = bArray[i];
   int i, j, bTemp;
   for (j = 0; j < iFilterLen - 1; j++)
   { for (i = 0; i < iFilterLen - j - 1; i++)
       { if (bTab[i] > bTab[i + 1])
           { bTemp = bTab[i];
              bTab[i] = bTab[i + 1];
              bTab[i + 1] = bTemp;
            }
        }
   }
if ((iFilterLen & 1) > 0)
     bTemp = bTab[(iFilterLen - 1) / 2];
 else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
return bTemp;
}