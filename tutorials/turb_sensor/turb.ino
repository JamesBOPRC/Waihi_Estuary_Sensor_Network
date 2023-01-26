
 #include <Arduino.h>  // The base Arduino library
 #include <EnableInterrupt.h>  // for external and pin change interrupts
 #include <LoggerBase.h>  // The modular sensors library
 #include <Adafruit_ADS1015.h>


#define TurbSensorPin 3
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
#define SENS_ON 22
Adafruit_ADS1115 ads;

float analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
float analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
float voltage = 0, ad3 = 9999;

void setup()
{
  Serial.begin(115200);
  pinMode(TurbSensorPin,INPUT);
  digitalWrite(SENS_ON, HIGH);
  ads.begin();
}

void loop()

{
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {

     analogSampleTimepoint = millis();
     ad3 = ads.readADC_SingleEnded(TurbSensorPin);
     voltage = (ad3*3.3)/17585.0;
     //Serial.print("Raw Voltage: ");
     //Serial.println(voltage);
     analogBuffer[analogBufferIndex] = voltage;   //read the analog value and store into the buffer
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
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT);  // read the analog value more stable by the median filtering algorithm, and convert to voltage value

      Serial.print("voltage:");
      Serial.println(averageVoltage,5);


   }
}
float getMedianNum(float bArray[], int iFilterLen)
{
      float bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j;
      float bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
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
      //Serial.println(bTemp);
}
