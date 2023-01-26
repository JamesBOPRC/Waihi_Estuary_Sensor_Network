// get SpC value from sensor
  float getSpC()
   {
      #define VREF 3.3      // analog reference voltage(Volt) of the ADC
      #define SCOUNT  40           // number of sample points to collect for averaging
      int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
      int analogBufferTemp[SCOUNT];
      int analogBufferIndex = 0,  copyIndex = 0;
      float averageVoltage = 0,  K = 1.0;  // K is a crude calibration factor that can be used to tune the readings
      int SpCSensorPin  = 3;
      float SpC = -1.1;

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
       SpC= ( 18.835*averageVoltage*averageVoltage*averageVoltage
            + 24.823*averageVoltage*averageVoltage
            + 624.194*averageVoltage) /compensationCoefficient * K; //convert voltage value to SpC value, then correct for temp

      Serial.print("SpC Value: ");
      Serial.println(SpC,2);
      return SpC;
   }  // end of getSpC

// get averageVolts value from sensor.  This can be sent to Ubidots for use later to calculate Specific Conductance
float getAvolts()
  {
     #define VREF 3.3      // analog reference voltage(Volt) of the ADC
     #define SCOUNT  40           // number of sample points to collect for averaging
     int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
     int analogBufferTemp[SCOUNT];
     int analogBufferIndex = 0, copyIndex = 0;
     float averageVoltage = 0,  K = 0.91;
     int SpCSensorPin  = A2;

     while (analogBufferIndex < SCOUNT)   // read the sensor every 50 milliseconds, SCOUNT times and store in array
       {
          analogBuffer[analogBufferIndex] = analogRead(SpCSensorPin);    //read the analog value and store into the buffer
          analogBufferIndex++;
 //         if(analogBufferIndex == SCOUNT)
           delay(50u);  //delay 50 milliseconds between taking sample
       }

     for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)  // for coppyIndex = 0 to SCOUNT-1
                analogBufferTemp[copyIndex]= analogBuffer[copyIndex]; // copy analogBuffer to analogBufferTemp
     averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4095.0; // read the analog value,
             // remember particle board has analog resolution of 4095
             //made more stable by the median filtering algorithm, and convert to voltage value
   return averageVoltage;
  }  // end of getAvolts

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
