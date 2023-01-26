// #include <Arduino.h>  // The base Arduino library
//
// #define RESISTOR 1000
// #define SENSORPIN A1
//
//
//
//
//
// void setup(void) {
// Serial.begin(115200);
// }
//
//
// void loop(void) {
//
//   float reading;
//   reading = analogRead(SENSORPIN);
//
//   Serial.print("Analog reading ");
//   Serial.println(reading);
//
//   reading = (1023 / reading)  - 1;     // (1023/ADC - 1)
//   reading = RESISTOR/ reading;  // 10K / (1023/ADC - 1)
//   Serial.print("Sensor resistance ");
//   Serial.println(reading);
//
//   delay(1000);
// }




#include <EnableInterrupt.h>  // for external and pin change interrupts
#include <LoggerBase.h>  // The modular sensors library
#include <Adafruit_ADS1015.h>

#define eTapePin 2
//#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SENS_ON 22
Adafruit_ADS1115 ads;
#define SCOUNT  40           // number of sample points to collect for averaging

// the value of the 'other' resistor
//#define SERIESRESISTOR 1000
 // What pin to connect the sensor to
//#define SENSORPIN A0

void setup(void) {
  Serial.begin(115200);
  ads.begin();
  pinMode(eTapePin,INPUT);
  digitalWrite(SENS_ON, HIGH);
}

void loop(void) {
  float voltage; float depth;
  int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
  int analogBufferTemp[SCOUNT];
  int analogBufferIndex = 0,  copyIndex = 0;

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


  voltage = (getMedianNum(analogBufferTemp,SCOUNT) *3.3)/17585.0; // read the analog value,
  depth = (voltage - 1.23)/0.0092;

//

  // // convert the value to resistance
    //reading = (65536/reading)  - 1;
    //reading = SERIESRESISTOR / reading;
  Serial.print("Sensor voltage: ");
   Serial.print(voltage,5);
   Serial.print("   Water level:");
   Serial.println(depth,3);


  delay(1000);
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



















// Liquid Level Sensor Sketch

// Show the raw resistance values measured from an eTape liquid level sensor.
// See details on the sensor at:
//   https://www.adafruit.com/products/1786

// Created by Tony DiCola
// Released under an MIT license: http://opensource.org/licenses/MIT

// // Configuration values:
// #define SERIES_RESISTOR     1000    // Value of the series resistor in ohms.
// #define SENSOR_PIN          1      // Analog pin which is connected to the sensor.
//
// // The following are calibration values you can fill in to compute the volume of measured liquid.
// // To find these values first start with no liquid present and record the resistance as the
// // ZERO_VOLUME_RESISTANCE value.  Next fill the container with a known volume of liquid and record
// // the sensor resistance (in ohms) as the CALIBRATION_RESISTANCE value, and the volume (which you've
// // measured ahead of time) as CALIBRATION_VOLUME.
// #define ZERO_VOLUME_RESISTANCE    0.00    // Resistance value (in ohms) when no liquid is present.
// #define CALIBRATION_RESISTANCE    0.00    // Resistance value (in ohms) when liquid is at max line.
// #define CALIBRATION_VOLUME        0.00    // Volume (in any units) when liquid is at max line.
//
// void setup(void) {
// Serial.begin(9600);
// }
//
// void loop(void) {
// // Measure sensor resistance.
// float resistance = readResistance(SENSOR_PIN, SERIES_RESISTOR);
// Serial.print("Resistance: ");
// Serial.print(resistance, 2);
// Serial.println(" ohms");
// // Map resistance to volume.
// float volume = resistanceToVolume(resistance, ZERO_VOLUME_RESISTANCE, CALIBRATION_RESISTANCE, CALIBRATION_VOLUME);
// Serial.print("Calculated volume: ");
// Serial.println(volume, 5);
// // Delay for a second.
// delay(1000);
// }
//
// float readResistance(int pin, int seriesResistance) {
// // Get ADC value.
// float resistance = analogRead(pin);
// // Convert ADC reading to resistance.
// resistance = (1023.0 / resistance) - 1.0;
// resistance = seriesResistance / resistance;
// return resistance;
// }
//
// float resistanceToVolume(float resistance, float zeroResistance, float calResistance, float calVolume) {
// if (resistance > zeroResistance || (zeroResistance - calResistance) == 0.0) {
//   // Stop if the value is above the zero threshold, or no max resistance is set (would be divide by zero).
//   return 0.0;
// }
// // Compute scale factor by mapping resistance to 0...1.0+ range relative to maxResistance value.
// float scale = (zeroResistance - resistance) / (zeroResistance - calResistance);
// // Scale maxVolume based on computed scale factor.
// return calVolume * scale;
// }
