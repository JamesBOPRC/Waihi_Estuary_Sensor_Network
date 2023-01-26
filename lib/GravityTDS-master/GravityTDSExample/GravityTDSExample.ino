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
#include "GravityTDS.h"
// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1015.h>

#define TdsSensorPin 3
GravityTDS gravityTds;

// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
/********************************************************************/
#define SENS_ON 22
Adafruit_ADS1115 ads;


float temperature = 25,tdsValue = 0;

void setup()
{
    Serial.begin(115200);
    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(3.3);  //reference voltage on ADC, default 5.0V on Arduino UNO
    gravityTds.setAdcRange(17585.0);  //1024 for 10bit ADC;4096 for 12bit ADC
    gravityTds.begin();  //initialization

    digitalWrite(SENS_ON, HIGH);
    //ec.begin();
    sensors.begin();
    ads.begin();

    EEPROM_read(KVALUEADDR, kvaluecheck);
    Serial.print("EEPROM KVALUE:");

    Serial.println(kvaluecheck);


}

void loop()
{
    temperature = readTemperature();  //add your temperature sensor and read it
    //gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    //gravityTds.update();  //sample and calculate
    //tdsValue = gravityTds.getTdsValue();  // then get the value
    voltage =


    this->ecValue=(133.42*this->voltage*this->voltage*this->voltage - 255.86*this->voltage*this->voltage + 857.39*this->voltage)*this->kValue;
        this->ecValue25  =  this->ecValue / (1.0+0.02*(this->temperature-25.0));  //temperature compensation
        this->tdsValue = ecValue25 ;
        return tdsValue;



    Serial.print(tdsValue,0);
    Serial.println("ppm");
    delay(1000);
}


float readTemperature()
{
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);//add your code here to get the temperature from your temperature sensor
  return temperature;
}
