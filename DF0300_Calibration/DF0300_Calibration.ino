/*
 * file DFRobot_EC.ino
 * @ https://github.com/DFRobot/DFRobot_EC
 *
 * This is the sample code for Gravity: Analog Electrical Conductivity Sensor / Meter Kit V2 (K=1.0), SKU: DFR0300.
 * In order to guarantee precision, a temperature sensor such as DS18B20 is needed, to execute automatic temperature compensation.
 * You can send commands in the serial monitor to execute the calibration.
 * Serial Commands:
 *   enterec -> enter the calibration mode
 *   calec -> calibrate with the standard buffer solution, two buffer solutions(1413us/cm and 12.88ms/cm) will be automaticlly recognized
 *   exitec -> save the calibrated parameters and exit from calibration mode
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  2018-03-21
 */

#include <DFRobot_EC.h>
#include <EEPROM.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor 

#define DHTTYPE DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t dhtDelayMS;

#define EC_PIN A1
float voltage,ecValue,temperature = 23;
DFRobot_EC ec;

void setup()
{
  Serial.begin(9600);  
  ec.begin();

  // DHT22 SETUP
  dht.begin();
  sensor_t sensor;

  dht.temperature().getSensor(&sensor);
  // DHT22 SETUP

  // Set delay between sensor readings based on sensor details.
  dhtDelayMS = sensor.min_delay / 1000; 
}

void loop()
{
    static unsigned long timepoint = millis();
    if(millis()-timepoint>dhtDelayMS)  //time interval: 1s
    {
      timepoint = millis();
      voltage = analogRead(EC_PIN)/1024.0*5000;   // read the voltage
      temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
      ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
      Serial.print("temperature:");
      Serial.print(temperature,1);
      Serial.print("^C  EC:");
      Serial.print(ecValue,2);
      Serial.println("ms/cm");
    }
    ec.calibration(voltage,temperature);          // calibration process by Serail CMD
}

float readTemperature()
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    return 0;
  }
  else {
    // Serial.print(F("Temperature: "));
    // Serial.print(event.temperature);
    // Serial.println(F("°C"));
    return event.temperature;
  }
}