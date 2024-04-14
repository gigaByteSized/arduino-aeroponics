#include <Arduino.h>
#include <DFRobot_EC.h>
#include <EEPROM.h>

#include <DHT.h>
#include <DHT_U.h>
#include <HCSR04.h>

// Pins for pH, EC and water level sensors
#define PH_SENS_PIN A0
#define EC_SENS_PIN A1
#define WATER_LEVEL_SENS_PIN A3

#define ULTRASONIC_TRIG_PIN 2
#define ULTRASONIC_ECHO_PIN 3
UltraSonicDistanceSensor distanceSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);


#define WATER_LEVEL_VCC_PIN 4

#define DHT_PIN 5
#define DHTTYPE DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHT_PIN, DHTTYPE);

uint32_t dhtDelayMS;

// OUTPUT pins
#define ATOMIZER_1_PIN 6
#define ATOMIZER_2_PIN 7
#define WATER_PUMP_PIN 8
#define COOLING_FAN_PIN 9

// EC sensor global init
float ecVoltage, ecValue;

// Water level init
int water_level = 0;

// Tank level init
float tank_level = 0;

// Temperature and humidity
// TODO: In the absence of a temperature probe, change this value to the ambient room temperature (in celsius)!
// tempHumid[1] = 25; // 25 = room temp
float *temp_humid = new float[2];

// Initialize EC EEPROM from DFRobot_EC headers
DFRobot_EC ec;

// Calibration formula is based on linear regression of voltage (x) against pH (y)
// Formula:
//  y = xOffset(x) + yOffset
// Formula is basically in slope-intercept form:
//  y = mx + b

float m = -1.92; //change this value to calibrate
float b = 11.55; //change this value to calibrate

// SENSOR STATE THRESHOLDS
// pH
float pHFloor = 5.5;
float pHCeiling = 6.5;

// EC, in mS/cm
float ecLow = 1.8;
float ecHigh = 2.2;

// Temperature, in °C
float tempLow = 15.7;
float tempHigh = 17.2;

// Humidity, in % * 100
float humidLow = 60.0;

// Water level, in mm^3
float waterLevelHigh = 500.0;
float waterLevelLow = 150.0;

// Tank level, in cm
float tankLevelHigh = 3.0; // 3cm gap between sensor and top of water
float tankLevelLow = 10.0; // 10cm gap, calibrate depending on tank height
// SENSOR STATE THRESHOLDS

// Pump dispense buffer (in ms), calibrate this
int pumpBuffer = 500;

// Atomizer buffer (in ms), calibrate this
int atomizerBuffer = 10000;
  
int samples = 10; // 10 samples per measurement cycle.
float adcResolution = 1024.0;

float *tempHumidTelemetry(){
  sensors_event_t event;
  float *output = new float[2];

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    return NULL;
  } else {
    // Serial.print(F("Temperature: "));
    // Serial.print(event.temperature);
    // Serial.print(F("°C"));
    output[0] = event.temperature;
  }
  
  // Serial.print(" | "); // Separator

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
    return NULL;
  } else {
    // Serial.print(F("Humidity: "));
    // Serial.print(event.relative_humidity);
    // Serial.println(F("%"));
    output[1] = event.relative_humidity;
  }

  return output;
}

int waterLevelTelemetry(){
  digitalWrite(WATER_LEVEL_VCC_PIN, HIGH);  // turn the sensor ON
  delay(10);                      // wait 10 milliseconds
  int value = analogRead(WATER_LEVEL_SENS_PIN); // read the analog value from sensor
  digitalWrite(WATER_LEVEL_VCC_PIN, LOW);   // turn the sensor OFF

  return value;
}

int tankLevelTelemetry(){
  float  distance = distanceSensor.measureDistanceCm();
  Serial.print(F("Tank Level: "));
  Serial.println(distance);
}

float getpH(float pHVoltage) {
    return m * pHVoltage + b;
}

void resetRelay(int pin){
  digitalWrite(pin, LOW);
}

void toggleRelay(int pin){
  digitalWrite(pin, !digitalRead(pin));
  // delay(ms);
}

void setup() {
  // SET PINMODES
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(WATER_LEVEL_VCC_PIN, OUTPUT);
  digitalWrite(WATER_LEVEL_VCC_PIN, LOW);

  pinMode(ATOMIZER_1_PIN, OUTPUT);
  pinMode(ATOMIZER_2_PIN, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(COOLING_FAN_PIN, OUTPUT);
  // SET PINMODES
  
  // TURN OFF OUTPUTS
  resetRelay(ATOMIZER_1_PIN);
  resetRelay(ATOMIZER_2_PIN);
  resetRelay(WATER_PUMP_PIN);
  resetRelay(COOLING_FAN_PIN);
  // TURN OFF OUTPUTS

  Serial.begin(9600);
  ec.begin();
  
  // DHT22 SETUP
  dht.begin();
  sensor_t dht_sensor;

  dht.temperature().getSensor(&dht_sensor);
  dht.humidity().getSensor(&dht_sensor);
  // DHT22 SETUP

  // Set delay between sensor readings based on sensor details.
  dhtDelayMS = dht_sensor.min_delay / 1000;
  delay(1000);
}

void loop() {
  start:
  tank_level = tankLevelTelemetry();
  if(tank_level >= tankLevelLow){
    Serial.print(F("Tank Level: "));
    Serial.print(tank_level);
    Serial.println(" | Tank level too low!");
    Serial.println("Fill tank with water");
  } else{
    // MEASURE PH AND EC
    int pHMeasurements = 0;
    int ecMeasurements = 0;
    float temperatureMeasurements = 0;

    Serial.println("Reading pH and EC!");
    for (int i = 0; i < samples; i++) {
      Serial.print("Reading sample ");
      Serial.println(i+1);
      pHMeasurements += analogRead(PH_SENS_PIN);
      ecMeasurements += analogRead(EC_SENS_PIN)/adcResolution*5000;
      temperatureMeasurements = tempHumidTelemetry()[0];

      delay(dhtDelayMS);
    }

    float pHVoltage = 5.0 / adcResolution * pHMeasurements / samples;
    float pHValue = getpH(pHVoltage);
    
    float ecVoltage = ecMeasurements / samples;

    temp_humid[0] = temperatureMeasurements / samples;
    
    float ecValue = ec.readEC(ecVoltage, temp_humid[0]);

    Serial.print(F("pH: "));
    Serial.println(pHValue);
    Serial.print(F("EC: "));
    Serial.println(ecValue);

    if(pHValue < pHFloor && ecValue < ecLow){
      Serial.println("pH or EC too low!");
      Serial.println("Fill tank with nutrient solution.");
    } else{
      Serial.println("pH and EC values good!");
        fillAtomizerTank:
        water_level = waterLevelTelemetry();

        if(water_level < waterLevelLow){
          Serial.println("Atomizer tank water level too low!.");
          Serial.println("Pumping water from main tank.");
          toggleRelay(WATER_PUMP_PIN);
          delay(pumpBuffer);
          toggleRelay(WATER_PUMP_PIN);
        }

        water_level = waterLevelTelemetry();
        if(water_level < waterLevelLow) goto fillAtomizerTank;

        Serial.print(F("Water level : "));
        Serial.println(water_level);

        float *foo = new float[2];
        float *tempHumidMeasurements = new float[2];

        for (int i = 0; i < samples; i++) {
          foo = tempHumidTelemetry();
          tempHumidMeasurements[0] += foo[0];
          tempHumidMeasurements[1] += foo[1];
          
          delay(dhtDelayMS);
        }

        temp_humid[0] = tempHumidMeasurements[0] / samples;
        temp_humid[1] = tempHumidMeasurements[1] / samples;

        Serial.print(F("Temperature: "));
        Serial.print(temp_humid[0]);
        Serial.print(F("°C"));

        Serial.print(" | "); // Separator

        Serial.print(F("Humidity: "));
        Serial.print(temp_humid[1]);
        Serial.println(F("%"));

        if(temp_humid[0] >= tempHigh){
          Serial.println("Temperature too high!");
          if(!digitalRead(COOLING_FAN_PIN)){
            Serial.println("Turning on cooling fan.");
            toggleRelay(COOLING_FAN_PIN);
          }
        } else{
          Serial.println("Temperature within desirable thresholds!");
          Serial.println("Turning off cooling fan.");
          resetRelay(COOLING_FAN_PIN);
        }

        if(temp_humid[1] <= humidLow){
          // TURN ON
          Serial.println("Tank not humid enough!");
          Serial.println("Turning on atomizers.");
          toggleRelay(ATOMIZER_1_PIN);
          toggleRelay(ATOMIZER_2_PIN);
          
          // Wait n miliseconds
          delay(atomizerBuffer);

          // TURN OFF
          Serial.println("Turning off atomizers.");
          toggleRelay(ATOMIZER_1_PIN);
          toggleRelay(ATOMIZER_2_PIN);
        }
    }

  }


  // Get temperature event and print its value.

  // water_level = waterLevelTelemetry();
  // Serial.print(F("Water Level: "));
  // Serial.println(water_level);

}
