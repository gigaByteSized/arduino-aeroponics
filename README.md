# Arduino IoT: Aeroponics 

Aeroponics system thru IoT using Arduino, PH-4502C pH Sensor, DFR0300 EC Sensor, AM2302 (DHT11) Sensor, and a water level sensor.

## Getting started

1. Install `arduino` Arduino IDE using this [link](https://www.arduino.cc/en/software/).

2. Clone this repository using `git` or download it as a zip using this [link](https://github.com/gigaByteSized/arduino-hydroponics/archive/refs/heads/main.zip)

```
git clone https://github.com/gigaByteSized/arduino-hydroponics.git
``` 

3. Extract the zip file 

4. In `arduino`, install the included `DFRobot_EC-master.zip` in `Sketch > Include library > Add .ZIP library...`

5. Additionally, using the Arduino Library Manager, install `HCSR04` by Martin Sosic and `DHT Sensor Library` and `Adafruit Unified Sensor` by Adafruit.

6. You are now good to go!

## Additional Setup

### Logging with PuTTY

1.1. Install `putty` using
```
sudo apt-get install putty putty-tools
```

1.2. Alternatively, if using a Windows machine, install `putty` using this [link](https://the.earth.li/~sgtatham/putty/latest/w64/putty-64bit-0.80-installer.msi).

2. Check serial line using Arduino IDE.

```
/dev/tty/USB0.
``` 

3. In PuTTY client, set connection type to 'Serial'. Make sure 'Serial line' is set to correct 'Serial line' and 'speed'  (baud) is set to '9600'.

4. Go to Serial > Logging (In left "Categories"). Set session logging to 'Printable output'

5. You are now good to go!