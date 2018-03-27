# Ardunio code for openflexure motor controller

## Arduino Hardware
Tested with
1. Arduino nano ATmega328p

## Arduino compiler version
Requires version 1.6.2 or higher
(Note the Ubuntu/Debian package version 2:1.0.5, is version 1.0.5 and hence will **NOT** work!)

## Install libraries for Adafruit
Adafruit libraries are required for light sensor support. These libraries can be installed in the Aruino library directory (normally `~/Arduino/libraries` on Linux or `C:\My Documents\Arduino\libraries` on Windows) and can be found on the [adafruit github](https://github.com/adafruit).
The easiest method to install is

	cd ~/Arduino/libraries
	git clone https://github.com/adafruit/Adafruit_TSL2591_Library.git
	git clone https://github.com/adafruit/Adafruit_Sensor.git
	git clone https://github.com/adafruit/Adafruit_ADS1X15.git
