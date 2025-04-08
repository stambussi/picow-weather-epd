# RPi Pico W E-Paper Weather Display

A low-power weather display using a Raspberry Pi Pico W and a 7.5" E-Paper display. Weather data is fetched from the OpenWeatherMap API. This project is a port of lmarzen's [ESP32 E-Paper Weather Display](https://github.com/lmarzen/esp32-weather-epd) to the Pico W platform. All credit goes to him for the original project.

Most of the setup steps and needed hardware is the same as lmarzen's project, except for the usage of the RPi Pico W. PlatformIO is also still used, but with the earlephilhower arduino-pico core. Additional changes are noted below.

## Changes from Original Project
These changes are a combination of unused things I removed and changes to wake-up behavior and error handling.
+ No Battery
  + Since this display is intended to stay plugged in at all times, I removed a lot of the code that handles the battery logic. This saves on Flash and RAM space as the Pico W only has 2MB of flash and 264KB of SRAM compared to the ESP32-E's 4MB of flash and 520KB of SRAM.
+ Not using BME280 sensor
  + I am not using the BME280 chip currently. I left the BME280-specific code mostly untouched so it can be added pretty easily if desired.
+ Updated control logic
  + I changed much of the main control loop so that each WiFi connection attempt, NTP sync, etc. is tried at least twice. I found that this increased the success rate (e.g. not encountering a transient error) of each wake-up to 100%.
+ Simple low-power sleep function
  + This is a first attempt at implementing a low-power sleep mode for the Pico W within the arduino-pico framework. It undervolts and downclocks the Pico W CPU and then performs a low-power WFE to wait for the specified amount of time before returning the voltage and frequency to their defaults. At the time of writing this, there is progress towards a proper low-power sleep mode in the arduino-pico core, but it is unfinished.
+ Error handling
  + As noted above, each attempt in the main control loop is tried twice before entering an error state. When an error occurs, instead of filling the entire screen with the error, the error overwrites only the alert section of the display. This way, the outdated (but probably accurate) weather data is still displayed on the screen.