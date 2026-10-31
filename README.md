# ISOFLUTE by Axel Henriksson
**A MIDI controller EWI with an isomorphic keyboard based on ESP32-S3 and the arduinoesp32 framework**
## Features
* 28 Keys arranged in 4 rows of 7 keys, using the Wicki-Hayden layout
* Sends MIDI pressure CC and aftertouch based on air pressure sensor readings
* Pressure sensor pad for MIDI modulation
* Sends pitch bend based on tilt
* Works with both USB MIDI and TRS MIDI (Type A)
* Buttons for changing octave, MIDI channel and setting pressure baseline
* Simple UI that shows all parameters, and settings for pressure/aftertouch and velocity which are written to flash memory.
* 3D-printed enclosure and keycaps

## Parts used
* WaveShare ESP-S3-PICO development board
* Adafruit LPS28 air pressure sensor module
* ADXL345 accelerometer module
* SSD1306 OLED module
* 28x Kailh black mechanical keyboard switches
* 4x 74HC4051 8ch analog multiplexers
* 3.5mm TRS audio jack
* 0-2kg FSR pressure sensor (too small, a FSR with higher pressure range should be used)
* Some resistors for MIDI out circuit and I2C pullup

## Future plans
* New version with PCB instead of protoboard
* Different pressure sensor for reduced cost
* Integrated battery?
* Maybe LED indicators instead of screen
* Smaller footprint

## The code
The code is written in VS Code using PlatformIO. I decided on using the Arduino framework instead of ESP-IDF because of previous experience.
### Libraries used:
* Adafruit LPS28
* Adafruit ADXL345 Unified
* U8G2 by olikraus
* Button2 by Lennart Hennings
