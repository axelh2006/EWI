# ISOFLUTE by Axel Henriksson
**An EWI with an isomorphic keyboard based on ESP32-S3 and the Arduino framework**
## Features
* 28 Keys
* Sends MIDI pressure CC and aftertouch based on air pressure sensor readings
* Pressure sensor pad for MIDI modulation
* Sends pitch bend based on tilt
* Works with both USB MIDI and TRS MIDI (Type A)
* Buttons for changing octave, MIDI channel and setting pressure baseline
* Simple UI that shows all parameters
* 3D-printed enclosure and keycaps

## Parts used
* WaveShare ESP-S3-PICO
* Adafruit LPS28 air pressure sensor module
* ADXL345 accelerometer module
* SSD1306 OLED module
* 28x Kailh black mechanical keyboard switches
* 4x 74HC4051 8ch analog multiplexers
* 3.5mm TRS audio jack
* 0-2kg FSR pressure sensor (too small, a FSR with higher pressure range should be used)
* Some resistors :)
