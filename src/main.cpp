// ---------------------------------------*** ISOFLUTE by Axel Henriksson ***----------------------------------------------
// An electronic wind instrument MIDI controller with an isomorphic keyboard based on ESP-S3 and the Arduino framework 

#if ARDUINO_USB_MODE
#error "USB mode is still 1 (hardware CDC/JTAG), not TinyUSB"
#endif

// Libraries
#include <Arduino.h>
#include <USB.h>
#include <USBMIDI.h>
#include <Button2.h>
#include <Adafruit_LPS28.h>
#include <U8g2lib.h>
#include <mappings.h>

// Screen definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define BUTTONA_PIN 7
#define BUTTONB_PIN 8

#define TOUCH_PIN 6

Button2 btnB, btnA;

// Mux definitions
const uint8_t sel[3] = { 36, 37, 38}; // Pins for the 3-bit binary address of the multiplexer
const uint8_t z[4] = { 1, 2, 4, 5}; // Multiplexer read channels

int8_t oct = 0;
const int8_t oct_min = -3, oct_max = 3;
uint8_t sounding_note[32];

bool last[32]; // Array to store the last state of each button for debouncing
uint32_t last_change[32]; // Array to store the last change time of each button for debouncing

Adafruit_LPS28 lps; // Create an instance of the LPS28 pressure sensor
float baseline;
uint8_t pressure; // Variable to store the pressure reading
uint8_t pressure_tresh = 2;
float rest;
const float pressure_deadzone_hpa = 1.2f;
const float pressure_range_hpa = 12.0f;

uint8_t  last_pressure_sent = 255;      // 255 forces the first send
uint32_t last_pressure_time = 0;

uint8_t mod; // Variable to store the mod pad reading
uint8_t last_mod = 127;

// MIDI definitions
constexpr int MIDI_TX = 13; // Pin for MIDI transmission
constexpr int MIDI_RX = 14; // Pin for MIDI reception
uint8_t midi_channel = 1; // Default MIDI channel (1-16)

USBMIDI usb_midi; // Create a USBMIDI object for USB MIDI communication

// Initialize the display object for the SSD1306 OLED display using hardware I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
uint32_t last_display_update_time = 0;

// UI globals
enum uiState : uint8_t {UI_OCT, UI_CH};
uiState ui = UI_OCT;
const uint8_t ui_items = 2;
const uint16_t long_press = 300; // Milliseconds
bool need_full = true;

void sendNote(bool on, uint8_t note, uint8_t vel) {
  vel = constrain(vel, 1, 127);
  // DIN
  uint8_t status = (on ? 0x90 : 0x80) | ((midi_channel - 1) & 0x0F); // Note on/off message
  uint8_t velmsg = on ? vel : 0;
  uint8_t msg[3] = { status, note, velmsg }; // MIDI message array
  Serial1.write(msg, 3);

  // USB
  if (on) {
    usb_midi.noteOn(note, vel, midi_channel);
  } else {
    usb_midi.noteOff(note, 0, midi_channel);
  }
}

void releaseAll() {
  for (uint8_t k = 0; k < 32; k++) {
    if (sounding_note[k] != 255) {
      sendNote(false, sounding_note[k], 0);
      sounding_note[k] = 255;
    }
    last[k] = false;
  }
}

void scanKeys() {
  uint32_t now = millis();
  for (uint8_t ch = 0; ch < 8; ch++) {
    digitalWrite(sel[0], ch & 1);
    digitalWrite(sel[1], (ch >> 1) & 1);
    digitalWrite(sel[2], (ch >> 2) & 1);
    delayMicroseconds(10);

    for (uint8_t m = 0; m < 4; m++) {
      uint8_t k = m * 8 + ch;
      uint8_t key = key_to_number[k];
      if (key == no_note) continue;

      bool pressed = !digitalRead(z[m]);
      if (pressed == last[k] || now - last_change[k] < 5) continue;
      last[k] = pressed;
      last_change[k] = now;

      if (pressed) {
        int n = key_to_note_wickihaydn[key] + oct * 12;
        if (n >= 0 && n <= 127) {
          sounding_note[k] = n;
          sendNote(true, n, pressure);
        }
      } else if (sounding_note[k] != 255) {
        sendNote(false, sounding_note[k], 0);
        sounding_note[k] = 255;
      }
    }
  }
}

void sendChannelPressure(uint8_t value) {
  // HID
  uint8_t msg[2] = { (uint8_t)(0xD0 | ((midi_channel - 1) & 0x0F)), value }; 
  Serial1.write(msg, 2);
  uint8_t msg1[3] = { (uint8_t)(0xB0 | ((midi_channel - 1) & 0x0F)), 2, value };
  Serial1.write(msg1, 3);

  // USB
  usb_midi.channelPressure(value, midi_channel);
  usb_midi.controlChange(2, value, midi_channel);
}

void sendModulation(uint8_t value) {
  // HID
  uint8_t msg1[3] = { (uint8_t)(0xB0 | ((midi_channel - 1) & 0x0F)), 1, value };
  Serial1.write(msg1, 3);

  // USB
  usb_midi.controlChange(1, value, midi_channel);
}

void changeOctave(int8_t delta) {
  oct = constrain((int)oct + delta, (int)oct_min, (int)oct_max);
}

void changeChannel(int8_t delta) {
  sendChannelPressure(0);
  midi_channel = ((midi_channel - 1 + delta + 16) % 16) + 1;
  last_pressure_sent = 255;
}

void onA(Button2&) {
  switch(ui) {
    case UI_OCT: changeOctave(-1); need_full = true; break;
    case UI_CH: changeChannel(-1); need_full = true; break;
  }
}

void onBReleased(Button2& b) {
  bool isLong = b.wasPressedFor() >= long_press;
  switch(ui) {
    case UI_OCT: 
      if (isLong) ui = UI_CH; else changeOctave(+1); need_full = true;
      break;
    case UI_CH: 
      if (isLong) ui = UI_OCT; else changeChannel(+1); need_full = true;
      break;
  }
}

void checkBoth(){
  if (btnA.isPressed() && btnB.isPressed()) rest = lps.getPressure();
}

void updateDisplay() {
  display.clearBuffer();
  display.drawRBox(0, 0, 10, map(pressure, 0, 127, 4, SCREEN_HEIGHT), 2); // Pressure indicator
  display.drawRBox(SCREEN_WIDTH - 10, 0, 10, map(mod, 0, 127, 4, SCREEN_HEIGHT), 2); // Mod indicator
  display.setCursor(20, 30);
  display.print("Oct");
  display.setCursor(70, 30);
  display.print(oct);
  display.setCursor(20, 62);
  display.print("Ch");
  display.setCursor(70, 62);
  display.print(midi_channel);
  if (ui == UI_OCT) {
    display.drawLine(20, 31, 108, 31);
  } else if (ui == UI_CH) {
    display.drawLine(20, 63, 108, 63);
  }

  // Only update the whole screen if it needs to
  if (need_full) {
    display.sendBuffer();
    need_full = false;
  } else {
    display.updateDisplayArea(0, 0, 2, 8);    // left bar: tiles 0-1, full height
    display.updateDisplayArea(14, 0, 2, 8);   // right bar: tiles 14-15
  }
}

void setup() { 
  
  USB.productName("ISOFLUTE");
  USB.manufacturerName("Axel Henriksson");
  USB.begin(); // Initialize USB communication   
  usb_midi.begin(); // Initialize USB MIDI communication
  Serial.begin(9600); // Initialize serial communication for debugging
  Serial1.begin(31250, SERIAL_8N1, MIDI_RX, MIDI_TX); // Initialize MIDI communication at 31250 baud rate

  // Initialize buttons
  btnA.begin(BUTTONA_PIN);
  btnB.begin(BUTTONB_PIN);

  btnB.setPressedHandler(onA);
  btnA.setReleasedHandler(onBReleased);

  // Mux setup
  for(uint8_t i = 0; i < 3; i++) pinMode(sel[i], OUTPUT); // Set multiplexer select pins as outputs
  for(uint8_t m = 0; m < 4; m++) pinMode(z[m], INPUT_PULLUP); // Set multiplexer read pins as inputs with pull-up resistors

  Wire.begin(9, 10); // Initialize I2C with custom SDA and SCL pins
  Wire.setClock(400000);
  
  Serial.println("Starting...");

  

  // Check if the display is availabe and initialized
  if (!display.begin()) {
    Serial.println("Display initialization failed");
    while (true) {
      delay(100);
    }
  }

  display.clearBuffer();
  display.enableUTF8Print();
  display.setFont(u8g2_font_logisoso28_tf);

  Serial.println("Display initialized");

  // Check if pressure sensor is available and initialized
  if (!lps.begin()) {
    Serial.println("Failed to find LPS28 sensor!");
    display.setCursor(20, 32);
    display.print("NO SENSOR");
    while (1) delay(10);
  }

  // Set the highest ODR (200 Hz) and 8-sample averaging
  lps.setDataRate(LPS28_ODR_200_HZ);
  lps.setAveraging(LPS28_AVG_8);
  lps.setFullScaleMode(true);
  delay(100);
  rest = lps.getPressure();

  Serial.print("LPS28 sensor found! Baseline: ");
  Serial.println(baseline);

  
  delay(500);
  Serial.println("Welcome to ISOFLUTE by Axel Henriksson");

  display.setCursor(40, 32);
  display.print("ISO");
  display.sendBuffer();
  delay(500);
  display.setCursor(20, 64);
  display.print("FLUTE");
  display.sendBuffer();
  delay(500);
  
  Serial.println("Commencing...");
  display.clear();
}

void loop() {
  btnB.loop();
  btnA.loop();
  checkBoth();

  // Pressure handling
  if (lps.getStatus() & LPS28_STATUS_PRESS_READY) { // Pressure data available
    float p = lps.getPressure();
    if (p - rest < pressure_deadzone_hpa) rest += 0.02f * (p - rest);
    float x = (p - rest - pressure_deadzone_hpa) / pressure_range_hpa;
    pressure = (uint8_t)constrain((int)(x * 127.0f), 0, 127);

    uint32_t now = millis();
    if (pressure != last_pressure_sent && now - last_pressure_time >= 10) {
      sendChannelPressure(pressure);
      last_pressure_sent = pressure;
      last_pressure_time = now;
    }
  }

  // Modulation pad handling
  mod = map((analogRead(TOUCH_PIN)), 0, 4096, 0, 127);
  //Serial.println(analogRead(TOUCH_PIN));
  //Serial.println(mod);
  if (mod <= 50) mod = 50;
  mod = constrain(map(mod, 50, 127, 0, 127), 0, 127);
  if (mod != last_mod) {
    if (mod > 5) {
      sendModulation(mod);      
    } else sendModulation(0);
    last_mod = mod;
  }
  
  // Key handling
  uint32_t now = millis(); // Get the current time in milliseconds
  if (pressure > pressure_tresh) { // If pressure is below threshold, skip processing
    scanKeys();
  } else {
    releaseAll();
  }

  // Update display
  uint32_t t = millis();
  if (now - last_display_update_time >= 20){
    last_display_update_time = t;
    updateDisplay();
  }

}

