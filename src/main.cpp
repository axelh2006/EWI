// ---------------------------------------*** ISOFLUTE by Axel Henriksson ***----------------------------------------------
// An electronic wind instrument MIDI controller with an isomorphic keyboard based on ESP-S3 and the Arduino framework 

#if ARDUINO_USB_MODE
#error "USB mode is still 1 (hardware CDC/JTAG), not TinyUSB"
#endif

// Libraries
#include <Arduino.h>
#include <USB.h>
#include <Preferences.h>
#include <USBMIDI.h>
#include <Button2.h>
#include <Adafruit_LPS28.h>
#include <Adafruit_ADXL345_U.h>
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

bool both = false;

// Mux definitions
const uint8_t sel[3] = { 36, 37, 38}; // Pins for the 3-bit binary address of the multiplexer
const uint8_t z[4] = { 1, 2, 4, 5}; // Multiplexer read channels

int8_t oct = 0;
const int8_t oct_min = -3, oct_max = 3;
uint8_t sounding_note[32];

bool last[32]; // Array to store the last state of each button for debouncing
uint32_t last_change[32]; // Array to store the last change time of each button for debouncing

Adafruit_LPS28 lps; // Create an instance of the LPS28 pressure sensor
uint8_t pressure_lin;
uint8_t pressure; // Variable to store the pressure reading
uint8_t pressure_tresh;
float rest;
float pressure_deadzone_hpa = 1.2f;
float pressure_range_hpa;

enum BreathCurve : uint8_t { LINEAR, SOFT, HARD, S_CURVE, COUNT };
const char* curve_names[COUNT] = { "Linear", "Soft", "Hard", "S-curve" };
uint8_t breath_curve = LINEAR;

Adafruit_ADXL345_Unified acc = Adafruit_ADXL345_Unified();

const float bend_full = 30.0f;
const float bend_deadzone = 4.0f;

float tilt_ref = 0;
float tilt_smooth = 0;
uint16_t last_bend = 8192;
uint32_t bend_time = 0;

float ay = 0.0;
float az = 0.0;

float tilt = 0.0;
int bend = 8192;

bool sendPressure;
bool sendVel;

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

// Preference saving globals
Preferences prefs;
bool prefs_dirty = false;
uint32_t prefs_changed = 0;

// UI globals
enum uiState : uint8_t {UI_OCT, UI_SENS, UI_TRESH, UI_CURVE, UI_CH, UI_SENDPRS, UI_SENDVEL};
uiState ui = UI_OCT;
const uint16_t long_press = 300; // Milliseconds
bool need_full = true;
uint8_t shown_pressure = 255; int shown_bend = -1; bool shown_sounding = false;
bool reset_active = false;
bool reset_drawn = false;
uint32_t reset_start = 0;


void loadSettings() {
  prefs.begin("isoflute", false);
  oct = constrain((int)prefs.getChar("oct", 0), (int)oct_min, (int)oct_max);
  pressure_range_hpa = constrain((int)prefs.getChar("range", 10), 1, 20);
  pressure_tresh = constrain((int)prefs.getChar("treshf", 1), 1, 20);
  breath_curve = constrain((int)prefs.getChar("curve", 0), 0, 3);
  midi_channel = constrain((int)prefs.getChar("ch", 1), 1, 16);
  sendPressure = prefs.getBool("prs", false);
  sendVel = prefs.getBool("vel", false);
}

void saveSettings() {
  prefs.putChar("oct", oct);
  prefs.putChar("range", pressure_range_hpa);
  prefs.putChar("treshf", pressure_tresh);
  prefs.putChar("curve", breath_curve);
  prefs.putChar("ch", midi_channel);
  prefs.putBool("prs", sendPressure);
  prefs.putBool("vel", sendVel);
  prefs_dirty = false;
}

void markSettingsDirty() {
  prefs_dirty = true;
  prefs_changed = millis();
}

float applyCurve(float x) {
  x = constrain(x, 0.0f, 1.0f);
  switch (breath_curve) {
    case SOFT: return x * x;
    case HARD: return sqrtf(x);
    case S_CURVE: return x * x * (3.0f - 2.0f * x);
    default: return x;
  }
}

bool anyNoteSounding() {
  for (uint8_t k = 0; k < 32; k++) if (sounding_note[k] != 255) return true;
  return false;
}

void sendNote(bool on, uint8_t note, uint8_t vel) {
  if (sendVel) vel = constrain(vel, 1, 127);
  else vel = 64;

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

void sendPitchBend(int16_t value) { // Pitch Bend, 0 - 16383, 8192 = centre
  value = map(constrain(value, 0, 16383), 0, 16383, 16383, 0);
  // HID
  uint8_t msg[3] = { (uint8_t)(0xE0 | ((midi_channel - 1) & 0x0F)),
                     (uint8_t)(value & 0x7F),
                     (uint8_t)(value >> 7) };
  Serial1.write(msg, 3);
  // USB
  usb_midi.pitchBend(((float)value - 8192.0f) / 8192.0f, midi_channel);
}

void captureTiltRef() {
  tilt_ref = tilt_smooth;
  last_bend = 8192;
  sendPitchBend(8192);
}

float readAngle() {
  ay = acc.getY();
  az = acc.getZ();

  tilt = atan2f(ay, az) * 57.29578f;

  return tilt;
}

float angleDiff(float current, float reference) {
  float d = current - reference;
  if (d > 180.0f) d -= 360.0f;
  else if (d < -180.0f) d += 360.0f;
  return d;
}

void updateTilt() {
  static uint32_t last_read = 0;
  uint32_t now = millis();
  if (now - last_read < 10) return;
  last_read = now;

  float a = readAngle();
  tilt_smooth += 0.25f * angleDiff(a, tilt_smooth);

  if (!anyNoteSounding()) { bend = 8192; return; }

  float d = angleDiff(tilt_smooth, tilt_ref);
  if (fabs(d) < bend_deadzone) d = 0;
  else d -= (d > 0 ? bend_deadzone : -bend_deadzone);

  bend = 8192 + (int)(d * 8192.0f / (bend_full - bend_deadzone));
  bend = constrain(bend, 0, 16383);

  bool moved = abs(bend - (int)last_bend) >= 16;
  bool recentered = (bend == 8192 && last_bend != 8192);
  if ((moved || recentered) && now - bend_time >= 10) {
    sendPitchBend(bend);
    last_bend = bend;
    bend_time = now;
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
          if (!anyNoteSounding()) captureTiltRef();
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
  if (!sendPressure) {
    uint8_t msg[2] = { (uint8_t)(0xD0 | ((midi_channel - 1) & 0x0F)), value }; 
    Serial1.write(msg, 2);

    usb_midi.channelPressure(value, midi_channel);
  } else {
    uint8_t msg1[3] = { (uint8_t)(0xB0 | ((midi_channel - 1) & 0x0F)), 2, value };
    Serial1.write(msg1, 3);

    usb_midi.controlChange(2, value, midi_channel);  
  }
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
  markSettingsDirty();
}

void changeChannel(int8_t delta) {
  releaseAll();
  sendChannelPressure(0);
  midi_channel = ((midi_channel - 1 + delta + 16) % 16) + 1;
  last_pressure_sent = 255;
  markSettingsDirty();
}

void changeCurve(int8_t delta) {
  breath_curve = (breath_curve + delta + COUNT) % COUNT;
  markSettingsDirty();
}

void onA(Button2&) {
  switch(ui) {
    case UI_OCT: changeOctave(-1); need_full = true; break;
    case UI_SENS: 
      pressure_range_hpa--; 
      if (pressure_range_hpa < 1) pressure_range_hpa = 1;
      need_full = true; 
      markSettingsDirty(); 
      break;
    case UI_TRESH: 
      pressure_tresh--; 
      if (pressure_tresh < 1) pressure_tresh = 1;
      need_full = true;
      markSettingsDirty(); 
      break;
    case UI_CURVE: changeCurve(-1); need_full = true; break;
    case UI_CH: changeChannel(-1); need_full = true; break;
    case UI_SENDPRS: sendPressure = true; need_full = true; markSettingsDirty(); break;
    case UI_SENDVEL: sendVel = false; need_full = true; markSettingsDirty(); break;
  }
}

void onBReleased(Button2& b) {
  bool isLong = b.wasPressedFor() >= long_press;
  switch(ui) {
    case UI_OCT: 
      if (isLong) ui = UI_SENS; else changeOctave(+1); need_full = true;
      break;
    case UI_SENS:
      if (isLong) ui = UI_TRESH; else {
        pressure_range_hpa++;
        if (pressure_range_hpa > 20) pressure_range_hpa = 20;              
      }
      markSettingsDirty();
      need_full = true;
      break;
    case UI_TRESH:
      if (isLong) ui = UI_CURVE; else {
        pressure_tresh++;
        if (pressure_tresh > 20) pressure_tresh = 20;        
      }
      markSettingsDirty();
      need_full = true;
      break;
    case UI_CURVE:
      if (isLong) ui = UI_CH; else changeCurve(+1); need_full = true;
      break;
    case UI_CH: 
      if (isLong) ui = UI_SENDPRS; else changeChannel(+1); need_full = true;
      break;
    case UI_SENDPRS:
      if (isLong) ui = UI_SENDVEL; else sendPressure = false; need_full = true;
      markSettingsDirty();
      break;
    case UI_SENDVEL:
      if (isLong) ui = UI_OCT; else sendVel = true; need_full = true;
      markSettingsDirty();
      break;
  }
}

void checkBoth(){
  bool a = btnA.isPressed(), b = btnB.isPressed();
  if ( a && b && !both) {
    both = true;
    rest = lps.getPressure();
    reset_active = true;
    reset_drawn = false;
    reset_start = millis();
    ui = UI_OCT;
  } else if ( !a && !b) {
    both = false;
  }
}

void updateDisplay() {
  display.clearBuffer();  

  if (reset_active) {
    if (millis() - reset_start < 1000) {
      if (!reset_drawn) {
        display.clearBuffer();
        
        display.drawRFrame(37, 24, 54, 18, 2);
        display.setCursor(44, 38);
        display.print("RESET");

        display.sendBuffer();
        reset_drawn = true;
      }
      return;
    }
    reset_active = false;
    reset_drawn = false;
    need_full = true;
  }

  // Menu handling
  switch (ui) {      
    case UI_SENS:
      display.drawRFrame(28, 24, 72, 18, 2);
      display.setCursor(32, 38);
      display.print("RANGE:");
      display.setCursor(81, 38);
      display.print(pressure_range_hpa, 0);
      break;
    case UI_TRESH:
      display.drawRFrame(24, 24, 80, 18, 2);
      display.setCursor(28, 38);
      display.print("TRESH:");
      display.setCursor(77, 38);
      display.print(pressure_tresh, 1);
      break;
    case UI_CURVE:
      display.drawRFrame(4, 24, 120, 18, 2);
      display.setCursor(11, 38);
      display.print("CURVE:");
      display.setCursor(60, 38);
      display.print(curve_names[breath_curve]);
      break;
    case UI_CH:
      display.drawRFrame(37, 24, 50, 18, 2);
      display.setCursor(41, 38);
      display.print("CH:");
      display.setCursor(68, 38);
      display.print(midi_channel);
      break;
    case UI_SENDPRS:
      display.drawRFrame(31, 24, 66, 18, 2);
      display.setCursor(35, 38);
      display.print("SEND");
      display.setCursor(70, 38);
      if (sendPressure) display.print("PRS");
      else if (!sendPressure) display.print("AFT");
      break;
    case UI_SENDVEL:
      display.drawRFrame(31, 24, 66, 18, 2);
      display.setCursor(35, 38);
      display.print("VEL:");
      display.setCursor(70, 38);
      if (sendVel) display.print("YES");
      else if (!sendVel) display.print("NO");
      break;
    default:
      for (int i = 0; i < 7; i++) {
        display.drawCircle(i*15+19, 58, 4);
      }
      display.drawDisc((oct)*15+65, 58, 4);

      int dotsize = map(pressure, 0, 127, 2, 15);
      int doty = map(bend, 0, 16383, 54, 10);
      uint32_t now = millis();
      int dotx = SCREEN_WIDTH / 2 + (int)(map(mod, 0, 127, 0, 10) * sinf(millis() / 35.0f));
      if (anyNoteSounding()) display.drawDisc(dotx, doty, dotsize);
      else display.drawCircle(dotx, doty, dotsize);
      break;
  }

  // Only update the whole screen if it needs to
  if (need_full) {
    display.sendBuffer();
    need_full = false;
  } else {
    display.updateDisplayArea(4, 0, 8, 8);
  }
}

void setup() { 
  
  USB.productName("ISOFLUTE");
  USB.manufacturerName("Axel Henriksson");
  USB.begin(); // Initialize USB communication   
  usb_midi.begin(); // Initialize USB MIDI communication
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial1.begin(31250, SERIAL_8N1, MIDI_RX, MIDI_TX); // Initialize MIDI communication at 31250 baud rate

  loadSettings();

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
    display.setCursor(20, 20);
    display.print("NO LPS");
    display.sendBuffer();
    while (1) delay(10);
  }

  // Check if accelerometer is available
  if (!acc.begin()) {
    Serial.println("Failed to find ADXL345 sensor!");
    display.setCursor(20, 35);
    display.print("NO ADXL");
    display.sendBuffer();
    while (1) delay(10);
  }

  // Set the highest ODR (200 Hz) and 8-sample averaging
  lps.setDataRate(LPS28_ODR_200_HZ);
  lps.setAveraging(LPS28_AVG_8);
  lps.setFullScaleMode(true);
  delay(100);
  rest = lps.getPressure();

  Serial.print("LPS28 sensor found!");

  // Set range and data rate for accelerometer
  acc.setRange(ADXL345_RANGE_2_G);
  acc.setDataRate(ADXL345_DATARATE_100_HZ);

  //ax = acc.getX();
  //ay = acc.getY();
  //az = acc.getZ();

  //tilt_smooth = readAngle();

  Serial.println("ADXL345 sensor found!");

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
  display.setFont(u8g2_font_pxplusibmvga8_mf);
}

void loop() {
  btnB.loop();
  btnA.loop();
  checkBoth();

  // Pressure handling
  if (lps.getStatus() & LPS28_STATUS_PRESS_READY) { // Pressure data available
    float p = lps.getPressure();
    uint32_t t_now = millis();

    static uint32_t below_since = 0;
    float diff = p - rest;
    if (diff < -0.5f) {
    // a dip below rest: don't follow it, unless it lasts long enough to be a real change in ambient pressure
      if (below_since == 0) below_since = t_now;
      if (t_now - below_since > 10000) { rest = p; below_since = 0; }
    } else {
      below_since = 0;
      if (diff < pressure_deadzone_hpa) rest += 0.003f * diff;    // time constant about 1.7 s
    }

    float x = (p - rest - pressure_deadzone_hpa) / pressure_range_hpa;
    pressure_lin = (uint8_t)constrain((int)(x * 127.0f), 0, 127);
    pressure = (uint8_t)constrain((int)(applyCurve(x) * 127.0f + 0.5f), 0, 127);

    uint32_t now = millis();
    if (pressure != last_pressure_sent && now - last_pressure_time >= 10) {
      sendChannelPressure(pressure);
      last_pressure_sent = pressure;
      last_pressure_time = now;
    }
  }

  // Modulation pad handling
  static float mod_f = 0;
  mod_f += 0.2f * (analogRead(TOUCH_PIN) - mod_f);
  mod = constrain(map((int)(mod_f * 127 / 4095), 50, 127, 0, 127), 0, 127);
  if (abs((int)mod - (int)last_mod) >= 2 || (mod == 0 && last_mod != 0)) {
    sendModulation(mod > 5 ? mod : 0);
    last_mod = mod;
  }

  updateTilt();
  
  // Key handling
  uint32_t now = millis(); // Get the current time in milliseconds
  if (pressure_lin > pressure_tresh) { // If pressure is below threshold, skip processing
    scanKeys();
  } else {
    releaseAll();
  }

  // Update display
  uint32_t t = millis();
  bool sounding = anyNoteSounding();
  bool dirty = need_full || mod > 5 || pressure != shown_pressure || bend != shown_bend || sounding != shown_sounding || reset_active;
  if (dirty && t - last_display_update_time >= 40) {
    last_display_update_time = t;
    shown_pressure = pressure; shown_bend = bend; shown_sounding = sounding;
    updateDisplay();
  }

  // Save settings
  if (prefs_dirty && millis() - prefs_changed > 2000 && !anyNoteSounding()) saveSettings();
}

