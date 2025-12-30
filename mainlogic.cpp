#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>

// Anzahl Boards
const int NUM_BOARDS = 5;

// Chip-Select Pins für MCP3008
int csPins[NUM_BOARDS] = {5, 16, 17, 4, 2};

// I2C-Adressen der MCP23017
int mcpAddr[NUM_BOARDS] = {0x20, 0x21, 0x22, 0x23, 0x24};

// MCP23017 Objekte
Adafruit_MCP23X17 mcp[NUM_BOARDS];

// LED / SK6812 (ein Pin für alle LEDs)
const int LED_PIN = 15; // Datenpin für SK6812 (anpassen)
const int LEDS_PER_BOARD = 32; // 16 Buttons + 8 Fader * 2 (oben/unten)
const int TOTAL_LEDS = NUM_BOARDS * LEDS_PER_BOARD;
Adafruit_NeoPixel strip(TOTAL_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// DMX buffer (1..512)
uint8_t dmxData[513];

// Fader-Glättung
float smoothVal[NUM_BOARDS][8];
const float SMOOTH_ALPHA = 0.20; // 0..1 (kleiner = stärker glätten)
const int ACTIVE_THRESHOLD = 10; // DMX> -> gilt als "aktiv"

// Button-Debounce / Toggle
bool lastRawButton[NUM_BOARDS][16];
bool debouncedButton[NUM_BOARDS][16];
unsigned long lastBounceTime[NUM_BOARDS][16];
const unsigned long DEBOUNCE_MS = 50;

// Toggle-Modus & Zustand pro Button
bool toggleMode[NUM_BOARDS][16]; // false = momentary, true = toggle; default alle false
bool toggledState[NUM_BOARDS][16];

// Hilfsfarbe: blau / grün
inline uint32_t colorBlue(uint8_t intensity) { return strip.Color(0, 0, intensity); }
inline uint32_t colorGreen(uint8_t intensity) { return strip.Color(0, intensity, 0); }

// --- RS485 / DMX over TTL module ---
const int RS485_TX_PIN = 13; // Datenpin ESP32 TX for Serial1 (anpassen falls belegt)
const int RS485_RX_PIN = 12; // optional, RX for Serial1 (anpassen)
const int RS485_DE_PIN = 14; // DE/RE Steuerung; falls Modul auto-direction, unbenutzt lassen


// ---------------- MCP3008 LESEN ----------------
int readMCP3008(int csPin, int channel) {
  // Standard 3-Byte MCP3008 Transaktion (Start, Single-ended+Channel, receive)
  digitalWrite(csPin, LOW);
  SPI.transfer(0x01); // Start bit
  uint8_t high = SPI.transfer(0x80 | (channel << 4)); // Single-ended + channel
  uint8_t low  = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);

  int result = ((high & 0x03) << 8) | low; // 0..1023
  return result;
}


// ---------------- DMX-Basisadresse pro Board ----------------
int dmxBase(int board) {
  return board * 32; // 0, 32, 64, 96, 128
}

// send DMX via Serial1 using manual Break (toggle TX pin as GPIO)
void sendDMX(int highestChannel) {
  if (highestChannel < 1) return;

  // enable driver (RS485 DE)
  digitalWrite(RS485_DE_PIN, HIGH);

  // stop UART so we can drive TX pin low for BREAK
  Serial1.end();

  // drive TX pin low for BREAK duration (>88us)
  pinMode(RS485_TX_PIN, OUTPUT);
  digitalWrite(RS485_TX_PIN, LOW);
  delayMicroseconds(120);

  // Mark After Break (MAB)
  digitalWrite(RS485_TX_PIN, HIGH);
  delayMicroseconds(12);

  // re-init UART and send DMX (start code 0 + channels 1..N)
  Serial1.begin(250000, SERIAL_8N2, RS485_RX_PIN, RS485_TX_PIN);
  Serial1.write((uint8_t)0);                       // start code
  Serial1.write(dmxData + 1, highestChannel);     // channel 1..N
  Serial1.flush();

  // disable driver
  digitalWrite(RS485_DE_PIN, LOW);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  // SPI starten
  SPI.begin(18, 19, 23); // SCK, MISO, MOSI

  // CS-Pins konfigurieren
  for (int i = 0; i < NUM_BOARDS; i++) {
    pinMode(csPins[i], OUTPUT);
    digitalWrite(csPins[i], HIGH);
  }

  // I2C starten
  Wire.begin(21, 22);

  // MCP23017 initialisieren
  for (int i = 0; i < NUM_BOARDS; i++) {
    // Adafruit_MCP23X17 verwendet begin_I2C(addr)
    mcp[i].begin_I2C(mcpAddr[i]);
    for (int p = 0; p < 16; p++) {
      mcp[i].pinMode(p, INPUT);
    }
    // Enable internal pull-ups on MCP23017 by writing GPPUA/GPPUB
    // GPPUA register = 0x0C, GPPUB = 0x0D (set bits=1 to enable pull-up)
    Wire.beginTransmission(mcpAddr[i]); // device address (0x20..0x27)
    Wire.write(0x0C);      // GPPUA
    Wire.write(0xFF);      // enable pull-ups on GPA0..GPA7
    Wire.endTransmission();

    Wire.beginTransmission(mcpAddr[i]);
    Wire.write(0x0D);      // GPPUB
    Wire.write(0xFF);      // enable pull-ups on GPB0..GPB7
    Wire.endTransmission();
  }

  // DMX starten
  // Configure Serial1 for RS485/DMX TTL output
  Serial1.begin(250000, SERIAL_8N2, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW); // default Empfang (Driver disabled)

  // LEDs initialisieren
  strip.begin();
  strip.show(); // alle aus / initial

  // Initialwerte setzen
  for (int b = 0; b < NUM_BOARDS; b++) {
    for (int f = 0; f < 8; f++) smoothVal[b][f] = 0.0f;
    for (int p = 0; p < 16; p++) {
      lastRawButton[b][p] = false;
      debouncedButton[b][p] = false;
      lastBounceTime[b][p] = 0;
      toggleMode[b][p] = false; // Default: momentary; bei Bedarf anpassen
      toggledState[b][p] = false;
    }
  }
}


// ---------------- LOOP ----------------
void loop() {

  for (int b = 0; b < NUM_BOARDS; b++) {

    int base = dmxBase(b);

    // --- FADER 1–8 (DMX 1–8) mit Glättung und LED ---
    for (int f = 0; f < 8; f++) {
      int raw = readMCP3008(csPins[b], f); // 0..1023

      // Exponentielle Glättung
      smoothVal[b][f] = (SMOOTH_ALPHA * raw) + ((1.0f - SMOOTH_ALPHA) * smoothVal[b][f]);
      int val = map((int)smoothVal[b][f], 0, 1023, 0, 255);

      // replace dmx.write(...) for faders:
      // dmx.write(base + 1 + f, val);
      dmxData[base + 1 + f] = val;

      // LED: für Board b, Fader f -> index baseLED + 16 + f*2 (oben), +1 (unten)
      int ledBase = b * LEDS_PER_BOARD;
      int topIdx = ledBase + 16 + f * 2;
      int botIdx = ledBase + 16 + f * 2 + 1;

      // Intensitäten (0..255)
      uint8_t topIntensity = (uint8_t)val;
      uint8_t botIntensity = (uint8_t)(255 - val);

      // Wenn "aktiv" (über Schwelle) grün, sonst blau (Skalierung)
      if (val > ACTIVE_THRESHOLD) {
        strip.setPixelColor(topIdx, colorGreen(topIntensity));
        strip.setPixelColor(botIdx, colorGreen(botIntensity));
      } else {
        strip.setPixelColor(topIdx, colorBlue(topIntensity));
        strip.setPixelColor(botIdx, colorBlue(botIntensity));
      }
    }

    // --- BUTTONS 1–16 (DMX 9–24) mit Debounce, Toggle und LED ---
    for (int p = 0; p < 16; p++) {
      bool rawPressed = (mcp[b].digitalRead(p) == LOW); // true = gedrückt
      unsigned long now = millis();

      // Debounce-Logik
      if (rawPressed != lastRawButton[b][p]) {
        lastBounceTime[b][p] = now;
        lastRawButton[b][p] = rawPressed;
      }
      if ((now - lastBounceTime[b][p]) > DEBOUNCE_MS) {
        if (rawPressed != debouncedButton[b][p]) {
          debouncedButton[b][p] = rawPressed;

          // Zustand auf Flanke behandeln: nur beim Press-Event toggeln
          if (debouncedButton[b][p]) {
            if (toggleMode[b][p]) {
              toggledState[b][p] = !toggledState[b][p];
            }
            // bei momentary: nothing to toggle, we use debouncedButton as active state
          }
        }
      }

      // Ermittelter "active"-Zustand
      bool active = toggleMode[b][p] ? toggledState[b][p] : debouncedButton[b][p];
      // replace dmx.write(...) for buttons:
      // dmx.write(base + 9 + p, active ? 255 : 0);
      dmxData[base + 9 + p] = active ? 255 : 0;

      // LED: Button LED = ledBase + p
      int ledIdx = b * LEDS_PER_BOARD + p;
      if (active) {
        strip.setPixelColor(ledIdx, colorGreen(200)); // grün wenn aktiv
      } else {
        strip.setPixelColor(ledIdx, colorBlue(80)); // Standard blau
      }
    }

    //// --- FADER 1–8 (zweite Gruppe, DMX 25–32) ---
    //for (int f = 0; f < 8; f++) {
    //  int raw = readMCP3008(csPins[b], f);
    //  int val = map(raw, 0, 1023, 0, 255);
    //  dmx.write(base + 25 + f, val);
    //}
  }
  sendDMX(NUM_BOARDS * 32);

  // LED ausgeben (bereits vorher gesetzt)
  strip.show();
}
