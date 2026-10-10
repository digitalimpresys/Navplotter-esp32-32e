// Hardware probe: display controller, touch wiring, SD card, GPS, backlight pin.
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>

TFT_eSPI tft;
SPIClass sdSpi(VSPI);

// Bit-banged XPT2046 read on arbitrary pins: returns a 12-bit conversion for a command byte
uint16_t xptRead(int clk, int mosi, int miso, int cs, uint8_t cmd) {
  pinMode(clk, OUTPUT); pinMode(mosi, OUTPUT); pinMode(miso, INPUT); pinMode(cs, OUTPUT);
  digitalWrite(clk, LOW); digitalWrite(cs, LOW);
  for(int i = 7; i >= 0; i--) { digitalWrite(mosi, (cmd >> i) & 1); digitalWrite(clk, HIGH); delayMicroseconds(5); digitalWrite(clk, LOW); delayMicroseconds(5); }
  uint16_t v = 0;
  for(int i = 0; i < 16; i++) { digitalWrite(clk, HIGH); delayMicroseconds(5); v = (v << 1) | digitalRead(miso); digitalWrite(clk, LOW); delayMicroseconds(5); }
  digitalWrite(cs, HIGH);
  return v >> 4;
}

void probeTouch(const char* name, int clk, int mosi, int miso, int cs) {
  // Channels: X (0xD0), Y (0x90), Z1 (0xB0), temperature (0x86), battery input (0xA6)
  Serial.printf("TOUCH %s clk=%d mosi=%d miso=%d cs=%d: X=%u Y=%u Z1=%u TEMP=%u VBAT=%u\n", name, clk, mosi, miso, cs,
    xptRead(clk, mosi, miso, cs, 0xD0), xptRead(clk, mosi, miso, cs, 0x90), xptRead(clk, mosi, miso, cs, 0xB0),
    xptRead(clk, mosi, miso, cs, 0x86), xptRead(clk, mosi, miso, cs, 0xA6));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== PROBE START ===");
  Serial.printf("PSRAM: %u bytes, heap free %u, largest block %u\n", ESP.getPsramSize(), ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  // Touch first, before the display library takes its SPI pins
  probeTouch("CYD-style", 25, 32, 39, 33);
  pinMode(36, INPUT);
  Serial.printf("TOUCH IRQ (IO36) level: %d\n", digitalRead(36));

  for(int bl : {21, 27}) { pinMode(bl, OUTPUT); digitalWrite(bl, HIGH); }
  tft.init();
  Serial.printf("LCD ID 0x04: %08X\n", (unsigned)tft.readcommand32(0x04));
  Serial.printf("LCD ID 0xD3: %08X\n", (unsigned)tft.readcommand32(0xD3));
  Serial.printf("LCD status 0x09: %08X\n", (unsigned)tft.readcommand32(0x09));
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 80, 60, TFT_RED);
  tft.fillRect(80, 0, 80, 60, TFT_GREEN);
  tft.fillRect(160, 0, 80, 60, TFT_BLUE);
  tft.fillRect(240, 0, 80, 60, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("ROSSO   VERDE   BLU   BIANCO", 10, 70, 2);
  tft.drawString("NavPlotter ESP32-32E", 10, 100, 4);
  tft.drawString("Angolo in alto a sinistra: A", 10, 140, 2);
  tft.drawString("A", 2, 2, 4);
  tft.drawString("Tocca lo schermo: vedi i valori", 10, 170, 2);
  tft.drawRect(0, 0, 320, 240, TFT_YELLOW);

  sdSpi.begin(18, 19, 23, 5);
  bool sd = SD.begin(5, sdSpi, 20000000U);
  Serial.printf("SD: %s", sd ? "ok" : "NOT found");
  if(sd) {
    Serial.printf(", %llu MB, /tiles %s", SD.cardSize() / (1024 * 1024), SD.exists("/tiles") ? "present" : "missing");
  }
  Serial.println();

  // GPS on IO35 (and the old wiring on IO27), common speeds
  for(int pin : {35, 27}) {
    for(uint32_t baud : {9600u, 38400u, 115200u}) {
      Serial2.end();
      Serial2.begin(baud, SERIAL_8N1, pin, -1);
      unsigned long t = millis(); int dollars = 0, total = 0, printable = 0;
      while(millis() - t < 1300) while(Serial2.available()) { char c = Serial2.read(); total++; if(c == '$') dollars++; if((c >= ' ' && c <= '~') || c == '\r' || c == '\n') printable++; }
      Serial.printf("GPS pin %d @ %u: %d bytes, %d '$', %d printable\n", pin, baud, total, dollars, printable);
    }
  }
  Serial.printf("Battery ADC IO34: %u mV\n", analogReadMilliVolts(34));
  Serial.println("=== PROBE END (touch readings follow) ===");
}

void loop() {
  static unsigned long last = 0;
  if(millis() - last < 300) return;
  last = millis();
  uint16_t z = xptRead(25, 32, 39, 33, 0xB0);
  if(z > 100 && z < 4000) {
    char buf[64];
    uint16_t x = xptRead(25, 32, 39, 33, 0xD0), y = xptRead(25, 32, 39, 33, 0x90);
    snprintf(buf, sizeof(buf), "touch x=%4u y=%4u z=%4u   ", x, y, z);
    Serial.printf("TOUCHED %s irq=%d\n", buf, digitalRead(36));
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(buf, 10, 200, 2);
  }
}
