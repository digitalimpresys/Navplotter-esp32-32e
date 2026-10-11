// Radio probe: is the SX1276 answering on the shared SPI bus, and does the SD card still work beside it?
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <driver/gpio.h>

SPIClass bus(VSPI);
const int LORA_CS = 27, SD_CS = 5;

uint8_t loraReg(uint8_t reg) {
  bus.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_CS, LOW);
  bus.transfer(reg & 0x7F);
  uint8_t v = bus.transfer(0);
  digitalWrite(LORA_CS, HIGH);
  bus.endTransaction();
  return v;
}

void loraWrite(uint8_t reg, uint8_t v) {
  bus.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_CS, LOW);
  bus.transfer(reg | 0x80);
  bus.transfer(v);
  digitalWrite(LORA_CS, HIGH);
  bus.endTransaction();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== RADIO PROBE ===");
  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
  bus.begin(18, 19, 23, -1);
  for(int i = 0; i < 3; i++) {
    Serial.printf("SX127x version register (expect 0x12): 0x%02X, opmode 0x%02X, freq %02X %02X %02X\n",
      loraReg(0x42), loraReg(0x01), loraReg(0x06), loraReg(0x07), loraReg(0x08));
    delay(100);
  }
  // Is anything driving MISO while the radio is selected? A floating line follows the internal pull resistor.
  gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY);
  delay(5);
  uint8_t up = loraReg(0x42);
  gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLDOWN_ONLY);
  delay(5);
  uint8_t down = loraReg(0x42);
  gpio_set_pull_mode(GPIO_NUM_19, GPIO_FLOATING);
  Serial.printf("MISO with pull-up: 0x%02X, with pull-down: 0x%02X -> %s\n", up, down,
    (up == 0xFF && down == 0x00) ? "FLOATING: nothing drives MISO" : "driven by something");
  // Write/read-back test on the sync word register
  uint8_t before = loraReg(0x39);
  loraWrite(0x39, 0x4E);
  Serial.printf("Sync word register: was 0x%02X, wrote 0x4E, reads 0x%02X\n", before, loraReg(0x39));
  loraWrite(0x39, before);

  bool sd = SD.begin(SD_CS, bus, 20000000U);
  Serial.printf("SD with the radio on the bus: %s\n", sd ? "ok" : "NOT working");
  if(sd) {
    File dir = SD.open("/tiles");
    int n = 0;
    for(File f = dir.openNextFile(); f; f = dir.openNextFile()) { n++; f.close(); }
    Serial.printf("  /tiles has %d entries\n", n);
    // Read some real data and checksum it twice: a disturbed MISO line gives different sums
    File t = SD.open("/waypoints.txt");
    for(int pass = 0; pass < 2; pass++) {
      File d = SD.open("/tiles/12");
      File x = d ? d.openNextFile() : File();
      File y = x ? x.openNextFile() : File();
      uint32_t sum = 0; size_t len = 0;
      while(y && y.available()) { sum = sum * 31 + y.read(); len++; }
      Serial.printf("  tile read pass %d: %u bytes, checksum %08X, starts with JPEG marker: %s\n", pass + 1, (unsigned)len, (unsigned)sum, "see checksum");
      if(y) y.close(); if(x) x.close(); if(d) d.close();
    }
  }
  Serial.printf("Radio after SD use: version 0x%02X\n", loraReg(0x42));
  Serial.println("=== END ===");
}

void loop() {}
