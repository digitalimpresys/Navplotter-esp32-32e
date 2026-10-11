// NavPlotter for the 2.8" ESP32-32E display board
//
// Same idea as the T-Deck plotter (chart tiles from the SD card, GPS, waypoints, compass), on a board with
// a resistive touch screen, no keys and no extra memory. Two things follow from that:
//  - the screen is drawn off-screen in three strips instead of one big buffer, then pushed: no flicker;
//  - everything is done by touch, including typing names on an on-screen keyboard.
// User interface texts are in English with an Italian translation, chosen in the Setup page.
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <TinyGPS++.h>
#include <RadioLib.h>
#include <vector>
#include <math.h>
#include "board.h"
#include "traffic_packet.h"

#define FW_VERSION "0.2.0"

// ---- Screen layout (landscape, USB connector on the right) ----
#define SCREEN_W 320
#define SCREEN_H 240
#define SIDEBAR_W 48
#define STATUS_H 20
#define MAP_W (SCREEN_W - SIDEBAR_W)
#define MAP_H (SCREEN_H - STATUS_H)
#define SIDE_BTN_COUNT 7
#define SIDE_BTN_H (SCREEN_H / SIDE_BTN_COUNT)

// ---- Colours ----
#define C_SEA 0x9EDD        // same sea colour as the chart tiles, which leave open sea out
#define C_BLACK TFT_BLACK
#define C_WHITE TFT_WHITE
#define C_PANEL 0x10A4      // dark blue-grey
#define C_BTN 0xC618        // light grey
#define C_ACCENT TFT_NAVY
#define C_RED TFT_RED
#define C_DIM 0x8410
#define C_GREEN 0x2E8B

// ---- Charts: /tiles/z/x/y.jpg on the SD card (XYZ scheme, Web Mercator, 256x256, baseline JPEG) ----
#define TILE_SIZE 256
#define ZOOM_MIN 3
#define ZOOM_MAX 18
#define ZOOM_DEFAULT 12

// ---- Touch ----
#define TOUCH_PRESSURE_MIN 350
#define TOUCH_POLL_MS 15
#define LONG_PRESS_MS 700
#define DRAG_MIN_PX 18

#define MOVING_MIN_KNOTS 0.5
#define ARRIVAL_NM 0.05
#define NAME_LEN 20

// ---- LoRa traffic (same packet, radio settings and mesh as the T-Deck plotter and the Heltec node) ----
#define LORA_POWER_DBM 17
#define LORA_TX_MOVING_MS 10000UL
#define LORA_TX_STOPPED_MS 30000UL
#define TARGET_STALE_MS 60000UL
#define TARGET_EXPIRE_MS 180000UL
#define MAX_TARGETS 20
// Collision alarm: closest point of approach below CPA_ALARM_NM within CPA_ALARM_S seconds
#define CPA_ALARM_NM 0.1
#define CPA_ALARM_S 300
#define ALARM_REPEAT_MS 3000UL
#define ALARM_MUTE_MS 60000UL   // a tap on the red band silences the alarm for a minute

// Texts: English first, Italian second
bool langIt = false;
#define TR(en, it) (langIt ? (it) : (en))

// ============================================
// Canvas: the whole screen drawn off-screen in three strips
// ============================================
// A full 320x240 16-bit buffer is 150 KB in one piece, more than this board can allocate.
// Three 320x80 strips fit, and each drawing call is simply sent to all of them.
#define STRIPS 3
#define STRIP_H (SCREEN_H / STRIPS)

TFT_eSPI tft;

class Canvas {
public:
  TFT_eSprite* s[STRIPS];

  bool begin() {
    for(int i = 0; i < STRIPS; i++) {
      s[i] = new TFT_eSprite(&tft);
      s[i]->setColorDepth(16);
      if(s[i]->createSprite(SCREEN_W, STRIP_H) == nullptr) return false;
      s[i]->setTextWrap(false);
    }
    return true;
  }
  void push() { for(int i = 0; i < STRIPS; i++) s[i]->pushSprite(0, i * STRIP_H); }
  // Sends only the strips that hold rows y..y+h-1 (used for the status bar, redrawn every second)
  void pushRows(int y, int h) {
    for(int i = 0; i < STRIPS; i++) {
      if(y < (i + 1) * STRIP_H && y + h > i * STRIP_H) s[i]->pushSprite(0, i * STRIP_H);
    }
  }
#define EACH for(int i = 0, o = 0; i < STRIPS; i++, o += STRIP_H)
  void fill(uint16_t c) { EACH s[i]->fillSprite(c); }
  void fillRect(int x, int y, int w, int h, uint16_t c) { EACH s[i]->fillRect(x, y - o, w, h, c); }
  void drawRect(int x, int y, int w, int h, uint16_t c) { EACH s[i]->drawRect(x, y - o, w, h, c); }
  void fillRoundRect(int x, int y, int w, int h, int r, uint16_t c) { EACH s[i]->fillRoundRect(x, y - o, w, h, r, c); }
  void drawRoundRect(int x, int y, int w, int h, int r, uint16_t c) { EACH s[i]->drawRoundRect(x, y - o, w, h, r, c); }
  void drawLine(int x0, int y0, int x1, int y1, uint16_t c) { EACH s[i]->drawLine(x0, y0 - o, x1, y1 - o, c); }
  void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c) { EACH s[i]->fillTriangle(x0, y0 - o, x1, y1 - o, x2, y2 - o, c); }
  void drawTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c) { EACH s[i]->drawTriangle(x0, y0 - o, x1, y1 - o, x2, y2 - o, c); }
  void fillCircle(int x, int y, int r, uint16_t c) { EACH s[i]->fillCircle(x, y - o, r, c); }
  void drawCircle(int x, int y, int r, uint16_t c) { EACH s[i]->drawCircle(x, y - o, r, c); }
  void pushImage(int x, int y, int w, int h, uint16_t* data) { EACH s[i]->pushImage(x, y - o, w, h, data); }
  // Text. datum: TL_DATUM, TC_DATUM, TR_DATUM or MC_DATUM; bg == fg means "no background".
  // The built-in fonts have no degree sign: a '^' in the text is drawn as one (a small ring).
  void text(const String& t, int x, int y, uint8_t font, uint16_t fg, uint16_t bg, uint8_t datum = TL_DATUM) {
    int mark = t.indexOf('^');
    String shown = t;
    if(mark >= 0) shown.setCharAt(mark, ' ');
    EACH {
      s[i]->setTextDatum(datum);
      if(fg == bg) s[i]->setTextColor(fg); else s[i]->setTextColor(fg, bg);
      s[i]->drawString(shown, x, y - o, font);
    }
    if(mark >= 0) {
      int w = s[0]->textWidth(shown, font), h = s[0]->fontHeight(font);
      int left = datum == TL_DATUM ? x : datum == TR_DATUM ? x - w : x - w / 2;
      int top = datum == MC_DATUM ? y - h / 2 : y;
      int r = font >= 4 ? 3 : 2;
      int cx = left + s[0]->textWidth(shown.substring(0, mark), font) + s[0]->textWidth(" ", font) / 2, cy = top + r + 2;
      EACH { s[i]->drawCircle(cx, cy - o, r, fg); if(font >= 4) s[i]->drawCircle(cx, cy - o, r - 1, fg); }
    }
  }
  int textWidth(const String& t, uint8_t font) { return s[0]->textWidth(t, font); }
#undef EACH
};

Canvas cv;

// ============================================
// State
// ============================================
struct Waypoint {
  int id = 0;
  char name[NAME_LEN] = "";
  double lat = 0, lon = 0;
  bool visible = true;
};

enum Page {PAGE_MAP, PAGE_WAYPOINTS, PAGE_COMPASS, PAGE_NEARBY, PAGE_SETUP, PAGE_KEYBOARD};
Page page = PAGE_MAP;

Preferences prefs;
TinyGPSPlus gps;
SPIClass sdSpi(VSPI);

std::vector<Waypoint> waypoints;
int selectedWp = -1;
int destId = -1;              // "go to" destination, -1 = none
int listTop = 0;

double mapLat = 45.0, mapLon = 12.0;
int zoom = ZOOM_DEFAULT, zoomMin = ZOOM_MIN, zoomMax = ZOOM_MAX;
bool follow = true;           // the map keeps the boat in the middle until it is dragged
bool mapPlaced = false;

bool sdOk = false, tilesOk = false;
uint32_t gpsBaud = 0;
uint32_t gpsBytes = 0;        // characters received from the GPS pin, shown in Setup as a wiring check
float lastCourse = 0;
bool courseKnown = false;
char boatName[BOAT_NAME_LEN] = "";
bool soundOn = true;

// Other nodes received over LoRa
struct Target {
  uint32_t id = 0;
  char name[BOAT_NAME_LEN] = "";
  uint8_t type = NODE_TYPE_BOAT;
  double lat = 0, lon = 0;
  float cog = 0, sog = 0;       // degrees, knots
  unsigned long lastSeen = 0;
  float rssi = 0;
  bool relayed = false;
  bool cpaValid = false, alarm = false, muted = false;
  unsigned long mutedAt = 0;
  double cpaNm = 0, tcpaS = 0;
};
std::vector<Target> targets;
bool radioOk = false, radioTx = false, radioTxIsRelay = false;
bool sharePosition = true;      // transmit our own position; when off the node only listens and relays
unsigned long radioTxStart = 0, radioNextTx = 0, alarmNextBurst = 0;
uint32_t txCount = 0, rxCount = 0, relayCount = 0, myId = 0;
uint8_t txSeq = 0;
bool alarmActive = false, alarmSilenced = false;

bool needsRedraw = true;
unsigned long lastFullDraw = 0, lastStatusDraw = 0;
int lastBoatX = -1000, lastBoatY = -1000;
String toast;
unsigned long toastUntil = 0;

// Touch calibration: screen = a * rawX + b * rawY + c, for x and for y
float cal[6] = {0, 0.0865f, -16, 0.0667f, 0, -13};
bool calOk = false;

// On-screen keyboard
String kbText, kbTitle;
int kbMax = NAME_LEN - 1;
bool kbShift = false;
Page kbReturn = PAGE_MAP;
void (*kbDone)(const String&) = nullptr;

// Sound
int beepsLeft = 0;
unsigned long beepNext = 0;
bool beepOn = false;

void requestRedraw() { needsRedraw = true; }
void showToast(const String& t) { toast = t; toastUntil = millis() + 2500; requestRedraw(); Serial.println(t); }
void goTo(Page p) { page = p; requestRedraw(); }

// ============================================
// Helpers
// ============================================
bool gpsFix() { return gps.location.isValid() && gps.location.age() < 8000; }
bool moving() { return gps.speed.isValid() && gps.speed.knots() >= MOVING_MIN_KNOTS; }

// Heading = GPS course; when stopped the GPS course is noise, so the last good one is kept
float heading() {
  if(gps.course.isValid() && moving()) { lastCourse = gps.course.deg(); courseKnown = true; }
  return lastCourse;
}

Waypoint* findWp(int id) {
  for(auto& w : waypoints) if(w.id == id) return &w;
  return nullptr;
}

int nextWpId() {
  int m = 0;
  for(const auto& w : waypoints) if(w.id > m) m = w.id;
  return m + 1;
}

String fmtNm(double nm) { return String(nm, nm < 10 ? 2 : nm < 100 ? 1 : 0); }
String deg3(double d) { char b[8]; snprintf(b, sizeof(b), "%03d", ((int)lround(d)) % 360); return b; }

float batteryVolts() { return analogReadMilliVolts(PIN_BATTERY) * 2 / 1000.0; }

// ---- Web Mercator, the projection of the tiles ----
double worldSize() { return TILE_SIZE * (double)(1UL << zoom); }
double lonToWX(double lon) { return (lon + 180.0) / 360.0 * worldSize(); }
double latToWY(double lat) {
  lat = constrain(lat, -85.0511, 85.0511);
  double r = lat * DEG_TO_RAD;
  return (1.0 - log(tan(r) + 1.0 / cos(r)) / PI) / 2.0 * worldSize();
}
int lonToPx(double lon) { return MAP_W / 2 + (int)lround(lonToWX(lon) - (double)llround(lonToWX(mapLon))); }
int latToPx(double lat) { return MAP_H / 2 + (int)lround(latToWY(lat) - (double)llround(latToWY(mapLat))); }
double pxToLon(int x) { return ((double)llround(lonToWX(mapLon)) + (x - MAP_W / 2)) / worldSize() * 360.0 - 180.0; }
double pxToLat(int y) {
  double wy = (double)llround(latToWY(mapLat)) + (y - MAP_H / 2);
  return atan(sinh(PI * (1.0 - 2.0 * wy / worldSize()))) * RAD_TO_DEG;
}
double metersPerPixel() { return 156543.03392 * cos(mapLat * DEG_TO_RAD) / (double)(1UL << zoom); }

// ============================================
// Settings (internal flash) and waypoints (SD card, same file format as the T-Deck plotter)
// ============================================
void saveSettings() {
  prefs.begin("navplotter", false);
  prefs.putBool("it", langIt);
  prefs.putString("name", boatName);
  prefs.putBool("sound", soundOn);
  prefs.putBool("share", sharePosition);
  prefs.putInt("zoom", zoom);
  prefs.putInt("dest", destId);
  prefs.putBool("calok2", calOk);
  prefs.putBytes("cal2", cal, sizeof(cal));
  prefs.end();
}

void loadSettings() {
  prefs.begin("navplotter", true);
  langIt = prefs.getBool("it", false);
  prefs.getString("name", boatName, sizeof(boatName));
  soundOn = prefs.getBool("sound", true);
  sharePosition = prefs.getBool("share", true);
  zoom = prefs.getInt("zoom", ZOOM_DEFAULT);
  destId = prefs.getInt("dest", -1);
  calOk = prefs.getBool("calok2", false);
  if(calOk) prefs.getBytes("cal2", cal, sizeof(cal));
  prefs.end();
  if(boatName[0] == '\0') strcpy(boatName, "BOAT");
}

void saveWaypoints() {
  if(!sdOk) return;
  File f = SD.open("/waypoints.txt", FILE_WRITE);
  if(!f) return;
  for(const auto& w : waypoints) f.printf("%d,%s,%.8f,%.8f,%d\n", w.id, w.name, w.lat, w.lon, w.visible);
  f.close();
}

void loadWaypoints() {
  if(!sdOk || !SD.exists("/waypoints.txt")) return;
  File f = SD.open("/waypoints.txt");
  if(!f) return;
  waypoints.clear();
  while(f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1), c3 = line.indexOf(',', c2 + 1), c4 = line.indexOf(',', c3 + 1);
    if(c1 <= 0 || c2 <= 0 || c3 <= 0 || c4 <= 0) continue;
    Waypoint w;
    w.id = line.substring(0, c1).toInt();
    line.substring(c1 + 1, c2).toCharArray(w.name, NAME_LEN);
    w.lat = line.substring(c2 + 1, c3).toDouble();
    w.lon = line.substring(c3 + 1, c4).toDouble();
    w.visible = line.substring(c4 + 1).toInt();
    waypoints.push_back(w);
  }
  f.close();
  Serial.printf("%d waypoints loaded\n", (int)waypoints.size());
}

// ============================================
// Sound: short beeps on the speaker output
// ============================================
#define BEEP_CHANNEL 0

void initSound() {
  pinMode(PIN_AUDIO_ENABLE, OUTPUT);
  digitalWrite(PIN_AUDIO_ENABLE, HIGH); // amplifier off until needed
  ledcSetup(BEEP_CHANNEL, 2700, 8);
  ledcAttachPin(PIN_AUDIO, BEEP_CHANNEL);
  ledcWrite(BEEP_CHANNEL, 0);
}

void beep(int count) {
  if(!soundOn) return;
  beepsLeft = count;
  beepNext = millis();
  beepOn = false;
}

void handleSound() {
  if(beepsLeft == 0 && !beepOn) return;
  if((long)(millis() - beepNext) < 0) return;
  if(!beepOn && beepsLeft > 0) {
    digitalWrite(PIN_AUDIO_ENABLE, LOW);
    ledcWriteTone(BEEP_CHANNEL, 2700);
    beepOn = true;
    beepNext = millis() + 130;
  } else {
    ledcWriteTone(BEEP_CHANNEL, 0);
    beepOn = false;
    beepsLeft--;
    beepNext = millis() + 110;
    if(beepsLeft <= 0) { beepsLeft = 0; digitalWrite(PIN_AUDIO_ENABLE, HIGH); }
  }
}

// ============================================
// GPS
// ============================================
void detectGpsBaud() {
  const uint32_t bauds[] = {9600, 38400, 115200, 4800, 57600};
  for(uint32_t baud : bauds) {
    Serial2.end();
    // Drawing the map can take a third of a second: without a larger buffer the GPS sentences that
    // arrive meanwhile are lost, and the fix seems to come and go
    Serial2.setRxBufferSize(2048);
    Serial2.begin(baud, SERIAL_8N1, PIN_GPS_RX, -1);
    unsigned long start = millis();
    int dollars = 0, printable = 0, total = 0;
    while(millis() - start < 1200) {
      while(Serial2.available()) {
        char c = Serial2.read();
        total++;
        gpsBytes++;
        if(c == '$') dollars++;
        if((c >= ' ' && c <= '~') || c == '\r' || c == '\n') printable++;
      }
    }
    if(dollars >= 2 && printable > total * 9 / 10) {
      gpsBaud = baud;
      Serial.printf("GPS found at %lu baud\n", (unsigned long)baud);
      return;
    }
  }
  Serial.println("GPS not found on IO35");
}

void readGps() {
  static unsigned long lastRetry = 0;
  while(Serial2.available()) { gps.encode(Serial2.read()); gpsBytes++; }
  // No module found at boot: look again every now and then
  if(!gpsBaud && millis() - lastRetry > 30000) {
    lastRetry = millis();
    detectGpsBaud();
  }
  if(gpsFix()) {
    if(!mapPlaced) { mapPlaced = true; mapLat = gps.location.lat(); mapLon = gps.location.lng(); requestRedraw(); }
    if(follow) {
      // Follow mode: recentre when the boat has drifted a few pixels, not at every tiny GPS wobble
      int bx = lonToPx(gps.location.lng()), by = latToPx(gps.location.lat());
      if(abs(bx - MAP_W / 2) > 3 || abs(by - MAP_H / 2) > 3) {
        mapLat = gps.location.lat();
        mapLon = gps.location.lng();
        if(page == PAGE_MAP) requestRedraw();
      }
    }
  }
}

// ============================================
// Touch (XPT2046 read bit by bit on its own pins)
// ============================================
uint16_t touchRead(uint8_t cmd) {
  digitalWrite(PIN_TOUCH_CLK, LOW);
  digitalWrite(PIN_TOUCH_CS, LOW);
  for(int i = 7; i >= 0; i--) {
    digitalWrite(PIN_TOUCH_MOSI, (cmd >> i) & 1);
    digitalWrite(PIN_TOUCH_CLK, HIGH);
    digitalWrite(PIN_TOUCH_CLK, LOW);
  }
  uint16_t v = 0;
  for(int i = 0; i < 16; i++) {
    digitalWrite(PIN_TOUCH_CLK, HIGH);
    v = (v << 1) | digitalRead(PIN_TOUCH_MISO);
    digitalWrite(PIN_TOUCH_CLK, LOW);
  }
  digitalWrite(PIN_TOUCH_CS, HIGH);
  return (v >> 3) & 0x0FFF; // 12-bit result, after one busy bit
}

void initTouch() {
  pinMode(PIN_TOUCH_CLK, OUTPUT);
  pinMode(PIN_TOUCH_MOSI, OUTPUT);
  pinMode(PIN_TOUCH_MISO, INPUT);
  pinMode(PIN_TOUCH_CS, OUTPUT);
  pinMode(PIN_TOUCH_IRQ, INPUT);
  digitalWrite(PIN_TOUCH_CS, HIGH);
}

// Is the screen being pressed? The single Z1 reading gets small towards one edge of the panel, so the
// usual combination of the two pressure readings is used: it works the same everywhere.
bool touchPressed() {
  int z = (int)touchRead(0xB0) + 4095 - (int)touchRead(0xC0);
  return z >= TOUCH_PRESSURE_MIN;
}

// Raw position, averaged; false when the screen is not pressed
bool touchRaw(int& rx, int& ry) {
  if(!touchPressed()) return false;
  long sx = 0, sy = 0;
  for(int i = 0; i < 4; i++) { sx += touchRead(0xD0); sy += touchRead(0x90); }
  if(!touchPressed()) return false; // lifted while reading: the values are not good
  rx = sx / 4;
  ry = sy / 4;
  return true;
}

bool touchPoint(int& x, int& y) {
  int rx, ry;
  if(!touchRaw(rx, ry)) return false;
  x = constrain((int)(cal[0] * rx + cal[1] * ry + cal[2]), 0, SCREEN_W - 1);
  y = constrain((int)(cal[3] * rx + cal[4] * ry + cal[5]), 0, SCREEN_H - 1);
  return true;
}

// Solves the 3x3 system m * r = v (Cramer); false when the points are useless
static bool solve3(double m[3][3], double v[3], float* r) {
  double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if(fabs(det) < 1e-6) return false;
  for(int c = 0; c < 3; c++) {
    double t[3][3];
    for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) t[i][j] = (j == c) ? v[i] : m[i][j];
    double d = t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1]) - t[0][1] * (t[1][0] * t[2][2] - t[1][2] * t[2][0]) + t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0]);
    r[c] = d / det;
  }
  return true;
}

// Waits for a steady press and returns its raw position (average of a short, still hold)
static void steadyPress(int& rx, int& ry) {
  while(true) {
    int x, y, minX = 9999, maxX = 0, minY = 9999, maxY = 0, n = 0;
    long sx = 0, sy = 0;
    unsigned long start = millis();
    while(touchRaw(x, y)) {
      sx += x; sy += y; n++;
      minX = min(minX, x); maxX = max(maxX, x); minY = min(minY, y); maxY = max(maxY, y);
      delay(15);
      // Half a second without moving: good
      if(millis() - start > 500 && n >= 20) {
        if(maxX - minX < 150 && maxY - minY < 150) { rx = sx / n; ry = sy / n; return; }
        break; // the finger slid: start again
      }
    }
    delay(15);
  }
}

static void waitRelease() {
  unsigned long clear = millis();
  while(millis() - clear < 400) {
    if(touchPressed()) clear = millis();
    delay(15);
  }
}

// Four crosses to press and hold. Whatever way round the touch panel is mounted, the fit sorts it out;
// if the four points do not agree with each other the procedure starts again.
void calibrateTouch() {
  const int px[4] = {24, SCREEN_W - 24, SCREEN_W - 24, 24};
  const int py[4] = {24, 24, SCREEN_H - 24, SCREEN_H - 24};
  bool retry = false;
  while(true) {
    int rx[4], ry[4];
    for(int k = 0; k < 4; k++) {
      tft.fillScreen(C_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(C_WHITE, C_BLACK);
      tft.drawString(TR("Touch calibration", "Calibrazione del touch"), SCREEN_W / 2, SCREEN_H / 2 - 30, 4);
      tft.drawString(TR("Press and HOLD the centre of the cross", "Premi e TIENI premuto il centro della croce"), SCREEN_W / 2, SCREEN_H / 2, 2);
      tft.drawString(TR("until it turns green", "finche' diventa verde"), SCREEN_W / 2, SCREEN_H / 2 + 18, 2);
      if(retry) {
        tft.setTextColor(C_RED, C_BLACK);
        tft.drawString(TR("Not precise enough: again", "Non abbastanza preciso: ripeti"), SCREEN_W / 2, SCREEN_H / 2 + 44, 2);
      }
      tft.drawLine(px[k] - 14, py[k], px[k] + 14, py[k], C_RED);
      tft.drawLine(px[k], py[k] - 14, px[k], py[k] + 14, C_RED);
      tft.drawCircle(px[k], py[k], 8, C_RED);
      waitRelease();
      steadyPress(rx[k], ry[k]);
      tft.fillCircle(px[k], py[k], 10, TFT_GREEN);
      Serial.printf("Calibration point %d: raw %d, %d\n", k + 1, rx[k], ry[k]);
      waitRelease();
    }

    // Least squares over the four points
    double m[3][3] = {{0}}, vx[3] = {0}, vy[3] = {0};
    for(int k = 0; k < 4; k++) {
      double r[3] = {(double)rx[k], (double)ry[k], 1.0};
      for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) m[i][j] += r[i] * r[j];
        vx[i] += r[i] * px[k];
        vy[i] += r[i] * py[k];
      }
    }
    float nc[6];
    bool ok = solve3(m, vx, nc) && solve3(m, vy, nc + 3);
    // Each cross must land within a few pixels of where it was drawn
    for(int k = 0; ok && k < 4; k++) {
      float ex = nc[0] * rx[k] + nc[1] * ry[k] + nc[2] - px[k], ey = nc[3] * rx[k] + nc[4] * ry[k] + nc[5] - py[k];
      if(fabs(ex) > 12 || fabs(ey) > 12) ok = false;
    }
    if(ok) {
      memcpy(cal, nc, sizeof(cal));
      calOk = true;
      saveSettings();
      Serial.printf("Calibration: %.5f %.5f %.1f / %.5f %.5f %.1f\n", cal[0], cal[1], cal[2], cal[3], cal[4], cal[5]);
      break;
    }
    Serial.println("Calibration rejected, repeating");
    retry = true;
  }
  requestRedraw();
}

// Touch events: tap, long press (map only) and drag (map only), decided when the finger moves or lifts
void onTap(int x, int y);
void onLongPress(int x, int y);
void onDrag(int dx, int dy);

void handleTouch() {
  static unsigned long lastPoll = 0, downAt = 0;
  static bool down = false, longFired = false;
  static int startX = 0, startY = 0, lastX = 0, lastY = 0, misses = 0, hits = 0;
  if(millis() - lastPoll < TOUCH_POLL_MS) return;
  lastPoll = millis();

  int x, y;
  if(touchPoint(x, y)) {
    misses = 0;
    if(!down) {
      // A resistive panel gives a wild first reading as the finger lands: wait for a second one
      if(++hits < 2) return;
      down = true; longFired = false; downAt = millis(); startX = x; startY = y;
    }
    lastX = x; lastY = y;
    bool still = abs(x - startX) < DRAG_MIN_PX && abs(y - startY) < DRAG_MIN_PX;
    if(!longFired && still && millis() - downAt > LONG_PRESS_MS) {
      longFired = true;
      onLongPress(startX, startY);
    }
  } else {
    hits = 0;
    if(down && ++misses >= 3) {
      down = false;
      if(longFired) return;
      int dx = lastX - startX, dy = lastY - startY;
      if(abs(dx) >= DRAG_MIN_PX || abs(dy) >= DRAG_MIN_PX) onDrag(dx, dy);
      else onTap(startX, startY);
    }
  }
}

// ============================================
// LoRa traffic
// ============================================
// The radio (RFM95W, SX1276) shares the SPI bus with the SD card. Its interrupt pin is not wired, because
// the board has no pin to spare: the radio is asked every loop whether a packet arrived or went out.
SX1276 radio = new Module(PIN_LORA_CS, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC, sdSpi);
#define SX127X_IRQ_RX_DONE 0x40
#define SX127X_IRQ_TX_DONE 0x08

void initRadio() {
  uint64_t mac = ESP.getEfuseMac();
  myId = (uint32_t)(mac >> 16) ^ (uint32_t)mac;
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, LORA_POWER_DBM, LORA_PREAMBLE, 0);
  if(state == RADIOLIB_ERR_NONE) {
    radio.setCRC(true);
    radioOk = radio.startReceive() == RADIOLIB_ERR_NONE;
  }
  radioNextTx = millis() + 5000 + random(0, 3000);
  Serial.printf("LoRa %s (%d): %.3f MHz, SF%d, id %08lX\n", radioOk ? "ready" : "NOT available", state, LORA_FREQ, LORA_SF, (unsigned long)myId);
}

void sendPosition() {
  TrafficPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic[0] = 'N'; pkt.magic[1] = 'P';
  pkt.version = 1;
  pkt.ttl = MESH_TTL;
  pkt.id = myId;
  pkt.seq = txSeq++;
  pkt.flags = NODE_TYPE_BOAT << TRAFFIC_TYPE_SHIFT;
  pkt.lat = (int32_t)lround(gps.location.lat() * 1e7);
  pkt.lon = (int32_t)lround(gps.location.lng() * 1e7);
  pkt.cog = (uint16_t)lround(heading() * 10);
  pkt.sog = moving() ? (uint16_t)lround(gps.speed.knots() * 100) : 0;
  strncpy(pkt.name, boatName, BOAT_NAME_LEN - 1);
  if(radio.startTransmit((uint8_t*)&pkt, sizeof(pkt)) == RADIOLIB_ERR_NONE) {
    radioTx = true; radioTxIsRelay = false; radioTxStart = millis();
  } else {
    radio.startReceive();
  }
  // A little random delay keeps two nodes from always transmitting at the same instant
  radioNextTx = millis() + (moving() ? LORA_TX_MOVING_MS : LORA_TX_STOPPED_MS) + random(0, 2000);
}

void processPacket(const TrafficPacket& pkt, float rssi) {
  if(pkt.magic[0] != 'N' || pkt.magic[1] != 'P' || pkt.version != 1 || pkt.id == myId) return;
  // Mesh: drop copies already received (and if somebody else relayed it, we do not need to)
  if(meshAlreadySeen(pkt.id, pkt.seq)) { meshCancelRelay(pkt.id, pkt.seq); return; }
  rxCount++;
  meshQueueRelay(pkt);

  Target* t = nullptr;
  for(auto& e : targets) if(e.id == pkt.id) t = &e;
  if(!t) {
    if((int)targets.size() >= MAX_TARGETS) return;
    targets.push_back(Target());
    t = &targets.back();
    t->id = pkt.id;
  }
  strncpy(t->name, pkt.name, BOAT_NAME_LEN - 1);
  t->name[BOAT_NAME_LEN - 1] = '\0';
  for(char* c = t->name; *c; c++) if(*c < ' ' || *c > '~') *c = '?';
  t->type = (pkt.flags >> TRAFFIC_TYPE_SHIFT) & TRAFFIC_TYPE_MASK;
  if(t->type > NODE_TYPE_OBJECT) t->type = NODE_TYPE_BOAT;
  t->lat = pkt.lat / 1e7;
  t->lon = pkt.lon / 1e7;
  t->cog = pkt.cog / 10.0;
  t->sog = pkt.sog / 100.0;
  t->lastSeen = millis();
  t->rssi = rssi;
  t->relayed = pkt.ttl < MESH_TTL;
  Serial.printf("LoRa RX %s%s %.5f %.5f %.1f kn %.0f deg, RSSI %.0f\n", t->name, t->relayed ? " (relayed)" : "", t->lat, t->lon, t->sog, t->cog, rssi);
  if(page == PAGE_MAP || page == PAGE_NEARBY) requestRedraw();
}

void handleRadio() {
  if(!radioOk) return;
  static unsigned long lastPoll = 0;
  if(millis() - lastPoll < 5) return;
  lastPoll = millis();

  uint16_t irq = radio.getIRQFlags();
  if(radioTx) {
    if((irq & SX127X_IRQ_TX_DONE) || millis() - radioTxStart > 3000) {
      radio.finishTransmit();
      radioTx = false;
      if(irq & SX127X_IRQ_TX_DONE) { if(radioTxIsRelay) relayCount++; else txCount++; }
      radio.startReceive();
    }
    return;
  }
  if(irq & SX127X_IRQ_RX_DONE) {
    TrafficPacket pkt;
    size_t len = radio.getPacketLength();
    if(len == sizeof(pkt) && radio.readData((uint8_t*)&pkt, sizeof(pkt)) == RADIOLIB_ERR_NONE) processPacket(pkt, radio.getRSSI());
    radio.startReceive();
  }

  // Our own position goes out only if sharing is allowed; receiving and relaying do not depend on it
  if((long)(millis() - radioNextTx) >= 0) {
    if(sharePosition && gpsFix()) { sendPosition(); return; }
    radioNextTx = millis() + 2000;
  }
  // Mesh: relay packets from other nodes that still have ttl
  TrafficPacket relay;
  if(meshNextRelay(relay)) {
    if(radio.startTransmit((uint8_t*)&relay, sizeof(relay)) == RADIOLIB_ERR_NONE) {
      radioTx = true; radioTxIsRelay = true; radioTxStart = millis();
    } else {
      radio.startReceive();
    }
  }
}

// Closest point of approach (CPA) and time to it (TCPA), on a local flat plane in metres
void updateTarget(Target& t) {
  t.cpaValid = false;
  t.alarm = false;
  if(!gpsFix() || millis() - t.lastSeen > TARGET_STALE_MS) return;
  const double KN = 0.514444; // knots -> m/s
  double age = (millis() - t.lastSeen) / 1000.0, lat0 = gps.location.lat() * DEG_TO_RAD;
  double ownSog = moving() ? gps.speed.knots() * KN : 0, ownCog = heading() * DEG_TO_RAD;
  double tgtSog = t.sog * KN, tgtCog = t.cog * DEG_TO_RAD;
  double vx = tgtSog * sin(tgtCog) - ownSog * sin(ownCog), vy = tgtSog * cos(tgtCog) - ownSog * cos(ownCog);
  // Relative position, moved forward by the time since the last packet
  double rx = (t.lon - gps.location.lng()) * 111320.0 * cos(lat0) + tgtSog * sin(tgtCog) * age;
  double ry = (t.lat - gps.location.lat()) * 110540.0 + tgtSog * cos(tgtCog) * age;
  double v2 = vx * vx + vy * vy;
  if(v2 < 0.06) return; // relative speed under half a knot: nothing is closing in
  double tcpa = -(rx * vx + ry * vy) / v2;
  if(tcpa < 0) return;  // moving apart
  t.cpaValid = true;
  t.tcpaS = tcpa;
  t.cpaNm = hypot(rx + vx * tcpa, ry + vy * tcpa) / 1852.0;
  t.alarm = t.cpaNm <= CPA_ALARM_NM && t.tcpaS <= CPA_ALARM_S;
}

const Target* worstTarget() {
  const Target* worst = nullptr;
  for(const auto& t : targets) if(t.alarm && (!worst || t.tcpaS < worst->tcpaS)) worst = &t;
  return worst;
}

// Once a second: drop lost targets, recompute approaches, sound the alarm
void updateTraffic() {
  static unsigned long last = 0;
  if(millis() - last < 1000) return;
  last = millis();
  for(auto it = targets.begin(); it != targets.end(); ) {
    if(millis() - it->lastSeen > TARGET_EXPIRE_MS) { it = targets.erase(it); if(page == PAGE_MAP || page == PAGE_NEARBY) requestRedraw(); }
    else ++it;
  }
  bool any = false, allMuted = true;
  for(auto& t : targets) {
    updateTarget(t);
    // Silencing lasts while that node stays in alarm, and never more than a minute
    if(!t.alarm || millis() - t.mutedAt > ALARM_MUTE_MS) t.muted = false;
    if(t.alarm) { any = true; if(!t.muted) allMuted = false; }
  }
  if(any != alarmActive) { alarmNextBurst = millis(); requestRedraw(); }
  if(any && page != PAGE_MAP && !alarmActive) showToast(String(TR("COLLISION: ", "ABBORDO: ")) + worstTarget()->name);
  alarmActive = any;
  alarmSilenced = any && allMuted;
  if(alarmActive && !alarmSilenced && beepsLeft == 0 && (long)(millis() - alarmNextBurst) >= 0) {
    beep(3);
    alarmNextBurst = millis() + ALARM_REPEAT_MS;
  }
}

// Silences the nodes now in alarm for a minute; a second call brings the sound back
void silenceAlarm() {
  bool allMuted = true;
  for(const auto& t : targets) if(t.alarm && !t.muted) allMuted = false;
  for(auto& t : targets) if(t.alarm) { t.muted = !allMuted; t.mutedAt = millis(); }
  alarmSilenced = !allMuted;
  if(alarmSilenced) beepsLeft = 0; else alarmNextBurst = millis();
  requestRedraw();
}

// ============================================
// Drawing helpers
// ============================================
void button(int x, int y, int w, int h, const String& label, bool active = false, uint8_t font = 2) {
  uint16_t fillc = active ? C_ACCENT : C_BTN, textc = active ? C_WHITE : C_BLACK;
  cv.fillRoundRect(x, y, w, h, 5, fillc);
  cv.drawRoundRect(x, y, w, h, 5, active ? TFT_CYAN : C_BLACK);
  cv.text(label, x + w / 2, y + h / 2, font, textc, textc, MC_DATUM);
}

bool inside(int x, int y, int bx, int by, int bw, int bh) { return x >= bx && x < bx + bw && y >= by && y < by + bh; }

void drawSidebar() {
  cv.fillRect(MAP_W, 0, SIDEBAR_W, SCREEN_H, C_PANEL);
  const char* labels[SIDE_BTN_COUNT] = {"MAP", "WPT", TR("CMP", "BUS"), TR("NEAR", "VIC"), "SET", "", ""};
  int active = page == PAGE_MAP ? 0 : page == PAGE_WAYPOINTS ? 1 : page == PAGE_COMPASS ? 2 : page == PAGE_NEARBY ? 3 : page == PAGE_SETUP ? 4 : -1;
  for(int i = 0; i < SIDE_BTN_COUNT; i++) {
    int x = MAP_W + 2, y = i * SIDE_BTN_H + 2, w = SIDEBAR_W - 4, h = SIDE_BTN_H - 4;
    // The small font keeps the labels well inside the buttons
    button(x, y, w, h, labels[i], i == active, 1);
    if(i >= 5) {
      // Zoom in and zoom out: a plus and a minus drawn as bars
      int cx = x + w / 2, cy = y + h / 2;
      cv.fillRect(cx - 7, cy - 1, 15, 3, C_BLACK);
      if(i == 5) cv.fillRect(cx - 1, cy - 7, 3, 15, C_BLACK);
    }
  }
}

void drawStatusBar() {
  int y = MAP_H;
  cv.fillRect(0, y, MAP_W, STATUS_H, C_BLACK);
  char buf[32];
  if(gps.time.isValid()) snprintf(buf, sizeof(buf), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  else strcpy(buf, "--:--:--");
  cv.text(buf, 3, y + 2, 2, C_WHITE, C_BLACK);
  float hdg = heading();
  if(!gpsBaud) snprintf(buf, sizeof(buf), "%s", TR("no GPS", "no GPS"));
  else if(!gpsFix()) snprintf(buf, sizeof(buf), "%s", TR("no fix", "no fix"));
  else if(courseKnown) snprintf(buf, sizeof(buf), "%.1fkn %03d^", gps.speed.knots(), ((int)lround(hdg)) % 360);
  else snprintf(buf, sizeof(buf), "%.1fkn ---^", gps.speed.knots());
  cv.text(buf, 70, y + 2, 2, gpsFix() ? C_WHITE : C_RED, C_BLACK);
  Waypoint* d = findWp(destId);
  if(d && gpsFix()) {
    double brg = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), d->lat, d->lon);
    double nm = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), d->lat, d->lon) / 1852.0;
    snprintf(buf, sizeof(buf), ">%s^ %sNM", deg3(brg).c_str(), fmtNm(nm).c_str());
    cv.text(buf, MAP_W - 3, y + 2, 2, TFT_YELLOW, C_BLACK, TR_DATUM);
  } else {
    snprintf(buf, sizeof(buf), "SAT %d", gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
    cv.text(buf, MAP_W - 3, y + 2, 2, C_DIM, C_BLACK, TR_DATUM);
  }
}

void drawToast() {
  if(toastUntil == 0) return;
  int y = MAP_H / 2 - 14;
  cv.fillRect(8, y, MAP_W - 16, 28, C_BLACK);
  cv.drawRect(9, y + 1, MAP_W - 18, 26, C_WHITE);
  cv.text(toast, MAP_W / 2, y + 14, 2, C_WHITE, C_BLACK, MC_DATUM);
}

// ============================================
// Map
// ============================================
bool tileOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if(y >= MAP_H) return false; // the rest of this tile is below the map
  cv.pushImage(x, y, w, h, bitmap);
  return true;
}

void initTiles() {
  if(!sdOk) return;
  File dir = SD.open("/tiles");
  if(!dir || !dir.isDirectory()) { Serial.println("No /tiles folder on the SD card"); return; }
  int zMin = 99, zMax = -1;
  for(File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    int z = String(f.name()).toInt();
    if(f.isDirectory() && z >= ZOOM_MIN && z <= ZOOM_MAX) { zMin = min(zMin, z); zMax = max(zMax, z); }
    f.close();
  }
  dir.close();
  if(zMax < 0) return;
  zoomMin = zMin;
  zoomMax = zMax;
  zoom = constrain(zoom, zoomMin, zoomMax);
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tileOutput);
  tilesOk = true;
  Serial.printf("Chart tiles: zoom %d to %d\n", zoomMin, zoomMax);
}

void drawBoat(int x, int y, float course) {
  float r = radians(course);
  cv.fillTriangle(x + 13 * sin(r), y - 13 * cos(r), x + 9 * sin(r + 2.5), y - 9 * cos(r + 2.5), x + 9 * sin(r - 2.5), y - 9 * cos(r - 2.5), C_RED);
  cv.drawTriangle(x + 13 * sin(r), y - 13 * cos(r), x + 9 * sin(r + 2.5), y - 9 * cos(r + 2.5), x + 9 * sin(r - 2.5), y - 9 * cos(r - 2.5), C_BLACK);
  cv.fillCircle(x, y, 2, C_WHITE);
}

void drawMap() {
  cv.fillRect(0, 0, MAP_W, MAP_H, C_SEA);

  if(tilesOk) {
    long long ox = llround(lonToWX(mapLon)) - MAP_W / 2, oy = llround(latToWY(mapLat)) - MAP_H / 2;
    long n = 1L << zoom;
    for(long ty = (long)floor((double)oy / TILE_SIZE); ty <= (long)floor((double)(oy + MAP_H - 1) / TILE_SIZE); ty++) {
      if(ty < 0 || ty >= n) continue;
      for(long tx = (long)floor((double)ox / TILE_SIZE); tx <= (long)floor((double)(ox + MAP_W - 1) / TILE_SIZE); tx++) {
        char path[48];
        snprintf(path, sizeof(path), "/tiles/%d/%ld/%ld.jpg", zoom, ((tx % n) + n) % n, ty);
        if(SD.exists(path)) TJpgDec.drawSdJpg((int32_t)(tx * TILE_SIZE - ox), (int32_t)(ty * TILE_SIZE - oy), path);
        readGps(); // reading the card takes a while: do not lose GPS data meanwhile
      }
    }
  }

  // Line to the destination, then the waypoints
  Waypoint* d = findWp(destId);
  if(d && gpsFix()) {
    int bx = lonToPx(gps.location.lng()), by = latToPx(gps.location.lat()), wx = lonToPx(d->lon), wy = latToPx(d->lat);
    for(int k = -1; k <= 1; k++) { cv.drawLine(bx + k, by, wx + k, wy, C_RED); cv.drawLine(bx, by + k, wx, wy + k, C_RED); }
  }
  for(const auto& w : waypoints) {
    if(!w.visible) continue;
    int x = lonToPx(w.lon), y = latToPx(w.lat);
    if(x < -20 || x > MAP_W + 20 || y < -20 || y > MAP_H + 20) continue;
    uint16_t c = w.id == destId ? C_RED : TFT_PURPLE;
    cv.drawCircle(x, y, 6, c);
    cv.drawCircle(x, y, 5, c);
    cv.drawLine(x - 10, y, x + 10, y, c);
    cv.drawLine(x, y - 10, x, y + 10, c);
    cv.text(w.name, x, y - 22, 2, c, C_WHITE, TC_DATUM);
  }

  // Other nodes: boat = hollow arrow with a course vector, person = ring with a dot, object = diamond
  for(const auto& t : targets) {
    int x = lonToPx(t.lon), y = latToPx(t.lat);
    if(x < -20 || x > MAP_W + 20 || y < -20 || y > MAP_H + 20) continue;
    bool stale = millis() - t.lastSeen > TARGET_STALE_MS;
    uint16_t c = t.alarm ? C_RED : stale ? C_DIM : TFT_BLUE;
    if(t.type == NODE_TYPE_PERSON) {
      cv.fillCircle(x, y, 9, C_WHITE); cv.drawCircle(x, y, 9, c); cv.drawCircle(x, y, 8, c); cv.fillCircle(x, y, 3, c);
    } else if(t.type == NODE_TYPE_OBJECT) {
      cv.fillTriangle(x, y - 10, x + 10, y, x - 10, y, C_WHITE); cv.fillTriangle(x, y + 10, x + 10, y, x - 10, y, C_WHITE);
      for(int k = 0; k < 2; k++) { cv.drawLine(x, y - 10 + k, x + 10 - k, y, c); cv.drawLine(x + 10 - k, y, x, y + 10 - k, c); cv.drawLine(x, y + 10 - k, x - 10 + k, y, c); cv.drawLine(x - 10 + k, y, x, y - 10 + k, c); }
    } else {
      float r = radians(t.cog);
      if(!stale && t.sog >= 0.3) {
        float len = min(50.0, t.sog * 0.514444 * 180.0 / metersPerPixel()); // where it will be in three minutes
        cv.drawLine(x, y, x + len * sin(r), y - len * cos(r), c);
      }
      int x1 = x + 12 * sin(r), y1 = y - 12 * cos(r), x2 = x + 9 * sin(r + 2.5), y2 = y - 9 * cos(r + 2.5), x3 = x + 9 * sin(r - 2.5), y3 = y - 9 * cos(r - 2.5);
      cv.fillTriangle(x1, y1, x2, y2, x3, y3, C_WHITE);
      cv.drawTriangle(x1, y1, x2, y2, x3, y3, c);
      cv.drawTriangle(x1, y1 + 1, x2, y2 + 1, x3, y3 + 1, c);
    }
    cv.text(t.name, x, y + 13, 2, c, C_WHITE, TC_DATUM);
  }

  // The boat
  if(gpsFix()) {
    int x = lonToPx(gps.location.lng()), y = latToPx(gps.location.lat());
    lastBoatX = x;
    lastBoatY = y;
    if(x >= 0 && x < MAP_W && y >= 0 && y < MAP_H) drawBoat(x, y, heading());
  } else {
    cv.text(!gpsBaud ? TR("GPS NOT FOUND", "GPS NON TROVATO") : TR("WAITING FOR GPS FIX", "IN ATTESA DEL FIX GPS"), MAP_W / 2, 48, 2, C_RED, C_WHITE, TC_DATUM);
  }
  if(!tilesOk) cv.text(!sdOk ? TR("NO SD CARD", "SCHEDA SD ASSENTE") : TR("NO CHARTS ON THE SD CARD", "NESSUNA CARTA SULLA SD"), MAP_W / 2, 70, 2, C_RED, C_WHITE, TC_DATUM);

  // Collision alarm: red band at the top; a tap on it silences the sound for a minute
  const Target* worst = worstTarget();
  int top = 0;
  if(worst) {
    char b[56];
    snprintf(b, sizeof(b), "%s %s %sNM %d:%02d", TR("COLLISION", "ABBORDO"), worst->name, fmtNm(worst->cpaNm).c_str(), (int)worst->tcpaS / 60, (int)worst->tcpaS % 60);
    cv.fillRect(0, 0, MAP_W, 22, C_RED);
    cv.text(b, 4, 3, 2, C_WHITE, C_RED);
    if(soundOn) cv.text(alarmSilenced ? TR("[muted]", "[muto]") : TR("[tap=mute]", "[tocca=muto]"), MAP_W - 4, 3, 2, C_WHITE, C_RED, TR_DATUM);
    top = 22;
  }
  if(!sharePosition) cv.text(TR("HIDDEN", "NASCOSTA"), 48, top + 6, 2, C_RED, C_WHITE);

  // Follow button (top left) and zoom level
  button(3, top + 3, 40, 36, follow ? "(o)" : " o ", follow);
  char z[8];
  snprintf(z, sizeof(z), "Z%d", zoom);
  cv.text(z, 3, MAP_H - 18, 2, C_BLACK, C_WHITE);

  // Scale bar
  static const float steps[] = {0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100};
  float nm = steps[0];
  for(float s : steps) if(s * 1852.0 / metersPerPixel() <= 90) nm = s;
  int bw = nm * 1852.0 / metersPerPixel(), bx = MAP_W - 8 - bw, by = MAP_H - 8;
  cv.fillRect(bx - 2, by - 16, bw + 6, 20, C_WHITE);
  cv.drawLine(bx, by, bx + bw, by, C_BLACK);
  cv.drawLine(bx, by - 5, bx, by, C_BLACK);
  cv.drawLine(bx + bw, by - 5, bx + bw, by, C_BLACK);
  cv.text(String(nm, nm < 0.1 ? 2 : nm < 1 ? 1 : 0) + " NM", bx + bw / 2, by - 16, 1, C_BLACK, C_WHITE, TC_DATUM);
}

void setZoom(int z) {
  zoom = constrain(z, zoomMin, zoomMax);
  saveSettings();
  if(page != PAGE_MAP) page = PAGE_MAP;
  requestRedraw();
}

void setFollow(bool on) {
  follow = on;
  if(on && gpsFix()) { mapLat = gps.location.lat(); mapLon = gps.location.lng(); }
  showToast(on ? TR("Following the boat", "La mappa segue la barca") : TR("Free map", "Mappa libera"));
}

void addWaypoint(double lat, double lon) {
  Waypoint w;
  w.id = nextWpId();
  snprintf(w.name, sizeof(w.name), "WP%d", w.id);
  w.lat = lat;
  w.lon = lon;
  waypoints.push_back(w);
  saveWaypoints();
  selectedWp = waypoints.size() - 1;
  beep(1);
  showToast(String(TR("Waypoint added: ", "Waypoint aggiunto: ")) + w.name);
}

// ============================================
// Waypoints page
// ============================================
#define LIST_ROWS 5
#define LIST_ROW_H 30
#define LIST_Y0 26
#define ACTION_Y (SCREEN_H - 34)
#define ACTION_COUNT 4

void actionBar(const char* const labels[ACTION_COUNT]) {
  int w = (MAP_W - 4 - (ACTION_COUNT - 1) * 4) / ACTION_COUNT;
  for(int i = 0; i < ACTION_COUNT; i++) button(2 + i * (w + 4), ACTION_Y, w, 32, labels[i]);
}

int actionAt(int x, int y) {
  if(y < ACTION_Y || x >= MAP_W) return -1;
  int w = (MAP_W - 4 - (ACTION_COUNT - 1) * 4) / ACTION_COUNT;
  int i = (x - 2) / (w + 4);
  return (i >= 0 && i < ACTION_COUNT) ? i : -1;
}

void drawWaypointsPage() {
  cv.fillRect(0, 0, MAP_W, SCREEN_H, C_WHITE);
  cv.text("WAYPOINT", 6, 3, 2, C_ACCENT, C_WHITE);
  if(waypoints.empty()) cv.text(TR("None yet: long press on the map", "Nessuno: tocco lungo sulla mappa"), 6, LIST_Y0 + 8, 2, C_DIM, C_WHITE);

  if(selectedWp >= (int)waypoints.size()) selectedWp = waypoints.size() - 1;
  if(selectedWp >= 0 && selectedWp < listTop) listTop = selectedWp;
  if(selectedWp >= listTop + LIST_ROWS) listTop = selectedWp - LIST_ROWS + 1;
  listTop = constrain(listTop, 0, max(0, (int)waypoints.size() - LIST_ROWS));

  for(int r = 0; r < LIST_ROWS && listTop + r < (int)waypoints.size(); r++) {
    const Waypoint& w = waypoints[listTop + r];
    int y = LIST_Y0 + r * LIST_ROW_H;
    bool sel = listTop + r == selectedWp;
    uint16_t bg = sel ? C_ACCENT : C_WHITE, fg = sel ? C_WHITE : C_BLACK;
    cv.fillRect(0, y, MAP_W - 36, LIST_ROW_H - 2, bg);
    String label = String(w.name) + (w.id == destId ? TR("  <- DEST", "  <- DEST") : "");
    cv.text(label, 6, y + 1, 2, fg, bg);
    if(gpsFix()) {
      double nm = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), w.lat, w.lon) / 1852.0;
      double brg = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), w.lat, w.lon);
      cv.text(fmtNm(nm) + " NM  " + deg3(brg) + "^", 6, y + 16, 1, sel ? C_WHITE : C_DIM, bg);
    } else {
      cv.text(String(w.lat, 5) + ", " + String(w.lon, 5), 6, y + 16, 1, sel ? C_WHITE : C_DIM, bg);
    }
  }
  // Scroll buttons, when the list is longer than the screen
  if((int)waypoints.size() > LIST_ROWS) {
    button(MAP_W - 34, LIST_Y0, 32, 60, "^");
    button(MAP_W - 34, LIST_Y0 + 90, 32, 60, "v");
  }
  static const char* const en[ACTION_COUNT] = {"Go to", "Show", "Name", "Delete"};
  static const char* const it[ACTION_COUNT] = {"Vai", "Mostra", "Nome", "Elimina"};
  actionBar(langIt ? it : en);
}

void wpRenamed(const String& s) {
  if(selectedWp < 0 || selectedWp >= (int)waypoints.size() || s.length() == 0) return;
  strncpy(waypoints[selectedWp].name, s.c_str(), NAME_LEN - 1);
  waypoints[selectedWp].name[NAME_LEN - 1] = '\0';
  saveWaypoints();
}

void openKeyboard(const String& title, const String& text, int maxLen, void (*done)(const String&)) {
  kbTitle = title; kbText = text; kbMax = maxLen; kbDone = done; kbReturn = page; kbShift = false;
  goTo(PAGE_KEYBOARD);
}

void tapWaypointsPage(int x, int y) {
  if((int)waypoints.size() > LIST_ROWS && x >= MAP_W - 36) {
    if(inside(x, y, MAP_W - 36, LIST_Y0, 36, 60)) { selectedWp = max(0, selectedWp - 1); requestRedraw(); }
    else if(inside(x, y, MAP_W - 36, LIST_Y0 + 90, 36, 60)) { selectedWp = min((int)waypoints.size() - 1, selectedWp + 1); requestRedraw(); }
    return;
  }
  if(y >= LIST_Y0 && y < LIST_Y0 + LIST_ROWS * LIST_ROW_H) {
    int idx = listTop + (y - LIST_Y0) / LIST_ROW_H;
    if(idx < (int)waypoints.size()) { selectedWp = idx; requestRedraw(); }
    return;
  }
  int a = actionAt(x, y);
  if(a < 0) return;
  if(selectedWp < 0 || selectedWp >= (int)waypoints.size()) { showToast(TR("Select a waypoint first", "Prima scegli un waypoint")); return; }
  Waypoint& w = waypoints[selectedWp];
  switch(a) {
    case 0: // go to / stop
      destId = destId == w.id ? -1 : w.id;
      saveSettings();
      showToast(destId < 0 ? TR("Destination cleared", "Destinazione annullata") : String(TR("Destination: ", "Destinazione: ")) + w.name);
      break;
    case 1: // show on the map
      mapLat = w.lat; mapLon = w.lon; follow = false; mapPlaced = true;
      goTo(PAGE_MAP);
      break;
    case 2:
      openKeyboard(TR("Waypoint name", "Nome del waypoint"), w.name, NAME_LEN - 1, wpRenamed);
      break;
    case 3:
      if(destId == w.id) { destId = -1; saveSettings(); }
      waypoints.erase(waypoints.begin() + selectedWp);
      selectedWp = min(selectedWp, (int)waypoints.size() - 1);
      saveWaypoints();
      showToast(TR("Waypoint deleted", "Waypoint eliminato"));
      break;
  }
}

// ============================================
// Compass page: dark face, rotating rose, fixed red bow marker, green dot for the destination
// ============================================
void drawCompassPage() {
  const uint16_t BG = C_BLACK, RING = TFT_LIGHTGREY;
  const int cx = 104, cy = 122, R = 98;
  cv.fillRect(0, 0, MAP_W, SCREEN_H, BG);
  float hdg = heading();
  cv.drawCircle(cx, cy, R, RING);
  cv.drawCircle(cx, cy, R - 1, RING);
  for(int d = 0; d < 360; d += 10) {
    float a = radians(d - hdg);
    int len = d % 90 == 0 ? 13 : d % 30 == 0 ? 9 : 5;
    cv.drawLine(cx + (R - len) * sin(a), cy - (R - len) * cos(a), cx + (R - 2) * sin(a), cy - (R - 2) * cos(a), RING);
  }
  const char* names[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  for(int i = 0; i < 8; i++) {
    float a = radians(i * 45 - hdg);
    bool mainPoint = i % 2 == 0;
    cv.text(names[i], cx + (R - 24) * sin(a), cy - (R - 24) * cos(a), mainPoint ? 4 : 2, i == 0 ? C_RED : mainPoint ? C_WHITE : C_DIM, BG, MC_DATUM);
  }
  Waypoint* d = findWp(destId);
  double brg = 0, nm = 0;
  bool have = d && gpsFix();
  if(have) {
    brg = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), d->lat, d->lon);
    nm = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), d->lat, d->lon) / 1852.0;
    float a = radians(brg - hdg);
    int x = cx + (R - 1) * sin(a), y = cy - (R - 1) * cos(a);
    cv.fillCircle(x, y, 7, TFT_GREEN);
    cv.drawCircle(x, y, 7, C_WHITE);
  }
  cv.fillTriangle(cx, cy - R + 12, cx - 9, cy - R - 8, cx + 9, cy - R - 8, C_RED);

  // Heading, large, in the middle
  cv.fillRect(cx - 42, cy - 18, 84, 42, BG);
  cv.text(courseKnown ? deg3(hdg) + "^" : String("---^"), cx, cy - 4, 4, moving() ? C_WHITE : C_DIM, BG, MC_DATUM);
  cv.text(moving() ? TR("from COG", "da COG") : TR("COG, stopped", "COG, ferma"), cx, cy + 16, 1, C_DIM, BG, MC_DATUM);

  // Small data column on the right
  int x = 208, y = 6;
  auto row = [&](const String& name, const String& value, uint16_t colour) {
    cv.text(name, x, y, 1, C_DIM, BG);
    cv.text(value, x, y + 10, 2, colour, BG);
    y += 30;
  };
  row(TR("SPEED", "VELOCITA'"), gps.speed.isValid() ? String(gps.speed.knots(), 1) + " kn" : String("--.-"), C_WHITE);
  if(d) row(TR("DEST", "DEST"), String(d->name).substring(0, 8), TFT_GREEN);
  if(have) {
    row(TR("BEARING", "RILEV."), deg3(brg) + "^", TFT_GREEN);
    row(TR("DISTANCE", "DISTANZA"), fmtNm(nm) + " NM", TFT_GREEN);
    if(courseKnown) {
      int turn = (int)lround(brg - hdg);
      while(turn > 180) turn -= 360;
      while(turn < -180) turn += 360;
      row(TR("STEER", "ACCOSTA"), abs(turn) < 3 ? String(TR("on course", "in rotta")) : String(abs(turn)) + (turn > 0 ? TR("^ stbd", "^ dritta") : TR("^ port", "^ sin.")), C_WHITE);
    }
  } else if(!d) {
    row(TR("DEST", "DEST"), TR("none", "nessuna"), C_DIM);
  }
}

// ============================================
// Nearby page: the nodes received over LoRa
// ============================================
void drawNearbyPage() {
  cv.fillRect(0, 0, MAP_W, SCREEN_H, C_WHITE);
  cv.text(TR("NEARBY", "VICINI"), 6, 3, 2, C_ACCENT, C_WHITE);
  char buf[64];
  int y = 24;
  if(!radioOk) cv.text(TR("LoRa radio not found", "Radio LoRa non trovata"), 6, y, 2, C_RED, C_WHITE);
  else if(targets.empty()) cv.text(TR("No nodes received yet", "Nessun nodo ricevuto"), 6, y, 2, C_DIM, C_WHITE);
  static const char* const typeEn[] = {"", " (person)", " (object)"};
  static const char* const typeIt[] = {"", " (persona)", " (oggetto)"};
  for(const auto& t : targets) {
    if(y > SCREEN_H - 62) break;
    bool stale = millis() - t.lastSeen > TARGET_STALE_MS;
    uint16_t c = t.alarm ? C_RED : stale ? C_DIM : C_BLACK;
    String line = String(t.name) + (langIt ? typeIt[t.type] : typeEn[t.type]);
    if(gpsFix()) {
      double nm = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), t.lat, t.lon) / 1852.0;
      double brg = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), t.lat, t.lon);
      line += "  " + fmtNm(nm) + "NM " + deg3(brg) + "^";
    }
    cv.text(line, 6, y, 2, c, C_WHITE);
    if(t.cpaValid) snprintf(buf, sizeof(buf), "%.1fkn %s^  CPA %sNM %d:%02d", t.sog, deg3(t.cog).c_str(), fmtNm(t.cpaNm).c_str(), (int)t.tcpaS / 60, (int)t.tcpaS % 60);
    else snprintf(buf, sizeof(buf), "%.1fkn %s^  %s", t.sog, deg3(t.cog).c_str(), TR("not approaching", "non in avvicinamento"));
    cv.text(buf, 6, y + 16, 1, c, C_WHITE);
    if(t.relayed) snprintf(buf, sizeof(buf), "%lus  %s", (millis() - t.lastSeen) / 1000, TR("relayed", "ripetuto"));
    else snprintf(buf, sizeof(buf), "%lus  %.0fdBm", (millis() - t.lastSeen) / 1000, t.rssi);
    cv.text(buf, MAP_W - 6, y + 16, 1, C_DIM, C_WHITE, TR_DATUM);
    y += 30;
  }
  snprintf(buf, sizeof(buf), "%s: %s   LoRa %.3f MHz", TR("Me", "Io"), boatName, LORA_FREQ);
  cv.text(buf, 6, SCREEN_H - 34, 2, C_DIM, C_WHITE);
  if(sharePosition) snprintf(buf, sizeof(buf), "TX %lu  RX %lu  %s %lu", (unsigned long)txCount, (unsigned long)rxCount, TR("relayed", "ripetuti"), (unsigned long)relayCount);
  else snprintf(buf, sizeof(buf), "%s  RX %lu  %s %lu", TR("HIDDEN", "NASCOSTA"), (unsigned long)rxCount, TR("relayed", "ripetuti"), (unsigned long)relayCount);
  cv.text(buf, 6, SCREEN_H - 18, 2, C_DIM, C_WHITE);
}

// ============================================
// Setup page
// ============================================
#define SETUP_COLS 2
#define SETUP_ITEMS 8
#define SETUP_BTN_W 132
#define SETUP_BTN_H 32
#define SETUP_Y0 24

void setupButtonRect(int i, int& x, int& y) {
  x = 3 + (i % SETUP_COLS) * (SETUP_BTN_W + 4);
  y = SETUP_Y0 + (i / SETUP_COLS) * (SETUP_BTN_H + 4);
}

void drawSetupPage() {
  cv.fillRect(0, 0, MAP_W, SCREEN_H, C_WHITE);
  cv.text(TR("SETUP", "IMPOSTAZIONI"), 6, 3, 2, C_ACCENT, C_WHITE);
  String labels[SETUP_ITEMS] = {
    String(TR("Name: ", "Nome: ")) + boatName,
    langIt ? "Lingua: Italiano" : "Language: English",
    soundOn ? TR("[X] Sound", "[X] Suono") : TR("[ ] Sound", "[ ] Suono"),
    TR("Test sound", "Prova suono"),
    follow ? TR("[X] Follow boat", "[X] Segui barca") : TR("[ ] Follow boat", "[ ] Segui barca"),
    TR("Calibrate touch", "Calibra il touch"),
    sharePosition ? TR("[X] Share position", "[X] Trasmetti pos.") : TR("[ ] Share position", "[ ] Trasmetti pos."),
    String("v") + FW_VERSION + "  " + String(batteryVolts(), 2) + " V"};
  for(int i = 0; i < SETUP_ITEMS; i++) {
    int x, y;
    setupButtonRect(i, x, y);
    button(x, y, SETUP_BTN_W, SETUP_BTN_H, labels[i]);
  }
  int y = SETUP_Y0 + 4 * (SETUP_BTN_H + 4) + 2;
  char buf[64];
  if(!gpsBaud) snprintf(buf, sizeof(buf), "GPS: %s, %lu byte", TR("not found on IO35", "non trovato su IO35"), (unsigned long)gpsBytes);
  else snprintf(buf, sizeof(buf), "GPS: %lu baud, %s, %d sat", (unsigned long)gpsBaud, gpsFix() ? "fix" : TR("no fix", "no fix"), gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  cv.text(buf, 6, y, 2, C_BLACK, C_WHITE);
  if(!sdOk) snprintf(buf, sizeof(buf), "SD: %s", TR("not found", "non trovata"));
  else if(!tilesOk) snprintf(buf, sizeof(buf), "SD: ok, %s", TR("no charts", "nessuna carta"));
  else snprintf(buf, sizeof(buf), "SD: ok, %s %d-%d, %d WP", TR("charts zoom", "carte zoom"), zoomMin, zoomMax, (int)waypoints.size());
  cv.text(buf, 6, y + 16, 2, C_BLACK, C_WHITE);
  if(!radioOk) snprintf(buf, sizeof(buf), "LoRa: %s", TR("radio not found", "radio non trovata"));
  else snprintf(buf, sizeof(buf), "LoRa: ok  TX %lu  RX %lu", (unsigned long)txCount, (unsigned long)rxCount);
  cv.text(buf, 6, y + 32, 2, C_BLACK, C_WHITE);
}

void nameEntered(const String& s) {
  if(s.length() == 0) return;
  strncpy(boatName, s.c_str(), BOAT_NAME_LEN - 1);
  boatName[BOAT_NAME_LEN - 1] = '\0';
  saveSettings();
}

void tapSetupPage(int x, int y) {
  for(int i = 0; i < SETUP_ITEMS; i++) {
    int bx, by;
    setupButtonRect(i, bx, by);
    if(!inside(x, y, bx, by, SETUP_BTN_W, SETUP_BTN_H)) continue;
    switch(i) {
      case 0: openKeyboard(TR("Boat name", "Nome della barca"), boatName, BOAT_NAME_LEN - 1, nameEntered); break;
      case 1: langIt = !langIt; saveSettings(); requestRedraw(); break;
      case 2: soundOn = !soundOn; saveSettings(); beep(1); requestRedraw(); break;
      case 3: { bool was = soundOn; soundOn = true; beep(3); soundOn = was; } break;
      case 4: setFollow(!follow); break;
      case 5: calibrateTouch(); break;
      case 6:
        sharePosition = !sharePosition;
        saveSettings();
        showToast(sharePosition ? TR("Position shared with the others", "Posizione trasmessa agli altri") : TR("Hidden: not transmitting", "Nascosta: non trasmetto"));
        break;
    }
    return;
  }
}

// ============================================
// On-screen keyboard (full screen)
// ============================================
#define KB_ROWS 4
#define KB_COLS 10
#define KB_KEY_W (SCREEN_W / KB_COLS)
#define KB_KEY_H 37
#define KB_Y0 50
#define KB_BOTTOM_Y (KB_Y0 + KB_ROWS * KB_KEY_H)
const char* const kbRows[KB_ROWS] = {"1234567890", "qwertyuiop", "asdfghjkl-", "zxcvbnm.+'"};

void drawKeyboardPage() {
  cv.fill(C_WHITE);
  cv.text(kbTitle, 6, 2, 2, C_ACCENT, C_WHITE);
  cv.drawRect(4, 20, SCREEN_W - 8, 26, C_BLACK);
  cv.text(kbText + "_", 8, 22, 4, C_BLACK, C_WHITE);
  for(int r = 0; r < KB_ROWS; r++) {
    for(int c = 0; c < KB_COLS; c++) {
      char ch = kbRows[r][c];
      if(kbShift && ch >= 'a' && ch <= 'z') ch -= 32;
      button(c * KB_KEY_W + 1, KB_Y0 + r * KB_KEY_H + 1, KB_KEY_W - 2, KB_KEY_H - 2, String(ch), false, 4);
    }
  }
  int h = SCREEN_H - KB_BOTTOM_Y - 1;
  button(1, KB_BOTTOM_Y + 1, 62, h, kbShift ? "abc" : "ABC", kbShift);
  button(65, KB_BOTTOM_Y + 1, 94, h, TR("space", "spazio"));
  button(161, KB_BOTTOM_Y + 1, 54, h, "<-");
  button(217, KB_BOTTOM_Y + 1, 50, h, "X");
  button(269, KB_BOTTOM_Y + 1, 50, h, "OK", true);
}

void tapKeyboardPage(int x, int y) {
  if(y >= KB_Y0 && y < KB_BOTTOM_Y) {
    int r = (y - KB_Y0) / KB_KEY_H, c = x / KB_KEY_W;
    if(r < KB_ROWS && c < KB_COLS && (int)kbText.length() < kbMax) {
      char ch = kbRows[r][c];
      if(kbShift && ch >= 'a' && ch <= 'z') ch -= 32;
      kbText += ch; // no comma on the keyboard: names are stored in comma-separated files
    }
  } else if(y >= KB_BOTTOM_Y) {
    if(x < 64) kbShift = !kbShift;
    else if(x < 160) { if((int)kbText.length() < kbMax) kbText += ' '; }
    else if(x < 216) { if(kbText.length()) kbText.remove(kbText.length() - 1); }
    else if(x < 268) { page = kbReturn; }
    else { kbText.trim(); if(kbDone) kbDone(kbText); page = kbReturn; }
  }
  requestRedraw();
}

// ============================================
// Touch actions
// ============================================
void onTap(int x, int y) {
  if(page == PAGE_KEYBOARD) { tapKeyboardPage(x, y); return; }
  if(x >= MAP_W) {
    switch(y / SIDE_BTN_H) {
      case 0: goTo(PAGE_MAP); break;
      case 1: goTo(PAGE_WAYPOINTS); break;
      case 2: goTo(PAGE_COMPASS); break;
      case 3: goTo(PAGE_NEARBY); break;
      case 4: goTo(PAGE_SETUP); break;
      case 5: setZoom(zoom + 1); break;
      case 6: setZoom(zoom - 1); break;
    }
    return;
  }
  switch(page) {
    case PAGE_MAP:
      if(alarmActive && y < 22) silenceAlarm();
      else if(inside(x, y, 0, alarmActive ? 22 : 0, 48, 44)) setFollow(!follow);
      break;
    case PAGE_WAYPOINTS: tapWaypointsPage(x, y); break;
    case PAGE_SETUP: tapSetupPage(x, y); break;
    default: break;
  }
}

void onLongPress(int x, int y) {
  if(page == PAGE_MAP && x < MAP_W && y < MAP_H && !inside(x, y, 0, 0, 48, 44)) addWaypoint(pxToLat(y), pxToLon(x));
}

void onDrag(int dx, int dy) {
  if(page != PAGE_MAP) return;
  mapLon = pxToLon(MAP_W / 2 - dx);
  mapLat = pxToLat(MAP_H / 2 - dy);
  mapPlaced = true;
  follow = false;
  requestRedraw();
}

// ============================================
// Rendering
// ============================================
void render() {
  switch(page) {
    case PAGE_MAP: drawMap(); drawStatusBar(); break;
    case PAGE_WAYPOINTS: drawWaypointsPage(); break;
    case PAGE_COMPASS: drawCompassPage(); break;
    case PAGE_NEARBY: drawNearbyPage(); break;
    case PAGE_SETUP: drawSetupPage(); break;
    case PAGE_KEYBOARD: drawKeyboardPage(); break;
  }
  if(page != PAGE_KEYBOARD) drawSidebar();
  drawToast();
  cv.push();
}

void updateDisplay() {
  unsigned long now = millis();
  if(toastUntil && now > toastUntil) { toastUntil = 0; requestRedraw(); }

  if(!needsRedraw) {
    if(page == PAGE_MAP) {
      // The boat moved on the screen: redraw, but not more than once a second
      if(gpsFix() && now - lastFullDraw > 1000) {
        int bx = lonToPx(gps.location.lng()), by = latToPx(gps.location.lat());
        if(abs(bx - lastBoatX) >= 2 || abs(by - lastBoatY) >= 2) needsRedraw = true;
      }
      // Clock and speed: only the status bar, which is cheap
      if(!needsRedraw && now - lastStatusDraw > 1000) {
        lastStatusDraw = now;
        drawStatusBar();
        cv.pushRows(MAP_H, STATUS_H);
      }
    } else if((page == PAGE_COMPASS || page == PAGE_SETUP || page == PAGE_WAYPOINTS || page == PAGE_NEARBY) && now - lastFullDraw > 1000) {
      needsRedraw = true;
    }
  }
  if(!needsRedraw) return;
  needsRedraw = false;
  render();
  lastFullDraw = lastStatusDraw = millis();
}

// ============================================
// Setup and loop
// ============================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\nNavPlotter ESP32-32E %s\n", FW_VERSION);

  for(int pin : {PIN_LED_R, PIN_LED_G, PIN_LED_B}) { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); } // LED off
  pinMode(PIN_LORA_CS, OUTPUT);
  digitalWrite(PIN_LORA_CS, HIGH); // keep a radio module on the shared bus quiet
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  loadSettings();
  initTouch();
  initSound();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BLACK);
  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.drawString("NavPlotter", SCREEN_W / 2, SCREEN_H / 2 - 14, 4);
  tft.drawString(FW_VERSION, SCREEN_W / 2, SCREEN_H / 2 + 14, 2);

  if(!cv.begin()) {
    tft.drawString("Not enough memory for the screen", SCREEN_W / 2, SCREEN_H / 2 + 40, 2);
    Serial.println("Canvas allocation failed");
    while(true) delay(1000);
  }
  Serial.printf("Free heap after the canvas: %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  sdSpi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdOk = SD.begin(PIN_SD_CS, sdSpi, 20000000U);
  if(!sdOk) { SD.end(); sdOk = SD.begin(PIN_SD_CS, sdSpi, 4000000U); }
  Serial.println(sdOk ? "SD card ready" : "SD card not found");
  initTiles();
  loadWaypoints();
  if(destId >= 0 && !findWp(destId)) destId = -1;

  // Calibrate at the first start, or when the screen is held pressed while the board starts
  // (the way out of a calibration gone wrong)
  bool held = true;
  for(int i = 0; i < 20 && held; i++) { held = touchPressed(); delay(40); }
  if(!calOk || held) calibrateTouch();
  initRadio();
  detectGpsBaud();
  beep(1);
  requestRedraw();
}

void loop() {
  readGps();
  handleTouch();
  handleRadio();
  updateTraffic();
  handleSound();
  updateDisplay();
}
