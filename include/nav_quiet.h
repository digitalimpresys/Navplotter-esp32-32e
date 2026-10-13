// Nodes whose collision alarm is switched off ("quiet"): friends sailing together would otherwise set each
// other's alarms off all the time. A quiet node is still received, shown and written to; it just never
// raises the alarm. The list is kept by node id and lasts until power-off: at the next start every alarm is
// on again, so that nobody stays silenced by mistake.
// This file MUST be identical in every NavPlotter that has it.
#pragma once
#include <Arduino.h>

#define QUIET_MAX 16

static uint32_t quietIds[QUIET_MAX];
static uint8_t quietCount = 0;

static inline bool isQuiet(uint32_t id) {
  for(uint8_t i = 0; i < quietCount; i++) if(quietIds[i] == id) return true;
  return false;
}

// Returns true when the list changed (and so has to be saved)
static inline bool setQuiet(uint32_t id, bool on) {
  if(id == 0) return false;
  for(uint8_t i = 0; i < quietCount; i++) {
    if(quietIds[i] != id) continue;
    if(on) return false;
    quietIds[i] = quietIds[--quietCount];
    return true;
  }
  if(!on) return false;
  // Full: the oldest entry makes room
  if(quietCount == QUIET_MAX) { memmove(quietIds, quietIds + 1, sizeof(uint32_t) * (QUIET_MAX - 1)); quietCount--; }
  quietIds[quietCount++] = id;
  return true;
}

// For the settings file: ids in hexadecimal, separated by commas
static inline String quietToText() {
  String s;
  char b[12];
  for(uint8_t i = 0; i < quietCount; i++) { snprintf(b, sizeof(b), "%s%08lX", i ? "," : "", (unsigned long)quietIds[i]); s += b; }
  return s;
}

static inline void quietFromText(const String& text) {
  quietCount = 0;
  int start = 0;
  while(start < (int)text.length() && quietCount < QUIET_MAX) {
    int comma = text.indexOf(',', start);
    if(comma < 0) comma = text.length();
    uint32_t id = strtoul(text.substring(start, comma).c_str(), nullptr, 16);
    if(id && !isQuiet(id)) quietIds[quietCount++] = id;
    start = comma + 1;
  }
}

// For the web app: ["A7C94F1B","BA2680C5"]
static inline String quietJson() {
  String s = "[";
  char b[14];
  for(uint8_t i = 0; i < quietCount; i++) { snprintf(b, sizeof(b), "%s\"%08lX\"", i ? "," : "", (unsigned long)quietIds[i]); s += b; }
  return s + "]";
}
