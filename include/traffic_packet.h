// Traffico LoRa tra plotter: pacchetto radio (formato nostro, versione 1) e ritrasmissione mesh.
// Questo file DEVE essere identico in tutti i NavPlotter: plotter T-Deck (include/), simulatori
// (simulatori/*/include/), navplotter-heltec/include/, navplotter-esp32-32e e navplotter-pico-w.
#pragma once
#include <Arduino.h>

#define LORA_FREQ 869.525   // MHz: banda europea 869,4-869,65 (fino a 500 mW, 10% del tempo)
#define LORA_BW 125.0       // kHz
#define LORA_SF 9
#define LORA_CR 5
#define LORA_SYNC 0x4E
#define LORA_PREAMBLE 8
#define BOAT_NAME_LEN 12    // 11 caratteri + terminatore
#define TRAFFIC_FLAG_SIMULATED 0x01
// Tipo di nodo, nei bit 1-2 di flags: decide il simbolo mostrato dagli altri
#define TRAFFIC_TYPE_SHIFT 1
#define TRAFFIC_TYPE_MASK 0x03
#define NODE_TYPE_BOAT 0     // barca
#define NODE_TYPE_PERSON 1   // persona (per esempio uomo a mare)
#define NODE_TYPE_OBJECT 2   // oggetto (boa, tender, attrezzatura)
// Nodo che dorme tra una trasmissione e l'altra (una boa): nei bit 3-5 di flags dichiara ogni quanto
// trasmette, cosi' chi lo riceve non lo considera perso tra un pacchetto e il successivo. 0 = nodo normale.
#define TRAFFIC_SLEEP_SHIFT 3
#define TRAFFIC_SLEEP_MASK 0x07
static const uint8_t TRAFFIC_SLEEP_MINUTES[8] = {0, 1, 2, 5, 10, 15, 30, 60};
#define TARGET_STALE_MS 60000UL     // nodo normale: dopo un minuto senza notizie e' "vecchio"
#define TARGET_EXPIRE_MS 180000UL   // dopo tre viene tolto
// Per un nodo che dorme: vecchio dopo un intervallo e mezzo, tolto dopo tre
static inline unsigned long trafficStaleMs(uint8_t flags) {
  uint8_t m = TRAFFIC_SLEEP_MINUTES[(flags >> TRAFFIC_SLEEP_SHIFT) & TRAFFIC_SLEEP_MASK];
  return m ? m * 90000UL + 30000UL : TARGET_STALE_MS;
}
static inline unsigned long trafficExpireMs(uint8_t flags) {
  uint8_t m = TRAFFIC_SLEEP_MINUTES[(flags >> TRAFFIC_SLEEP_SHIFT) & TRAFFIC_SLEEP_MASK];
  return m ? m * 180000UL + 60000UL : TARGET_EXPIRE_MS;
}

// Mesh: ogni pacchetto parte con ttl = MESH_TTL e può essere ritrasmesso da altri apparati finché ttl > 0
#define MESH_TTL 2
#define MESH_RELAY_MIN_MS 300    // attesa casuale prima di ritrasmettere, per non sovrapporsi agli altri
#define MESH_RELAY_MAX_MS 1500
#define MESH_SEEN_SIZE 24
#define MESH_SEEN_EXPIRE_MS 60000UL
#define MESH_RELAY_SIZE 4

struct __attribute__((packed)) TrafficPacket {
  uint8_t magic[2];   // 'N','P'
  uint8_t version;    // 1
  uint8_t ttl;        // ritrasmissioni ancora permesse
  uint32_t id;        // identificativo dell'apparato di origine (dal MAC)
  uint8_t seq;        // numero progressivo del pacchetto, per riconoscere le copie
  uint8_t flags;      // bit 0 = barca simulata, bit 1-2 = tipo di nodo, bit 3-5 = nodo che dorme
  int32_t lat, lon;   // gradi * 1e7
  uint16_t cog;       // decimi di grado
  uint16_t sog;       // centesimi di nodo
  char name[BOAT_NAME_LEN];
};

struct MeshSeenEntry {
  uint32_t id = 0;
  uint8_t seq = 0;
  unsigned long time = 0;
  bool used = false;
};

struct MeshRelayEntry {
  TrafficPacket pkt;
  unsigned long due = 0;
  bool used = false;
};

static MeshSeenEntry meshSeenTable[MESH_SEEN_SIZE];
static MeshRelayEntry meshRelayQueue[MESH_RELAY_SIZE];

// Vero se questo pacchetto (o uno più recente della stessa barca) è già arrivato; altrimenti lo annota
static inline bool meshAlreadySeen(uint32_t id, uint8_t seq) {
  unsigned long now = millis();
  MeshSeenEntry* slot = nullptr;
  for(auto& e : meshSeenTable) {
    if(e.used && e.id == id) {
      if(now - e.time < MESH_SEEN_EXPIRE_MS && (uint8_t)(e.seq - seq) < 16) return true;
      slot = &e;
      break;
    }
  }
  if(!slot) {
    // Posto libero, oppure il più vecchio
    slot = &meshSeenTable[0];
    for(auto& e : meshSeenTable) {
      if(!e.used) { slot = &e; break; }
      if(e.time < slot->time) slot = &e;
    }
  }
  slot->used = true;
  slot->id = id;
  slot->seq = seq;
  slot->time = now;
  return false;
}

// Qualcun altro ha già ritrasmesso questo pacchetto: non serve farlo anche noi
static inline void meshCancelRelay(uint32_t id, uint8_t seq) {
  for(auto& r : meshRelayQueue) {
    if(r.used && r.pkt.id == id && r.pkt.seq == seq) r.used = false;
  }
}

// Mette in coda la ritrasmissione, se il pacchetto ha ancora ttl
static inline void meshQueueRelay(const TrafficPacket& pkt) {
  if(pkt.ttl == 0) return;
  for(auto& r : meshRelayQueue) {
    if(!r.used) {
      r.pkt = pkt;
      r.pkt.ttl = pkt.ttl - 1;
      r.due = millis() + random(MESH_RELAY_MIN_MS, MESH_RELAY_MAX_MS);
      r.used = true;
      return;
    }
  }
}

// Vero se c'è una ritrasmissione da fare adesso; la copia in "out" e la toglie dalla coda
static inline bool meshNextRelay(TrafficPacket& out) {
  for(auto& r : meshRelayQueue) {
    if(r.used && (long)(millis() - r.due) >= 0) {
      out = r.pkt;
      r.used = false;
      return true;
    }
  }
  return false;
}
