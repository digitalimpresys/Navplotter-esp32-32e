// Short text messages between NavPlotters over LoRa ("chat"), next to the position packets.
// This file MUST be identical in every NavPlotter that has messages.
//
// A message goes to one node or to all. A message to one node is repeated until that node says it has
// received it, and the sender is told again when somebody there has read it. Messages travel through the
// mesh like the positions do. Nothing here is guaranteed: a packet can be lost, and only nodes with this
// firmware, switched on and in range, receive anything.
#pragma once
#include <Arduino.h>
#include <vector>
#include "traffic_packet.h"

#define MSG_TEXT_LEN 64          // 63 characters + terminator
#define MSG_KIND_TEXT 0
#define MSG_KIND_RECEIVED 1      // "your message number ref has arrived here"
#define MSG_KIND_READ 2          // "somebody here has read your message number ref"
#define MSG_RETRY_MS 7000UL      // a message to one node is sent again this often...
#define MSG_TRIES 4              // ...this many times at most, until it is confirmed
#define MSG_KEEP 12              // messages kept in memory
#define MSG_DEDUP_MARK 0x4D534721UL // keeps message numbers apart from position numbers in the mesh table

struct __attribute__((packed)) MessagePacket {
  uint8_t magic[2];   // 'N','M'
  uint8_t version;    // 1
  uint8_t ttl;        // relays still allowed
  uint32_t id;        // sender
  uint8_t seq;        // number of this transmission, to tell copies apart in the mesh
  uint8_t kind;       // MSG_KIND_*
  uint32_t to;        // receiver, 0 = everybody
  uint8_t ref;        // text: number of the message; confirmations: number of the message confirmed
  char name[BOAT_NAME_LEN]; // sender's name
  char text[MSG_TEXT_LEN];  // only the used part is transmitted
};
#define MSG_HEADER_LEN (sizeof(MessagePacket) - MSG_TEXT_LEN)

enum MsgStatus : uint8_t {
  MSG_INCOMING,    // received from somebody else
  MSG_PENDING,     // ours, being sent
  MSG_SENT,        // ours, to everybody: sent, no confirmation is asked
  MSG_DELIVERED,   // ours, arrived at the other node
  MSG_READ,        // ours, read by somebody there
  MSG_NO_ANSWER    // ours, never confirmed
};

struct ChatMessage {
  bool mine = false;
  bool toAll = false;
  bool unread = false;            // incoming, not yet shown to anybody
  uint32_t peer = 0;              // the other node (for ours to everybody: 0)
  char peerName[BOAT_NAME_LEN] = "";
  char text[MSG_TEXT_LEN] = "";
  uint8_t number = 0;             // message number, given by its sender
  MsgStatus status = MSG_INCOMING;
  uint8_t tries = 0;
  unsigned long nextTry = 0;
  unsigned long time = 0;         // millis() when written or received
};

// What msgReceive() reports
#define MSG_EVENT_NEW 1      // a new message for us: it is the last one in chatMessages
#define MSG_EVENT_STATUS 2   // one of our messages changed status

struct MsgQueued {
  MessagePacket pkt;
  uint8_t len = 0;
  unsigned long due = 0;
  bool used = false;
};

static std::vector<ChatMessage> chatMessages;
static MsgQueued msgControlQueue[6]; // confirmations waiting to go out
static MsgQueued msgRelayQueue[3];   // other nodes' packets waiting to be relayed
static uint8_t msgNextSeq = 0, msgNextNumber = 0;
static bool msgCountersSet = false;

static inline void msgInitCounters() {
  if(msgCountersSet) return;
  msgCountersSet = true;
  msgNextSeq = random(0, 256);
  msgNextNumber = random(0, 256);
}

// Only plain printable characters travel
static inline void msgCleanText(char* text) {
  for(char* c = text; *c; c++) if(*c < ' ' || *c > '~') *c = '?';
}

static inline void msgFillHeader(MessagePacket& pkt, uint32_t myId, const char* myName, uint8_t kind, uint32_t to, uint8_t ref) {
  msgInitCounters();
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic[0] = 'N'; pkt.magic[1] = 'M';
  pkt.version = 1;
  pkt.ttl = MESH_TTL;
  pkt.id = myId;
  pkt.seq = msgNextSeq++;
  pkt.kind = kind;
  pkt.to = to;
  pkt.ref = ref;
  strncpy(pkt.name, myName, BOAT_NAME_LEN - 1);
}

static inline void msgQueueControl(uint32_t myId, const char* myName, uint8_t kind, uint32_t to, uint8_t ref, unsigned long delayMs) {
  MsgQueued* slot = &msgControlQueue[0];
  for(auto& q : msgControlQueue) if(!q.used) { slot = &q; break; }
  msgFillHeader(slot->pkt, myId, myName, kind, to, ref);
  slot->len = MSG_HEADER_LEN + 1;
  slot->due = millis() + delayMs;
  slot->used = true;
}

static inline void msgTrimStore() {
  while(chatMessages.size() > MSG_KEEP) chatMessages.erase(chatMessages.begin());
}

// Writes a message: to one node (to != 0) or to everybody (to == 0). It leaves with msgNextPacket().
static inline void msgSend(uint32_t to, const char* toName, const char* text) {
  msgInitCounters();
  ChatMessage m;
  m.mine = true;
  m.toAll = to == 0;
  m.peer = to;
  strncpy(m.peerName, toName, BOAT_NAME_LEN - 1);
  strncpy(m.text, text, MSG_TEXT_LEN - 1);
  msgCleanText(m.text);
  m.number = msgNextNumber++;
  m.status = MSG_PENDING;
  m.nextTry = millis();
  m.time = millis();
  chatMessages.push_back(m);
  msgTrimStore();
}

// The radio is free: is there a message packet to transmit? Fills buf (at least sizeof(MessagePacket)) and len.
static inline bool msgNextPacket(uint32_t myId, const char* myName, uint8_t* buf, size_t& len) {
  unsigned long now = millis();
  // Confirmations first: somebody is waiting for them
  for(auto& q : msgControlQueue) {
    if(q.used && (long)(now - q.due) >= 0) {
      memcpy(buf, &q.pkt, q.len); len = q.len; q.used = false;
      return true;
    }
  }
  for(auto& m : chatMessages) {
    if(!m.mine || m.status != MSG_PENDING || (long)(now - m.nextTry) < 0) continue;
    // To everybody: twice, then done. To one node: until confirmed, or given up.
    if(m.tries >= (m.toAll ? 2 : MSG_TRIES)) { m.status = m.toAll ? MSG_SENT : MSG_NO_ANSWER; continue; }
    MessagePacket pkt;
    msgFillHeader(pkt, myId, myName, MSG_KIND_TEXT, m.peer, m.number);
    strncpy(pkt.text, m.text, MSG_TEXT_LEN - 1);
    len = MSG_HEADER_LEN + strlen(pkt.text) + 1;
    memcpy(buf, &pkt, len);
    m.tries++;
    m.nextTry = now + MSG_RETRY_MS + random(0, 1500);
    return true;
  }
  for(auto& q : msgRelayQueue) {
    if(q.used && (long)(now - q.due) >= 0) {
      memcpy(buf, &q.pkt, q.len); len = q.len; q.used = false;
      return true;
    }
  }
  return false;
}

// True while one of our messages still has to change status by itself (so the screen should be refreshed)
static inline bool msgBusy() {
  for(const auto& m : chatMessages) if(m.mine && m.status == MSG_PENDING) return true;
  return false;
}

// A packet starting with 'N','M' has arrived. Returns MSG_EVENT_* bits.
static inline int msgReceive(uint32_t myId, const char* myName, const uint8_t* data, size_t len) {
  if(len < MSG_HEADER_LEN + 1 || len > sizeof(MessagePacket)) return 0;
  MessagePacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  memcpy(&pkt, data, len);
  pkt.name[BOAT_NAME_LEN - 1] = '\0';
  pkt.text[MSG_TEXT_LEN - 1] = '\0';
  if(pkt.magic[0] != 'N' || pkt.magic[1] != 'M' || pkt.version != 1 || pkt.id == myId) return 0;
  bool forMe = pkt.to == myId, forAll = pkt.to == 0;
  int events = 0;

  // The same transmission heard again (directly and through a relay): only a text addressed to us is looked
  // at twice, because its sender may be repeating it for want of our confirmation
  bool seen = meshAlreadySeen(pkt.id ^ MSG_DEDUP_MARK, pkt.seq);

  if(pkt.kind == MSG_KIND_TEXT && (forMe || forAll)) {
    ChatMessage* known = nullptr;
    for(auto& m : chatMessages) if(!m.mine && m.peer == pkt.id && m.number == pkt.ref) known = &m;
    if(!known) {
      ChatMessage m;
      m.toAll = forAll;
      m.unread = true;
      m.peer = pkt.id;
      strncpy(m.peerName, pkt.name, BOAT_NAME_LEN - 1);
      msgCleanText(m.peerName);
      strncpy(m.text, pkt.text, MSG_TEXT_LEN - 1);
      msgCleanText(m.text);
      m.number = pkt.ref;
      m.status = MSG_INCOMING;
      m.time = millis();
      chatMessages.push_back(m);
      msgTrimStore();
      events |= MSG_EVENT_NEW;
      if(forMe) msgQueueControl(myId, myName, MSG_KIND_RECEIVED, pkt.id, pkt.ref, random(200, 600));
    } else if(forMe && !seen) {
      // A repeat: our confirmation got lost, say it again
      msgQueueControl(myId, myName, known->unread ? MSG_KIND_RECEIVED : MSG_KIND_READ, pkt.id, pkt.ref, random(200, 600));
    }
  } else if((pkt.kind == MSG_KIND_RECEIVED || pkt.kind == MSG_KIND_READ) && forMe) {
    for(auto& m : chatMessages) {
      if(!m.mine || m.toAll || m.peer != pkt.id || m.number != pkt.ref) continue;
      MsgStatus s = pkt.kind == MSG_KIND_READ ? MSG_READ : MSG_DELIVERED;
      if(m.status != MSG_READ && m.status != s) { m.status = s; events |= MSG_EVENT_STATUS; }
    }
  }

  // Mesh: pass on what is not for us alone
  if(!seen && !forMe && pkt.ttl > 0) {
    for(auto& q : msgRelayQueue) {
      if(q.used) continue;
      q.pkt = pkt;
      q.pkt.ttl--;
      q.len = len;
      q.due = millis() + random(MESH_RELAY_MIN_MS, MESH_RELAY_MAX_MS);
      q.used = true;
      break;
    }
  }
  return events;
}

// Somebody has seen incoming message number index: the sender of a message addressed to us is told (twice,
// a few seconds apart, because this confirmation is not repeated on request)
static inline void msgMarkRead(uint32_t myId, const char* myName, size_t index) {
  if(index >= chatMessages.size()) return;
  ChatMessage& m = chatMessages[index];
  if(m.mine || !m.unread) return;
  m.unread = false;
  if(m.toAll) return;
  msgQueueControl(myId, myName, MSG_KIND_READ, m.peer, m.number, 100);
  msgQueueControl(myId, myName, MSG_KIND_READ, m.peer, m.number, 4000);
}

static inline int msgUnreadCount() {
  int n = 0;
  for(const auto& m : chatMessages) if(m.unread) n++;
  return n;
}

// ---- For the web app ----

// A number that changes whenever anything in the message list changes: the web app asks for the list
// only then
static inline uint32_t msgStamp() {
  uint32_t h = chatMessages.size();
  for(const auto& m : chatMessages) h = h * 31 + m.number * 7 + (uint8_t)m.status * 3 + m.tries + (m.unread ? 1 : 0) + m.peer;
  return h;
}

// The message list as JSON. Texts and names hold only plain printable characters.
static inline String msgListJson() {
  String json = "{\"stamp\":" + String(msgStamp()) + ",\"messages\":[";
  bool first = true;
  for(const auto& m : chatMessages) {
    char head[96];
    snprintf(head, sizeof(head), "%s{\"mine\":%s,\"all\":%s,\"peer\":\"%08lX\",\"status\":%u,\"unread\":%s,\"age\":%lu,\"name\":\"",
      first ? "" : ",", m.mine ? "true" : "false", m.toAll ? "true" : "false", (unsigned long)m.peer, (unsigned)m.status,
      m.unread ? "true" : "false", (millis() - m.time) / 1000);
    json += head;
    for(const char* c = m.peerName; *c; c++) { if(*c == '"' || *c == '\\') json += '\\'; json += *c; }
    json += "\",\"text\":\"";
    for(const char* c = m.text; *c; c++) { if(*c == '"' || *c == '\\') json += '\\'; json += *c; }
    json += "\"}";
    first = false;
  }
  json += "]}";
  return json;
}

// Marks as read the incoming messages of one conversation: those addressed to us by node peer, or (all)
// those sent to everybody. Returns how many.
static inline int msgMarkConversationRead(uint32_t myId, const char* myName, uint32_t peer, bool all) {
  int n = 0;
  for(size_t i = 0; i < chatMessages.size(); i++) {
    const ChatMessage& m = chatMessages[i];
    if(m.mine || !m.unread || m.toAll != all || (!all && m.peer != peer)) continue;
    msgMarkRead(myId, myName, i);
    n++;
  }
  return n;
}
