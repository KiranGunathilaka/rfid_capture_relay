/*
 BridgeReceiverDatabase.ino
 Role: ESP32 "bridge" node between multiple ESP-NOW RFID readers and a Raspberry Pi DB checker.

 - Receives ESP-NOW rfid_msg_t from many nodes
 - De-obfuscates UID (XOR with (SECRET_KEY & 0xFF))
 - Queues requests and forwards to Raspberry Pi over Serial in a small framed protocol
 - Receives responses (status_msg_t + MAC) from Raspberry Pi
 - Fills auth_key ON THE ESP (not the Pi) using per-MAC last UID first char
 - Sends status_msg_t back via ESP-NOW to the correct peer MAC

 Wiring:
   ESP32 <-> Raspberry Pi over UART (Serial)
   Set SERIAL_BAUD to match your RPi side (default 115200)

 Notes:
   - Uses dynamic ESP-NOW peer add when a new MAC is seen.
   - Keeps a tiny map MAC -> last_uid_first_char to compute auth_key.
   - Simple length-prefixed binary frame on UART:
        SOF(0xAA) | LEN_LO | LEN_HI | TYPE | CHECKSUM | PAYLOAD(len)
     TYPE=0x01 (REQ to Pi), payload = src_mac[6] + uid[64]
     TYPE=0x02 (RESP from Pi), payload = dst_mac[6] + status_msg_t (packed)
   - checksum = (TYPE + sum(payload)) & 0xFF

 Compile:
   Board: ESP32 Dev Module (Arduino core)
   Requires: <WiFi.h>, <esp_now.h>
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

// ====== Shared structs (match your IDF side) ======
typedef struct {
  char uid[64]; // RFID string (on-wire from readers is XOR-obfuscated)
} rfid_msg_t;

typedef struct {
  uint8_t  status;      // 0 = denied, 1 = granted
  uint32_t timestamp;   // UNIX epoch or millis
  uint8_t  event_type;  // ENTRY/EXIT/ERROR etc.
  char     ticket_id[16];
  char     name[32];
  uint32_t auth_key;    // FILLED BY ESP BRIDGE (not RPi)
} status_msg_t;

// SECRET_KEY must match your readers
#define SECRET_KEY 0xA5A5F00D

// ====== UART protocol ======
static const uint8_t SOF = 0xAA;
static const uint8_t FT_REQ  = 0x01; // ESP->RPi
static const uint8_t FT_RESP = 0x02; // RPi->ESP

// Request payload to Pi
typedef struct {
  uint8_t mac[6];       // source reader MAC
  char    uid[64];      // plain (de-obfuscated) UID, NUL-terminated
} __attribute__((packed)) PiReqPayload;

// Response payload from Pi
typedef struct {
  uint8_t     mac[6];   // destination reader MAC
  status_msg_t status;  // Pi fills everything EXCEPT auth_key; ESP will fill auth_key before re-send
} __attribute__((packed)) PiRespPayload;

// ====== Queues ======
template <typename T, size_t CAP>
class Ring {
public:
  bool push(const T& item) {
    size_t next = (head_ + 1) % CAP;
    if (next == tail_) return false; // full
    buf_[head_] = item;
    head_ = next;
    return true;
  }
  bool pop(T& out) {
    if (empty()) return false;
    out = buf_[tail_];
    tail_ = (tail_ + 1) % CAP;
    return true;
  }
  bool empty() const { return head_ == tail_; }
  size_t size() const { return (head_ + CAP - tail_) % CAP; }
private:
  volatile size_t head_{0}, tail_{0};
  T buf_[CAP];
};

typedef struct {
  uint8_t mac[6];
  char    uid[64]; // plain
} ToPiItem;

typedef struct {
  uint8_t     mac[6];
  status_msg_t status;  // auth_key will be set before sending
} ToEspItem;

// Queues
static Ring<ToPiItem, 32>  toPiQ;
static Ring<ToEspItem, 32> toEspQ;

// ====== MAC -> last UID first-char map (for auth_key) ======
typedef struct {
  uint8_t mac[6];
  uint8_t first_char; // plain (de-obfuscated) UID[0]
  bool in_use;
} PeerState;

static PeerState peers[24]; // up to 24 distinct readers

static bool macEqual(const uint8_t* a, const uint8_t* b) {
  for (int i=0;i<6;i++) if (a[i]!=b[i]) return false;
  return true;
}

static int findPeerIndex(const uint8_t* mac) {
  for (int i=0;i<(int)(sizeof(peers)/sizeof(peers[0])); i++) {
    if (peers[i].in_use && macEqual(peers[i].mac, mac)) return i;
  }
  return -1;
}

static int findOrAddPeer(const uint8_t* mac) {
  int idx = findPeerIndex(mac);
  if (idx >= 0) return idx;
  // add new
  for (int i=0;i<(int)(sizeof(peers)/sizeof(peers[0])); i++) {
    if (!peers[i].in_use) {
      memcpy(peers[i].mac, mac, 6);
      peers[i].first_char = 0;
      peers[i].in_use = true;
      return i;
    }
  }
  return -1; // table full
}

// ====== ESP-NOW helpers ======
static bool ensurePeerAdded(const uint8_t* mac) {
  // check if already known to esp-now (Arduino core reuses peer if exists)
  esp_now_peer_info_t info{};
  memcpy(info.peer_addr, mac, 6);
  info.channel = 0;       // current channel
  info.encrypt = false;   // no encryption in this sample
  // Try add; if already added, it will fail with ESP_ERR_ESPNOW_EXIST; that's OK
  esp_err_t err = esp_now_add_peer(&info);
  if (err == ESP_ERR_ESPNOW_EXIST) return true;
  return err == ESP_OK;
}

// ====== UART framing ======
static uint8_t checksum8(uint8_t type, const uint8_t* payload, uint16_t len) {
  uint32_t s = type;
  for (uint16_t i=0;i<len;i++) s += payload[i];
  return (uint8_t)(s & 0xFF);
}

static void sendFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
  uint8_t cks = checksum8(type, payload, len);
  Serial.write(SOF);
  Serial.write((uint8_t)(len & 0xFF));
  Serial.write((uint8_t)((len >> 8) & 0xFF));
  Serial.write(type);
  Serial.write(cks);
  Serial.write(payload, len);
}

// UART parser state
enum RxState { RX_SOF, RX_LEN1, RX_LEN2, RX_TYPE, RX_CKS, RX_PAYLOAD };
static RxState rxState = RX_SOF;
static uint16_t rxLen = 0, rxGot = 0;
static uint8_t rxType = 0, rxCks = 0;
static uint8_t rxBuf[6 + sizeof(status_msg_t)]; // fits the largest payload we expect

// ====== Config ======
#define SERIAL_BAUD 115200

// ====== De-obfuscate UID in-place ======
static void deobfuscate_uid(char* uid) {
  uint8_t key = (uint8_t)(SECRET_KEY & 0xFF);
  for (size_t i=0; i<strlen(uid); i++) uid[i] ^= key;
}

// ====== ESP-NOW callbacks ======
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // Optional: debug
  // Serial.printf("[ESP-NOW] Sent to %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
  //   mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5],
  //   status==ESP_NOW_SEND_SUCCESS?"OK":"FAIL");
}

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  if (len < (int)sizeof(rfid_msg_t)) return;

  rfid_msg_t msg{};
  memcpy(&msg, incomingData, sizeof(msg));

  const uint8_t* mac = info ? info->src_addr : nullptr;
  // Decrypt (simple XOR LSB)
  // We need a working copy because msg.uid is obfuscated on-wire
  char plain[64]; memset(plain, 0, sizeof(plain));
  strncpy(plain, msg.uid, sizeof(plain)-1);
  deobfuscate_uid(plain);

  // Track first char for auth_key generation later
  int pidx = findOrAddPeer(mac);
  if (pidx >= 0) {
    peers[pidx].first_char = (uint8_t)(plain[0]); // may be 0 if empty
  }

  // Enqueue for Raspberry Pi
  ToPiItem item{};
  memcpy(item.mac, mac, 6);
  strncpy(item.uid, plain, sizeof(item.uid)-1);
  if (!toPiQ.push(item)) {
    // queue full; drop oldest by popping once (optional strategy), then push again
    ToPiItem dump;
    toPiQ.pop(dump);
    toPiQ.push(item);
  }
}

// ====== Pump: to Raspberry Pi over UART ======
static void pumpToPi() {
  if (toPiQ.empty()) return;

  // One-at-a-time sending (simple model); you can add pacing if needed.
  ToPiItem it;
  if (!toPiQ.pop(it)) return;

  PiReqPayload payload{};
  memcpy(payload.mac, it.mac, 6);
  strncpy(payload.uid, it.uid, sizeof(payload.uid)-1);

  sendFrame(FT_REQ, reinterpret_cast<uint8_t*>(&payload), sizeof(payload));
  Serial.printf("[DBG] To Pi: MAC=%02X:%02X:%02X:%02X:%02X:%02X UID=%s\n",
              it.mac[0], it.mac[1], it.mac[2], it.mac[3], it.mac[4], it.mac[5],
              it.uid);
}

// ====== Pump: read Raspberry Pi frames ======
static void pumpFromPi() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    switch (rxState) {
      case RX_SOF:
        if (b == SOF) { rxState = RX_LEN1; rxLen = 0; rxGot = 0; }
        break;
      case RX_LEN1:
        rxLen = b;
        rxState = RX_LEN2;
        break;
      case RX_LEN2:
        rxLen |= ((uint16_t)b) << 8;
        if (rxLen > sizeof(rxBuf)) { rxState = RX_SOF; } else { rxState = RX_TYPE; }
        break;
      case RX_TYPE:
        rxType = b;
        rxState = RX_CKS;
        break;
      case RX_CKS:
        rxCks = b;
        rxGot = 0;
        if (rxLen == 0) {
          // zero-length payload, validate checksum over TYPE only
          uint8_t calc = checksum8(rxType, nullptr, 0);
          if (calc == rxCks) {
            // handle zero-length if we ever define one
          }
          rxState = RX_SOF;
        } else {
          rxState = RX_PAYLOAD;
        }
        break;
      case RX_PAYLOAD:
        rxBuf[rxGot++] = b;
        if (rxGot >= rxLen) {
          // validate checksum
          uint8_t calc = checksum8(rxType, rxBuf, rxLen);
          if (calc == rxCks && rxType == FT_RESP && rxLen == (6 + sizeof(status_msg_t))) {
            // Valid response frame
            PiRespPayload resp{};
            memcpy(resp.mac, rxBuf, 6);
            memcpy(&resp.status, rxBuf+6, sizeof(status_msg_t));

            // Fill auth_key on ESP using the last UID first char for this MAC
            int idx = findPeerIndex(resp.mac);
            uint32_t auth = 0;
            if (idx >= 0) auth = (uint32_t)peers[idx].first_char; // reader expects this (see reader check)
            resp.status.auth_key = auth;

            // Queue to send back to the correct reader
            ToEspItem out{};
            memcpy(out.mac, resp.mac, 6);
            out.status = resp.status;
            if (!toEspQ.push(out)) {
              ToEspItem dump;
              toEspQ.pop(dump);
              toEspQ.push(out);
            }
          }
          rxState = RX_SOF;
        }
        break;
    }
  }
}

// ====== Pump: send results to readers via ESP-NOW ======
static void pumpToEsp() {
  if (toEspQ.empty()) return;

  ToEspItem it;
  if (!toEspQ.pop(it)) return;

  // Ensure peer is added
  ensurePeerAdded(it.mac);

  // Send status back
  esp_err_t err = esp_now_send(it.mac, reinterpret_cast<uint8_t*>(&it.status), sizeof(status_msg_t));
  if (err != ESP_OK) {
    // Optional: retry/backoff strategy
    // Serial.printf("[ESP-NOW] send error %d\n", (int)err);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  WiFi.mode(WIFI_STA);
  // It’s fine to stay on default channel; if your readers are fixed on a channel, set it here.

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Bridge started.");
}

void loop() {
  pumpFromPi(); // read any responses first
  pumpToPi();   // forward pending requests
  pumpToEsp();  // send results back to readers
  // Small pacing to share CPU
  delay(2);
}
