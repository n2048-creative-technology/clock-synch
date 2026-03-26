#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>

// ============================================================
// User settings
// ============================================================
static constexpr gpio_num_t LED_PIN = GPIO_NUM_2;

static constexpr uint8_t WIFI_CHANNEL = 6;          // ALL boards must use the same channel
static constexpr uint32_t BLINK_PERIOD_US = 1000000; // 1 second
static constexpr uint32_t LED_ON_US = 10000;         // 10 ms on
static constexpr uint32_t SEND_INTERVAL_US = 200000; // sync packet every 200 ms

// Sync behavior tuning
static constexpr int32_t ACQUIRE_SNAP_THRESHOLD_US = 20000;  // snap on acquisition if > 20 ms
static constexpr int32_t PHASE_SLEW_LIMIT_US = 1500;         // max phase correction per control step
static constexpr int32_t CONTROL_INTERVAL_US = 100000;       // 100 ms control loop
static constexpr int32_t PEER_TIMEOUT_US = 3000000;          // peer stale after 3 s
static constexpr int32_t LOCK_TIMEOUT_US = 4000000;          // lose lock after 4 s without peers

// Frequency trim limits
static constexpr int32_t MAX_TRIM_PPM = 300;     // max software trim
static constexpr int32_t MIN_DRIFT_DT_US = 200000;
static constexpr int32_t MAX_DRIFT_DT_US = 3000000;

// ============================================================
// Packet format
// ============================================================
static constexpr uint32_t PACKET_MAGIC = 0x534E4331; // "SNC1"
static constexpr uint8_t PACKET_VERSION = 1;

struct __attribute__((packed)) SyncPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t reserved[3];
  uint32_t bootId;
  uint32_t seq;
  uint64_t txLocalUs;
  uint64_t nextTickUs;
  int32_t trimPpm;
  uint32_t periodUs;
};

// ============================================================
// Peer state
// ============================================================
struct PeerState {
  bool used = false;
  uint8_t mac[6] = {0};

  uint32_t bootId = 0;
  uint32_t lastSeq = 0;
  int64_t lastSeenUs = 0;

  int32_t phaseErrorUs = 0;       // latest estimated phase error from this peer
  int64_t lastPhaseSampleUs = 0;  // local time when phase error was sampled
  int32_t lastPhaseErrorUs = 0;

  int32_t driftEstimatePpm = 0;   // estimated relative drift from this peer
};

static constexpr size_t MAX_PEERS_TRACKED = 32;
PeerState peers[MAX_PEERS_TRACKED];

// ============================================================
// RX queue from ESP-NOW callback to main loop
// ============================================================
struct RxEvent {
  uint8_t mac[6];
  int64_t rxUs;
  SyncPacket pkt;
};

static constexpr size_t RX_QUEUE_SIZE = 32;
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
RxEvent rxQueue[RX_QUEUE_SIZE];

portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Global timing state
// ============================================================
uint8_t selfMac[6] = {0};
uint32_t bootId = 0;
uint32_t txSeq = 0;

// Local oscillator state
int64_t nextTickUs = 0;     // next LED ON edge in local clock
int64_t ledOffUs = 0;
bool ledIsOn = false;

// Phase/frequency control
int32_t trimPpm = 0;        // software frequency trim, applied to local period
bool locked = false;
int64_t lastGoodPeerUs = 0;

// Scheduling
int64_t lastSendUs = 0;
int64_t lastControlUs = 0;
int64_t lastStatusUs = 0;

// Fixed-point period accumulator: Q16.16 microseconds
int64_t nominalPeriodQ16 = ((int64_t)BLINK_PERIOD_US) << 16;
int64_t periodStepQ16 = ((int64_t)BLINK_PERIOD_US) << 16;
int64_t fracAccumulatorQ16 = 0;

// ============================================================
// Utility
// ============================================================
static inline int64_t nowUs() {
  return esp_timer_get_time();
}

static void macToString(const uint8_t *mac, char *out, size_t len) {
  snprintf(out, len, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int64_t clamp64(int64_t v, int64_t lo, int64_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void updatePeriodStepFromTrim() {
  // periodStepQ16 = nominalPeriod * (1 - trimPpm / 1e6)
  // positive trimPpm means "I am slow, tick a little sooner"
  periodStepQ16 = nominalPeriodQ16 - ((nominalPeriodQ16 * (int64_t)trimPpm) / 1000000LL);
}

static int32_t medianInt32(int32_t *vals, int count) {
  for (int i = 1; i < count; ++i) {
    int32_t key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      --j;
    }
    vals[j + 1] = key;
  }

  if (count % 2 == 1) {
    return vals[count / 2];
  }
  return (vals[count / 2 - 1] + vals[count / 2]) / 2;
}

static int32_t getEffectivePeriodUsInteger() {
  return (int32_t)(periodStepQ16 >> 16);
}

static void advanceNextTickOnePeriod() {
  fracAccumulatorQ16 += periodStepQ16;
  int64_t wholeUs = fracAccumulatorQ16 >> 16;
  fracAccumulatorQ16 &= 0xFFFF;
  nextTickUs += wholeUs;
}

static int findPeerIndex(const uint8_t *mac) {
  for (size_t i = 0; i < MAX_PEERS_TRACKED; ++i) {
    if (peers[i].used && sameMac(peers[i].mac, mac)) {
      return (int)i;
    }
  }
  return -1;
}

static int getOrCreatePeerIndex(const uint8_t *mac) {
  int idx = findPeerIndex(mac);
  if (idx >= 0) return idx;

  for (size_t i = 0; i < MAX_PEERS_TRACKED; ++i) {
    if (!peers[i].used) {
      peers[i].used = true;
      memcpy(peers[i].mac, mac, 6);
      peers[i].bootId = 0;
      peers[i].lastSeq = 0;
      peers[i].lastSeenUs = 0;
      peers[i].phaseErrorUs = 0;
      peers[i].lastPhaseSampleUs = 0;
      peers[i].lastPhaseErrorUs = 0;
      peers[i].driftEstimatePpm = 0;
      return (int)i;
    }
  }
  return -1;
}

// ============================================================
// ESP-NOW callbacks
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len) {
  if (!recvInfo || !data || len != (int)sizeof(SyncPacket)) return;

  SyncPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != PACKET_MAGIC || pkt.version != PACKET_VERSION) return;

  uint8_t next = (rxHead + 1) % RX_QUEUE_SIZE;
  if (next == rxTail) return; // drop if full

  portENTER_CRITICAL_ISR(&rxMux);
  memcpy(rxQueue[rxHead].mac, recvInfo->src_addr, 6);
  rxQueue[rxHead].rxUs = esp_timer_get_time();
  rxQueue[rxHead].pkt = pkt;
  rxHead = next;
  portEXIT_CRITICAL_ISR(&rxMux);
}
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (!mac || !data || len != (int)sizeof(SyncPacket)) return;

  SyncPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != PACKET_MAGIC || pkt.version != PACKET_VERSION) return;

  uint8_t next = (rxHead + 1) % RX_QUEUE_SIZE;
  if (next == rxTail) return; // drop if full

  portENTER_CRITICAL_ISR(&rxMux);
  memcpy(rxQueue[rxHead].mac, mac, 6);
  rxQueue[rxHead].rxUs = esp_timer_get_time();
  rxQueue[rxHead].pkt = pkt;
  rxHead = next;
  portEXIT_CRITICAL_ISR(&rxMux);
}
#endif

void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  (void)macAddr;
  (void)status;
}

// ============================================================
// Networking
// ============================================================
static void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  esp_err_t err = esp_wifi_set_promiscuous(true);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_promiscuous(true) failed: %d\n", err);
  }

  err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_channel failed: %d\n", err);
  }

  err = esp_wifi_set_promiscuous(false);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_promiscuous(false) failed: %d\n", err);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memset(peerInfo.peer_addr, 0xFF, 6); // broadcast
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(peerInfo.peer_addr)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add broadcast peer");
      while (true) {
        delay(1000);
      }
    }
  }
}

static void sendSyncPacket() {
  SyncPacket pkt = {};
  pkt.magic = PACKET_MAGIC;
  pkt.version = PACKET_VERSION;
  pkt.bootId = bootId;
  pkt.seq = ++txSeq;
  pkt.txLocalUs = (uint64_t)nowUs();
  pkt.nextTickUs = (uint64_t)nextTickUs;
  pkt.trimPpm = trimPpm;
  pkt.periodUs = BLINK_PERIOD_US;

  const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_err_t err = esp_now_send(bcast, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
  if (err != ESP_OK) {
    Serial.printf("esp_now_send failed: %d\n", err);
  }
}

// ============================================================
// RX processing
// ============================================================
static bool popRxEvent(RxEvent &ev) {
  bool hasData = false;

  portENTER_CRITICAL(&rxMux);
  if (rxTail != rxHead) {
    ev = rxQueue[rxTail];
    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    hasData = true;
  }
  portEXIT_CRITICAL(&rxMux);

  return hasData;
}

static void processRxEvent(const RxEvent &ev) {
  if (sameMac(ev.mac, selfMac)) {
    return;
  }

  int idx = getOrCreatePeerIndex(ev.mac);
  if (idx < 0) return;

  PeerState &p = peers[idx];

  // Reset sequence continuity if peer rebooted
  if (p.bootId != ev.pkt.bootId) {
    p.bootId = ev.pkt.bootId;
    p.lastSeq = 0;
    p.lastPhaseSampleUs = 0;
    p.lastPhaseErrorUs = 0;
    p.driftEstimatePpm = 0;
  }

  // Ignore stale or repeated packets from same boot session
  if (p.lastSeq != 0 && ev.pkt.seq <= p.lastSeq) {
    return;
  }
  p.lastSeq = ev.pkt.seq;
  p.lastSeenUs = ev.rxUs;

  // Estimate when sender's next tick will occur, expressed in our local time.
  // senderRemainingUs = senderNextTick - senderTxTime
  // estimatedSenderNextTickLocal = ourRxTime + senderRemainingUs
  // phaseError = targetNextTick - ourNextTick
  int64_t senderRemainingUs = (int64_t)ev.pkt.nextTickUs - (int64_t)ev.pkt.txLocalUs;

  // Reject obviously broken packets
  if (senderRemainingUs < -50000 || senderRemainingUs > (int64_t)BLINK_PERIOD_US + 50000) {
    return;
  }

  int64_t estimatedSenderNextTickLocal = ev.rxUs + senderRemainingUs;
  int64_t phaseErr64 = estimatedSenderNextTickLocal - nextTickUs;
  int32_t phaseErrUs = (int32_t)clamp64(phaseErr64, -200000, 200000);

  p.phaseErrorUs = phaseErrUs;

  // Estimate relative drift in ppm from phase error slope
  if (p.lastPhaseSampleUs > 0) {
    int64_t dt = ev.rxUs - p.lastPhaseSampleUs;
    if (dt >= MIN_DRIFT_DT_US && dt <= MAX_DRIFT_DT_US) {
      int64_t dErr = (int64_t)phaseErrUs - (int64_t)p.lastPhaseErrorUs;

      // ppm ≈ dErr / dt * 1e6
      int64_t ppm64 = (dErr * 1000000LL) / dt;
      p.driftEstimatePpm = (int32_t)clamp64(ppm64, -1000, 1000);
    }
  }

  p.lastPhaseSampleUs = ev.rxUs;
  p.lastPhaseErrorUs = phaseErrUs;
}

// ============================================================
// Control loop
// ============================================================
static void applyControl() {
  int64_t now = nowUs();

  int32_t phaseErrors[MAX_PEERS_TRACKED];
  int32_t driftPpms[MAX_PEERS_TRACKED];
  int phaseCount = 0;
  int driftCount = 0;

  for (size_t i = 0; i < MAX_PEERS_TRACKED; ++i) {
    if (!peers[i].used) continue;
    if ((now - peers[i].lastSeenUs) > PEER_TIMEOUT_US) continue;

    phaseErrors[phaseCount++] = peers[i].phaseErrorUs;

    if (peers[i].lastPhaseSampleUs > 0) {
      driftPpms[driftCount++] = peers[i].driftEstimatePpm;
    }
  }

  if (phaseCount == 0) {
    if ((now - lastGoodPeerUs) > LOCK_TIMEOUT_US) {
      locked = false;
    }
    return;
  }

  lastGoodPeerUs = now;

  int32_t medianPhaseUs = medianInt32(phaseErrors, phaseCount);
  int32_t medianDriftPpm = 0;
  if (driftCount > 0) {
    medianDriftPpm = medianInt32(driftPpms, driftCount);
  }

  if (!locked) {
    if (abs(medianPhaseUs) > ACQUIRE_SNAP_THRESHOLD_US) {
      nextTickUs += medianPhaseUs;
    } else {
      nextTickUs += medianPhaseUs / 2;
    }
    locked = true;
  } else {
    int32_t phaseAdjustUs = clamp32(medianPhaseUs / 6, -PHASE_SLEW_LIMIT_US, PHASE_SLEW_LIMIT_US);
    nextTickUs += phaseAdjustUs;
  }

  // Drift control:
  // If phase error grows positive over time, our next tick is too late, so we need a positive trim.
  // Use a very slow I-like update from peer-consensus slope.
  trimPpm += clamp32(medianDriftPpm / 8, -2, 2);
  trimPpm = clamp32(trimPpm, -MAX_TRIM_PPM, MAX_TRIM_PPM);
  updatePeriodStepFromTrim();
}

// ============================================================
// LED/timing engine
// ============================================================
static void updateBlinkEngine() {
  int64_t now = nowUs();

  // Turn LED off after the pulse duration
  if (ledIsOn && now >= ledOffUs) {
    digitalWrite(LED_PIN, LOW);
    ledIsOn = false;
  }

  // Catch up if multiple periods elapsed
  while (now >= nextTickUs) {
    digitalWrite(LED_PIN, HIGH);
    ledIsOn = true;
    ledOffUs = nextTickUs + LED_ON_US;
    advanceNextTickOnePeriod();
  }
}

// ============================================================
// Status
// ============================================================
static void printStatus() {
  int64_t now = nowUs();

  int activePeers = 0;
  for (size_t i = 0; i < MAX_PEERS_TRACKED; ++i) {
    if (!peers[i].used) continue;
    if ((now - peers[i].lastSeenUs) <= PEER_TIMEOUT_US) {
      activePeers++;
    }
  }

  Serial.printf(
      "t=%lld ms | next=%lld | trim=%ld ppm | lock=%s | peers=%d | period=%ld us\n",
      now / 1000,
      nextTickUs,
      (long)trimPpm,
      locked ? "yes" : "no",
      activePeers,
      (long)getEffectivePeriodUsInteger());
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  bootId = esp_random();

  WiFi.mode(WIFI_STA);
  WiFi.macAddress(selfMac);

  char macStr[18];
  macToString(selfMac, macStr, sizeof(macStr));

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32 distributed LED sync");
  Serial.printf("MAC: %s\n", macStr);
  Serial.printf("Boot ID: %lu\n", (unsigned long)bootId);
  Serial.printf("WiFi channel: %u\n", WIFI_CHANNEL);
  Serial.println("========================================");

  initEspNow();

  int64_t now = nowUs();

  // Start ticking immediately, even with no peers
  fracAccumulatorQ16 = 0;
  trimPpm = 0;
  updatePeriodStepFromTrim();

  // Start from the next clean 1-second boundary in local time
  nextTickUs = ((now / BLINK_PERIOD_US) + 1) * (int64_t)BLINK_PERIOD_US;
  ledOffUs = nextTickUs + LED_ON_US;

  lastSendUs = now;
  lastControlUs = now;
  lastStatusUs = now;
  lastGoodPeerUs = 0;
  locked = false;
}

void loop() {
  updateBlinkEngine();

  RxEvent ev;
  while (popRxEvent(ev)) {
    processRxEvent(ev);
  }

  int64_t now = nowUs();

  if ((now - lastControlUs) >= CONTROL_INTERVAL_US) {
    lastControlUs += CONTROL_INTERVAL_US;
    applyControl();
  }

  if ((now - lastSendUs) >= SEND_INTERVAL_US) {
    lastSendUs += SEND_INTERVAL_US;
    sendSyncPacket();
  }

  if ((now - lastStatusUs) >= 1000000) {
    lastStatusUs += 1000000;
    printStatus();
  }

  delay(1);
}