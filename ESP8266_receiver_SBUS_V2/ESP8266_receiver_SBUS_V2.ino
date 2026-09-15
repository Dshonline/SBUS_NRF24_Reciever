/* NRF24 Receiver with SBUS output — ESP8266 (NodeMCU) version
 *
 * Receives 7 channels over NRF24L01 and outputs a valid SBUS frame
 * on Serial1 TX (GPIO2 / D4) at 100000 baud, 8E2.
 *
 * WHY Serial1 and not Serial?
 *   Serial  (GPIO1 / D10) is shared with USB. Using it for SBUS
 *   means you lose the serial monitor and upload becomes tricky.
 *   Serial1 (GPIO2 / D4)  is TX-only, free to use for SBUS, and
 *   leaves Serial available for debug prints during development.
 *
 * ── NRF24 wiring ──────────────────────────────────────────────
 *   NRF24   NodeMCU   GPIO
 *   GND  →  GND
 *   Vcc  →  3.3V      (ESP8266 is 3.3V native — no level shift needed)
 *   CE   →  D2        GPIO4
 *   CSN  →  D8        GPIO15
 *   CLK  →  D5        GPIO14  (hardware SPI SCK)
 *   MOSI →  D7        GPIO13  (hardware SPI MOSI)
 *   MISO →  D6        GPIO12  (hardware SPI MISO)
 *
 * ── SBUS output ───────────────────────────────────────────────
 *   SBUS signal wire → D4 (GPIO2) → transistor inverter → FC SBUS pin
 *
 *   Inverter circuit (SBUS is inverted logic — required):
 *   D4 ──[1kΩ]──► Base  of BC547 / 2N2222
 *                  Collector ──► SBUS wire to FC
 *                  Emitter   ──► GND
 *   3.3V ──[10kΩ]──► Collector  (pull-up)
 *
 * ── SBUS frame format ─────────────────────────────────────────
 *   25 bytes | 100000 baud | 8E2 | inverted signal
 *   Byte  0    : 0x0F  (header)
 *   Bytes 1-22 : 16 channels × 11 bits, bit-packed
 *   Byte  23   : flags (failsafe / signal loss bits)
 *   Byte  24   : 0x00  (footer)
 *   Frame rate : every 14 ms
 *
 *   Channel value range: 172 (min) → 992 (center) → 1811 (max)
 *
 * ── Protocol versioning ─────────────────────────────────────────
 *   PROTOCOL_VERSION must match on TX and RX. If you ever change the
 *   Received_data struct layout, bump this on both sides. A mismatch
 *   is detected and the receiver goes to failsafe instead of silently
 *   misinterpreting garbage bytes as channel data.
 *
 * ── Debug modes ───────────────────────────────────────────────
 *   DEBUG_CHANNELS : print raw channel values + link status (original behaviour)
 *   DEBUG_SBUS     : decode the just-built SBUS frame back into channel
 *                    values and print them, to verify buildSBUSFrame()
 *                    is packing bits correctly (round-trip sanity check)
 *   Both are toggled independently below — leave both off for flight use,
 *   since Serial prints cost time and can perturb the loop timing.
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP8266WiFi.h>

// ── Debug toggles ────────────────────────────────────────────
#define DEBUG_CHANNELS 0   // set to 0 to disable raw channel prints
#define DEBUG_SBUS     0   // set to 1 to enable SBUS round-trip decode print

// ── Pin definitions ───────────────────────────────────────────
#define NRF_CE  D2   // GPIO4
#define NRF_CSN D8   // GPIO15

// ── Protocol ──────────────────────────────────────────────────
#define PROTOCOL_VERSION 1   // bump on both TX and RX if struct layout changes

// ── Tunable constants (previously magic numbers) ────────────────
#define SBUS_MIN            172
#define SBUS_CENTER         992
#define SBUS_MAX            1811
#define SBUS_NUM_CHANNELS   16
#define SBUS_FRAME_BYTES    25
#define SBUS_HEADER         0x0F
#define SBUS_FOOTER         0x00
#define SBUS_FLAG_FAILSAFE  0x0C   // bit2 = signal loss, bit3 = failsafe

#define FRAME_INTERVAL_MS   14     // standard SBUS frame rate
#define FAILSAFE_TIMEOUT_MS 1000   // no packet for this long → failsafe
#define LINK_STATS_PERIOD_MS 1000  // how often to print packet-loss stats

// ── Radio ─────────────────────────────────────────────────────
const uint64_t pipeIn = 0xE8E8F0F0E1LL;  // must match transmitter
RF24 radio(NRF_CE, NRF_CSN);

// ── Data struct (must match transmitter exactly) ───────────────
struct Received_data {
  uint8_t version; // PROTOCOL_VERSION — mismatch means TX/RX are out of sync
  byte ch1;   // Throttle  (analog 0–255)
  byte ch2;   // Roll      (analog 0–255)
  byte ch3;   // Pitch     (analog 0–255)
  byte ch4;   // Yaw       (analog 0–255)
  byte ch5;   // Switch 1  (digital 0 or 1)
  byte ch6;   // Switch 2  (digital 0 or 1)
  byte ch7;   // Aux       (analog 0–255)
};

Received_data received_data;
unsigned long lastRecvTime = 0;
bool failsafe = false;

// ── Link quality tracking ────────────────────────────────────
uint32_t packetsReceived = 0;
uint32_t packetsBadVersion = 0;
uint32_t consecutiveMisses = 0;
unsigned long lastLinkStatsTime = 0;

// ── Non-blocking frame timing ────────────────────────────────
unsigned long lastFrameTime = 0;

// ── SBUS frame buffer ─────────────────────────────────────────
uint8_t sbusFrame[SBUS_FRAME_BYTES];

// Map 0–255 byte  →  SBUS min–max
int toSBUS(byte val) {
  return map(val, 0, 255, SBUS_MIN, SBUS_MAX);
}

// Map digital 0/1  →  SBUS min / max
int toSBUS_digital(byte val) {
  return val ? SBUS_MAX : SBUS_MIN;
}

void buildSBUSFrame(int ch[SBUS_NUM_CHANNELS]) {
  sbusFrame[0]  = SBUS_HEADER;

  // Pack 16 × 11-bit channel values into bytes 1–22
  sbusFrame[1]  =  (ch[0])                          & 0xFF;
  sbusFrame[2]  = ((ch[0]  >> 8) | (ch[1]  << 3))   & 0xFF;
  sbusFrame[3]  = ((ch[1]  >> 5) | (ch[2]  << 6))   & 0xFF;
  sbusFrame[4]  =  (ch[2]  >> 2)                    & 0xFF;
  sbusFrame[5]  = ((ch[2]  >> 10)| (ch[3]  << 1))   & 0xFF;
  sbusFrame[6]  = ((ch[3]  >> 7) | (ch[4]  << 4))   & 0xFF;
  sbusFrame[7]  = ((ch[4]  >> 4) | (ch[5]  << 7))   & 0xFF;
  sbusFrame[8]  =  (ch[5]  >> 1)                    & 0xFF;
  sbusFrame[9]  = ((ch[5]  >> 9) | (ch[6]  << 2))   & 0xFF;
  sbusFrame[10] = ((ch[6]  >> 6) | (ch[7]  << 5))   & 0xFF;
  sbusFrame[11] =  (ch[7]  >> 3)                    & 0xFF;
  sbusFrame[12] =  (ch[8])                          & 0xFF;
  sbusFrame[13] = ((ch[8]  >> 8) | (ch[9]  << 3))   & 0xFF;
  sbusFrame[14] = ((ch[9]  >> 5) | (ch[10] << 6))   & 0xFF;
  sbusFrame[15] =  (ch[10] >> 2)                    & 0xFF;
  sbusFrame[16] = ((ch[10] >> 10)| (ch[11] << 1))   & 0xFF;
  sbusFrame[17] = ((ch[11] >> 7) | (ch[12] << 4))   & 0xFF;
  sbusFrame[18] = ((ch[12] >> 4) | (ch[13] << 7))   & 0xFF;
  sbusFrame[19] =  (ch[13] >> 1)                    & 0xFF;
  sbusFrame[20] = ((ch[13] >> 9) | (ch[14] << 2))   & 0xFF;
  sbusFrame[21] = ((ch[14] >> 6) | (ch[15] << 5))   & 0xFF;
  sbusFrame[22] =  (ch[15] >> 3)                    & 0xFF;

  sbusFrame[23] = failsafe ? SBUS_FLAG_FAILSAFE : 0x00;
  sbusFrame[24] = SBUS_FOOTER;
}

// Decode sbusFrame[] back into 16 channel values — used only for the
// DEBUG_SBUS round-trip sanity check, to confirm buildSBUSFrame() is
// packing bits correctly. Not used in the normal control path.
void decodeSBUSFrame(uint8_t *frame, int outCh[SBUS_NUM_CHANNELS]) {
  outCh[0]  = ((frame[1]      | frame[2]  << 8)                      & 0x07FF);
  outCh[1]  = ((frame[2]  >> 3 | frame[3]  << 5)                     & 0x07FF);
  outCh[2]  = ((frame[3]  >> 6 | frame[4]  << 2 | frame[5] << 10)    & 0x07FF);
  outCh[3]  = ((frame[5]  >> 1 | frame[6]  << 7)                     & 0x07FF);
  outCh[4]  = ((frame[6]  >> 4 | frame[7]  << 4)                     & 0x07FF);
  outCh[5]  = ((frame[7]  >> 7 | frame[8]  << 1 | frame[9] << 9)     & 0x07FF);
  outCh[6]  = ((frame[9]  >> 2 | frame[10] << 6)                     & 0x07FF);
  outCh[7]  = ((frame[10] >> 5 | frame[11] << 3)                     & 0x07FF);
  outCh[8]  = ((frame[12]      | frame[13] << 8)                     & 0x07FF);
  outCh[9]  = ((frame[13] >> 3 | frame[14] << 5)                     & 0x07FF);
  outCh[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10)   & 0x07FF);
  outCh[11] = ((frame[16] >> 1 | frame[17] << 7)                     & 0x07FF);
  outCh[12] = ((frame[17] >> 4 | frame[18] << 4)                     & 0x07FF);
  outCh[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9)    & 0x07FF);
  outCh[14] = ((frame[20] >> 2 | frame[21] << 5)                     & 0x07FF);
  outCh[15] = ((frame[21] >> 5 | frame[22] << 3)                     & 0x07FF);
}

// ── Failsafe ──────────────────────────────────────────────────
// Design rationale: throttle drops to zero (cutting motors is the
// only universally-safe action), while roll/pitch/yaw go to center
// (stick neutral) rather than zero, since zero on these axes has no
// special "safe" meaning and center avoids a sudden control input if
// signal is briefly interrupted then recovered mid-frame.
void reset_the_Data() {
  received_data.ch1 = 0;    // Throttle → zero (cuts power — the safe state)
  received_data.ch2 = 127;  // Roll     → center (neutral stick, not "off")
  received_data.ch3 = 127;  // Pitch    → center
  received_data.ch4 = 127;  // Yaw      → center
  received_data.ch5 = 0;
  received_data.ch6 = 0;
  received_data.ch7 = 127;
  failsafe = true;
}

// ── Setup ─────────────────────────────────────────────────────
void setup()
{
  // ── Kill WiFi FIRST — must be before radio.begin() ──────────
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);  // required — gives the radio stack time to actually shut down
  // ────────────────────────────────────────────────────────────

  Serial.begin(115200);   // USB debug — keep this free by using Serial1 for SBUS

  // Serial1 = GPIO2 (D4) TX only — used for SBUS output
  // ESP8266 Arduino core supports SERIAL_8E2
  Serial1.begin(100000, SERIAL_8E2);

  // NOTE: this is an undocumented ESP8266 core register hack that
  // inverts the Serial1 TX line in hardware (SBUS requires inverted
  // logic). It relies on internal UART register names (USC0, UCTXI)
  // that are not part of the public Arduino API and can change or
  // break across ESP8266 core versions. Known to work on core 3.x —
  // pin your exact core version in the README, and if SBUS output
  // stops working after a core update, check this line first.
  USC0(UART1) |= (1 << UCTXI);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();

  reset_the_Data();
  Serial.println("ESP8266 SBUS receiver ready");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  // Receive NRF24 packet
  if (radio.available()) {
    Received_data incoming;
    radio.read(&incoming, sizeof(Received_data));

    if (incoming.version == PROTOCOL_VERSION) {
      received_data = incoming;
      lastRecvTime = millis();
      failsafe = false;
      packetsReceived++;
      consecutiveMisses = 0;
    } else {
      // TX/RX struct layout mismatch — don't trust the payload,
      // treat like a dropped packet instead of flying garbage values.
      packetsBadVersion++;
      consecutiveMisses++;
    }
  }

  // No good signal for FAILSAFE_TIMEOUT_MS → failsafe
  if (millis() - lastRecvTime > FAILSAFE_TIMEOUT_MS) {
    reset_the_Data();
  }

  // Non-blocking frame pacing — replaces delay(FRAME_INTERVAL_MS) so
  // the loop stays responsive if you add more work here later
  // (buttons, status LED, OTA, etc.) without stalling SBUS timing.
  if (millis() - lastFrameTime >= FRAME_INTERVAL_MS) {
    lastFrameTime = millis();

    // Build 16-channel array
    // ch[0]–ch[6]  = our 7 received channels
    // ch[7]–ch[15] = unused, set to SBUS center
    int ch[SBUS_NUM_CHANNELS];
    ch[0] = toSBUS(received_data.ch1);          // Throttle
    ch[1] = toSBUS(received_data.ch2);          // Roll
    ch[2] = toSBUS(received_data.ch3);          // Pitch
    ch[3] = toSBUS(received_data.ch4);          // Yaw
    ch[4] = toSBUS_digital(received_data.ch5);  // Switch 1
    ch[5] = toSBUS_digital(received_data.ch6);  // Switch 2
    ch[6] = toSBUS(received_data.ch7);          // Aux
    for (int i = 7; i < SBUS_NUM_CHANNELS; i++) ch[i] = SBUS_CENTER;

    buildSBUSFrame(ch);
    Serial1.write(sbusFrame, SBUS_FRAME_BYTES);   // send SBUS frame out on D4

#if DEBUG_CHANNELS
    Serial.print(failsafe ? "FAILSAFE  " : "OK  ");
    Serial.print("THR:"); Serial.print(received_data.ch1);
    Serial.print("  ROL:"); Serial.print(received_data.ch2);
    Serial.print("  PIT:"); Serial.print(received_data.ch3);
    Serial.print("  YAW:"); Serial.print(received_data.ch4);
    Serial.print("  SW1:"); Serial.print(received_data.ch5);
    Serial.print("  SW2:"); Serial.print(received_data.ch6);
    Serial.print("  AUX:"); Serial.println(received_data.ch7);
#endif

#if DEBUG_SBUS
    // Round-trip check: decode the frame we just built and print it,
    // to verify buildSBUSFrame() bit-packing against the source values.
    int decoded[SBUS_NUM_CHANNELS];
    decodeSBUSFrame(sbusFrame, decoded);
    Serial.print("DECODED  ");
    for (int i = 0; i < 7; i++) {
      Serial.print("ch"); Serial.print(i); Serial.print(":");
      Serial.print(decoded[i]); Serial.print("  ");
    }
    Serial.println();
#endif
  }

  // ── Periodic link-quality stats ──────────────────────────────
  if (millis() - lastLinkStatsTime > LINK_STATS_PERIOD_MS) {
    lastLinkStatsTime = millis();
    Serial.print("[LINK] packets/s:"); Serial.print(packetsReceived);
    Serial.print("  badVersion:"); Serial.print(packetsBadVersion);
    Serial.print("  consecutiveMisses:"); Serial.println(consecutiveMisses);
    packetsReceived = 0;   // reset for next window
  }
}
