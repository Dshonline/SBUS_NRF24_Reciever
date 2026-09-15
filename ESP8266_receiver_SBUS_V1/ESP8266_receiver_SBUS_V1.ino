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
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP8266WiFi.h>

// ── Pin definitions ───────────────────────────────────────────
#define NRF_CE  D2   // GPIO4
#define NRF_CSN D8   // GPIO15

// ── Radio ─────────────────────────────────────────────────────
const uint64_t pipeIn = 0xE8E8F0F0E1LL;  // must match transmitter
RF24 radio(NRF_CE, NRF_CSN);

// ── Data struct (must match transmitter exactly) ───────────────
struct Received_data {
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

// ── SBUS frame buffer ─────────────────────────────────────────
uint8_t sbusFrame[25];

// Map 0–255 byte  →  SBUS 172–1811
int toSBUS(byte val) {
  return map(val, 0, 255, 172, 1811);
}

// Map digital 0/1  →  SBUS min / max
int toSBUS_digital(byte val) {
  return val ? 1811 : 172;
}

void buildSBUSFrame(int ch[16]) {
  sbusFrame[0]  = 0x0F;   // header

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

  // Flags: bit2 = signal loss, bit3 = failsafe
  sbusFrame[23] = failsafe ? 0x0C : 0x00;
  sbusFrame[24] = 0x00;   // footer
}

// ── Failsafe ──────────────────────────────────────────────────
void reset_the_Data() {
  received_data.ch1 = 0;    // Throttle → zero (safe for drone)
  received_data.ch2 = 127;  // Roll     → center
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
  USC0(UART1) |= (1 << UCTXI);   // ← this inverts the TX signal in hardware
  
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
    radio.read(&received_data, sizeof(Received_data));
    lastRecvTime = millis();
    failsafe = false;
  }

  // No signal for > 1 second → failsafe
  if (millis() - lastRecvTime > 1000) {
    reset_the_Data();
  }

  // Build 16-channel array
  // ch[0]–ch[6]  = our 7 received channels
  // ch[7]–ch[15] = unused, set to SBUS center (992)
  int ch[16];
  ch[0] = toSBUS(received_data.ch1);          // Throttle
  ch[1] = toSBUS(received_data.ch2);          // Roll
  ch[2] = toSBUS(received_data.ch3);          // Pitch
  ch[3] = toSBUS(received_data.ch4);          // Yaw
  ch[4] = toSBUS_digital(received_data.ch5);  // Switch 1
  ch[5] = toSBUS_digital(received_data.ch6);  // Switch 2
  ch[6] = toSBUS(received_data.ch7);          // Aux
  for (int i = 7; i < 16; i++) ch[i] = 992;  // unused → center

  buildSBUSFrame(ch);
  Serial1.write(sbusFrame, 25);   // send SBUS frame out on D4

  // ── Debug output on USB serial ──────────────────────────
  Serial.print(failsafe ? "FAILSAFE  " : "OK  ");
  Serial.print("THR:"); Serial.print(received_data.ch1);
  Serial.print("  ROL:"); Serial.print(received_data.ch2);
  Serial.print("  PIT:"); Serial.print(received_data.ch3);
  Serial.print("  YAW:"); Serial.print(received_data.ch4);
  Serial.print("  SW1:"); Serial.print(received_data.ch5);
  Serial.print("  SW2:"); Serial.print(received_data.ch6);
  Serial.print("  AUX:"); Serial.println(received_data.ch7);
  // ────────────────────────────────────────────────────────

  delay(14);  // 14ms frame rate (standard SBUS)
}
