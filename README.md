# SBUS_NRF24_Reciever
# ESP8266 NRF24 → SBUS Receiver

Receives 7 RC channels over an NRF24L01 radio and outputs a standard
**SBUS** frame, so it can plug straight into any flight controller
with an SBUS input (Betaflight, INAV, ESP-Drone, etc.) as if it were
a normal SBUS receiver.

Pairs with a matching NRF24 transmitter sketch (handheld TX / gimbal
board) that sends the same `Received_data` struct.

## Features

- 7 channels: throttle, roll, pitch, yaw, 2 switches, 1 aux
- Outputs a full 16-channel SBUS frame (unused channels held at center)
- Failsafe: throttle cuts to zero, other axes hold center, after 1s of signal loss
- Protocol version byte — TX/RX struct mismatches are detected instead of silently misread
- Basic link-quality stats (packets/sec, dropped/bad packets) printed over USB serial

## Hardware

- ESP8266 board (NodeMCU / Wemos D1 mini, etc.)
- NRF24L01 (+PA/LNA variant recommended for range)
- 1 NPN transistor (BC547 / 2N2222) + resistors, for the SBUS signal inverter

### Wiring

**NRF24 → NodeMCU**

| NRF24 pin | NodeMCU pin | GPIO |
|-----------|-------------|------|
| GND       | GND         | —    |
| Vcc       | 3.3V        | —    |
| CE        | D2          | GPIO4 |
| CSN       | D8          | GPIO15 |
| CLK       | D5          | GPIO14 |
| MOSI      | D7          | GPIO13 |
| MISO      | D6          | GPIO12 |

**SBUS output (inverter required — SBUS is inverted logic):**

```
D4 (GPIO2) ──[1kΩ]──► Base of BC547/2N2222
                        Collector ──► SBUS wire to FC
                        Emitter   ──► GND
3.3V ──[10kΩ]──► Collector (pull-up)
```

Serial1 (GPIO2 / D4, TX-only) is used for SBUS so that the normal USB
`Serial` stays free for debug prints.

## Required libraries

Install via Arduino Library Manager:

- [RF24](https://github.com/nRF24/RF24) by TMRh20

Board package: **esp8266 by ESP8266 Community** (tested on core 3.x —
see note below on the SBUS inversion hack).

## Flashing

1. Open `ESP8266_receiver_SBUS.ino` in the Arduino IDE
2. Select your ESP8266 board under Tools > Board
3. Select the correct COM port
4. Upload

## Configuration

At the top of the sketch:

```cpp
#define DEBUG_CHANNELS 1   // print raw channel values over USB serial
#define DEBUG_SBUS     0   // print decoded SBUS frame (bit-packing sanity check)
```

Leave both at `0` for actual flight use — serial prints cost time and
can perturb loop timing.

The NRF24 pipe address (`pipeIn`) and `PROTOCOL_VERSION` must match
exactly on the transmitter sketch.

## Known limitations / notes

- The SBUS signal inversion uses an undocumented ESP8266 core register
  write (`USC0(UART1) |= (1 << UCTXI)`). This is known to work on core
  3.x; if SBUS output breaks after updating the ESP8266 board package,
  check this line first.
- `radio.setAutoAck(false)` is used for low latency — there's no
  packet acknowledgment/retry between TX and RX.

## License

MIT — see [LICENSE](LICENSE).
