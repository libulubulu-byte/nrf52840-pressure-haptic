# XIAO nRF52840 — Foot-Pressure (FSR) Sensing with BLE Telemetry and LRA Haptic Feedback

**An Arduino sketch for the Seeed XIAO nRF52840 / nRF52840 Sense**

[![Board](https://img.shields.io/badge/board-XIAO%20nRF52840%20%7C%20Sense-3ddc84)](https://wiki.seeedstudio.com/XIAO_BLE/)
[![Core](https://img.shields.io/badge/Arduino%20core-Seeeduino%3Anrf52%201.1.13-00979d)]()
[![Hire me on Upwork](https://img.shields.io/badge/Upwork-Hire%20me-14a800?logo=upwork&logoColor=white)](https://www.upwork.com/freelancers/~01XXXXXXXXXXXXXXXX)

A thin-film force sensor (FSR) in a shoe insole is sampled and filtered, streamed over a
custom **BLE GATT service**, and turned into **haptic feedback** through a **DRV2605 + LRA**
actuator. A **PIR motion sensor** adds presence-based continuous vibration. The board is
meant to run off a coin cell, so the sketch also carries a power state machine, module
supply gating and a from-scratch low-power sleep path.

The whole project is one sketch folder: `nrf52840-pressure-haptic.ino`, its configuration
header `power_config.h`, and this README.

---

## Table of contents

- [Highlights](#highlights)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [BLE interface](#ble-interface)
- [Build and upload](#build-and-upload)
- [Configuration](#configuration)
- [Low-power design notes](#low-power-design-notes)
- [Field notes and gotchas](#field-notes-and-gotchas)
- [Known limitations](#known-limitations)
- [Hire me](#hire-me)

---

## Highlights

| Area | What is implemented |
|---|---|
| **FSR acquisition** | 12-bit SAADC, 8x hardware oversampling, 5-tap median filter, EMA low-pass (`alpha = 0.25`), peak hold |
| **Event detection** | Hysteresis (`TH_ON 1200` / `TH_OFF 900`), 60 ms debounce, 250 ms refractory period, runtime threshold calibration |
| **BLE GATT service** | 1 custom service + 6 characteristics: 9-byte data notify, command write, threshold write, ASCII status notify, and two single-value notify characteristics for live charting |
| **Haptic feedback** | DRV2605 over I2C: single / double / long patterns, continuous RTP vibration while PIR is active, live auto-calibration, hot-plug re-initialisation |
| **Presence sensing** | PIR on a GPIO with a warm-up guard (no phantom trigger on power-up) and hold-time reporting |
| **Power management** | State machine (BOOT / CONNECTED / ADVERTISING / IDLE_ADV / DEEP_SLEEP), bare `__WFE()` idle sleep, all LEDs off, PIR supply gating, DRV2605 to STANDBY |
| **Runtime configuration** | Sample/wake period selectable at runtime (1 / 3 / 5 / 10 s) over BLE - no re-flash required |
| **Diagnostics** | Periodic `[pwr]` state line (loop rate, advertising interval, wake period, pin levels, LED state), BLE disconnect reason, I2C bus health probe |

---

## Hardware

| Qty | Item | Notes |
|---|---|---|
| 1 | Seeed **XIAO nRF52840** or **XIAO nRF52840 Sense** | Board package `Seeeduino:nrf52` (tested with 1.1.13) |
| 1 | **FSR** (force sensing resistor) film sensor | Foot insole pressure input, wired as a voltage divider into `A0` |
| 1 | **10 kOhm** resistor | FSR divider pull-down |
| 1 | **DRV2605** breakout + **LRA** actuator | I2C haptics driver, `0x5A` default address |
| 1 | **10 uF** capacitor | Local decoupling at the DRV2605 module supply |
| 1 | **PIR** motion module | e.g. AM312 (3.3 V) or HC-SR501 (5 V, 3.3 V-compatible output) |
| 1 | LiPo cell or coin cell | The sketch is written for battery operation |

---

## Wiring

| XIAO pin | nRF pin | Connects to | Notes |
|---|---|---|---|
| `A0` / `D0` | P0.02 | FSR divider midpoint | Analogue input |
| `D4` | P0.04 | DRV2605 `SDA` | I2C SDA |
| `D5` | P0.05 | DRV2605 `SCL` | I2C SCL |
| `D1` | P0.03 | PIR `OUT` | Digital input, active high |
| `D2` | P0.28 | PIR module `VCC` | GPIO-gated supply (high = powered) |
| `D3` | P0.29 | *unused* | Firmware leaves it as a high-impedance input (`DRV_PWR_GATE_ENABLE 0`); the DRV2605 module is powered directly from 3V3 |
| `D11` | P0.26 | RGB red LED | The on-board RGB LED is **common anode**: `LOW` = on |
| `D12` | P0.06 | RGB blue LED | Also used by the library's advertising blink, which this sketch disables |
| `D13` | P0.30 | RGB green LED | |

Warnings worth respecting:

- **nRF52840 pins are not 5 V tolerant.** A PIR module must output 3.3 V (AM312 needs 3.3 V
  supply; HC-SR501 can use 5 V but its output is only safe if it swings to 3.3 V).
- `D22` / P0.13 is the **HICHG** charge-current configuration pin. Do not repurpose it.
- Motor current must never flow through an IO pin. A single nRF52840 pin is rated 15 mA while
  an LRA is rated 75 mA nominal - power the module from the 3V3 rail.
- Only one pin may be used for one purpose. Driving a supply pin *and* using it as a signal
  disturbs the I2C bus.

---

## BLE interface

Device name: `XIAO-FSR-Haptic`. TX power: `+4 dBm`.

### Service and characteristics

| Role | UUID |
|---|---|
| Service | `4fafc201-1fb5-459e-8fcc-c5c9c331914b` |
| Data notify (9 bytes) | `beb5483e-36e1-4688-b7f5-ea07361b26a8` |
| Command write | `beb5483e-36e1-4688-b7f5-ea07361b26a9` |
| Threshold write | `beb5483e-36e1-4688-b7f5-ea07361b26aa` |
| Status notify (ASCII) | `beb5483e-36e1-4688-b7f5-ea07361b26ab` |
| Filtered value notify (uint16 LE) | `beb5483e-36e1-4688-b7f5-ea07361b26ac` |
| Raw value notify (uint16 LE) | `beb5483e-36e1-4688-b7f5-ea07361b26ad` |

### Data packet (data notify, 9 bytes, little-endian, packed)

| Offset | Type | Field |
|---|---|---|
| 0 | `uint16` | `raw` - oversampled ADC value |
| 2 | `uint16` | `filt` - median + EMA filtered value |
| 4 | `uint16` | `peak` - peak hold |
| 6 | `uint8` | `pressed` - 0/1 current press state |
| 7 | `uint8` | `drv_ok` - 0/1 DRV2605 online |
| 8 | `uint8` | reserved, 0 |

The two single-value characteristics exist because most mobile BLE apps can only plot one
value at a time; set their value format to `UINT16 (little-endian)` to get a live chart
(nRF Connect, for example).

### Commands (write one ASCII character to the command characteristic)

| Character | Action |
|---|---|
| `S` | Single-click vibration |
| `D` | Double-click vibration |
| `L` | Long vibration |
| `C` | Calibrate thresholds from the current filtered value (needs a value above 200) |
| `P` | Clear the peak value |
| `1` / `3` / `5` / `0` | Set sample/wake period to 1 s / 3 s / 5 s / 10 s |
| `A` | Run DRV2605 auto-calibration (physically vibrates for 1-2 s) |
| `I` | Re-initialise DRV2605 (use after hot-plugging the module) |

### Threshold characteristic

Write 4 bytes little-endian `[thOn, thOff]`, or 2 bytes for a single threshold that gets
`1.15x` / `0.80x` hysteresis automatically.

Connection parameters are negotiated in two stages: a fast 30 ms interval for service
discovery, then a request to drop to 1000 ms (slave latency 1, supervision timeout 6000 ms)
3 s after the link is up. The values are validated at compile time against the Bluetooth
specification rule and Apple's accessory guidance.

---

## Build and upload

### Prerequisites

1. **Arduino IDE** with the **Seeed nRF52 Boards** package (`Seeeduino:nrf52`, tested with
   1.1.13) — Boards Manager URL and instructions are on the Seeed wiki.
2. Library: **Adafruit DRV2605 Library** (Library Manager).

### Folder name

Arduino requires the **sketch folder name to match the main `.ino` file name**. Here the
sketch is `nrf52840-pressure-haptic.ino`, and a plain `git clone` produces exactly that
folder name, so it just works.

It breaks if the folder name changes:

| How you got the code | Folder name | What to do |
|---|---|---|
| `git clone` | `nrf52840-pressure-haptic` | nothing to do |
| **Code → Download ZIP** | `nrf52840-pressure-haptic-main` | rename the folder to `nrf52840-pressure-haptic` |
| you renamed the folder | anything | rename the main `.ino` to the same name |

Otherwise the IDE has to move the file into a correctly named folder, and `arduino-cli`
refuses outright:

```
Can't open sketch: main file missing from sketch: .../<folder>/<folder>.ino
```

### Steps

1. Open `nrf52840-pressure-haptic.ino` in the Arduino IDE.
2. Select **Seeed XIAO nRF52840** or **Seeed XIAO nRF52840 Sense** as the board (either
   works; the Sense variant is what the sketch was tuned on).
3. Select the port and upload.

### First test

Open the serial monitor at **115200 baud**. The `[pwr]` diagnostic line is printed
periodically:

```
[pwr] state=ADVERTISING drv=on pir=0 hap=0 loop=9/s adv=1000ms wake=3000ms samp=3000ms d2=1 drvgate=0 led=0/0/0
```

- `loop=9/s` means the CPU really sleeps. A `loop` rate in the thousands means the CPU is
  spinning (see [Low-power design notes](#low-power-design-notes)).
- `drv=on/off` tells you whether the DRV2605 answered on I2C — the single most useful field
  when "the motor does not buzz".
- `adv` is the advertising interval (fixed at 1000 ms by design); `wake`/`samp` are the idle
  and the currently applied sampling periods.

---

## Configuration

Everything tunable lives in `power_config.h`. The three values that get changed most:

| Macro | Meaning | Default |
|---|---|---|
| `WAKE_THRESHOLD` | ADC count above which the board counts as "pressed" | `500` |
| `SAMPLE_WAKE_PERIOD_MS` | Idle sampling period while nobody is connected (runtime-switchable 1/3/5/10 s) | `3000` |
| `SAMPLE_CONNECTED_MS` | Sampling period while a central is connected | `1000` |

Other groups in the same file: advertising strategy (`ADV_FAST_INTERVAL_MS`,
`ADV_USE_SLOW_PHASE`), deep sleep (`DEEP_SLEEP_AFTER_S`, default `0` = disabled),
measurement mode (`PWR_MEASURE_MODE`), the idle sleep primitive (`PWR_SLEEP_USE_WFE`),
connection parameters (validated at compile time), and the global LED switch
(`PWR_LED_HEARTBEAT`, default `0` = all LEDs off).

The signal-processing and haptics constants (thresholds, filter windows, vibration mode,
DRV2605 drive mode) are at the top of the `.ino`.

---

## Low-power design notes

Everything in this section was measured on the real board; the reasoning is kept in the
comments next to the macros in `power_config.h`.

- **Idle sleep uses a bare `__WFE()`**, not `sd_app_evt_wait()`. On this core + SoftDevice
  S140 combination `sd_app_evt_wait()` returns immediately, which turns the main loop into a
  spin (measured `loop = 45000/s`) and adds 3-6 mA of static current.
- **Never measure current with USB plugged in.** The USB PHY and the 1 ms SOF interrupt keep
  the CPU awake; current draw is dominated by the interface, not by the application.
- **`PWR_MEASURE_MODE 1`** skips `Serial.begin()` entirely. The board then no longer
  enumerates as a COM port, so you flash by double-tapping RESET and copying the `.uf2`
  file to the bootloader drive.
- **Module supply gating matters.** A module's LDO and power LED cost mA; gating the PIR
  module `VCC` with `D2` is what makes a low standby current possible. What remains is the
  modules' own quiescent current, which firmware cannot remove.
- **LEDs are mA-level.** The on-board RGB LED is common anode, and the library's advertising
  blink is disabled via `Bluefruit.autoConnLed(false)`.
- **Broadcast continuously.** The sketch advertises at a fixed 1000 ms interval and calls
  `setFastTimeout(0)` so the stack never reconfigures advertising. Stopping or reconfiguring
  advertising creates a window in which connection requests are dropped — which is exactly
  what "the device is visible but will not connect" looks like from the phone's side.
- **Deep sleep is disabled by default** (`DEEP_SLEEP_AFTER_S 0`), because it stops
  advertising and the device stays undiscoverable until a GPIO wake-up.

---

## Field notes and gotchas

A condensed list of the real issues hit during development — they are the reason several
design decisions above look the way they do.

| Symptom | Cause | Resolution |
|---|---|---|
| Phone cannot find or connect | Advertising was stopped after 30 s to "save power" | Keep advertising permanently; the interval is the power knob, not an on/off switch |
| Service discovery fails, then the link drops | Blocking work (I2C writes, `delay()`) inside a BLE callback | Callbacks only set flags; all work happens in the main loop |
| Connection intermittently fails | `stop()` + `start()` around every interaction, plus a 3000 ms idle advertising interval | Single fixed 1000 ms interval, never reconfigured; only `start()` when advertising is genuinely not running |
| Connection fails although the device is visible | A 1000 ms connection interval used from the start, so service discovery times out | Advertise a fast 30 ms interval, then request 1000 ms after 3 s |
| Motor does not buzz | *Wiring* - not firmware. Measure the module supply voltage, ground continuity and SDA/SCL | Check hardware first; use `drv=on/off` as the software-side indicator |
| Motor stops buzzing after the bus was disturbed | A module that lost power pulls the I2C bus down and the firmware blocks on the bus (zero log output) | Never cut module power while I2C pull-ups are live; re-power in the correct order |
| An unusual I2C address (e.g. `0x5B`) responds | One pin used for two purposes at once | One pin, one purpose |
| Motor vibrates for two seconds on every power-up | DRV2605 auto-calibration physically drives the actuator | Disable calibration on boot; trigger it on demand with the `'A'` command |
| Motor vibrates once at power-up | PIR modules output a valid level during their warm-up window | Require one inactive reading before accepting triggers |
| Standby current of 5.6 mA | The CPU never slept (`sd_app_evt_wait()` returning immediately) | Use a bare `__WFE()`; verify with `loop=xx/s` |
| Two source files silently reverted | The sketch was open in the Arduino IDE, which wrote its stale buffer over the disk | Close the sketch in the IDE (or reopen it) before editing files externally |
| `cleaning build path: unlinkat ... directory is not empty` | The IDE and `arduino-cli` share a build cache | Use a dedicated build path, or clear the cache directory |
| `Can't open sketch: main file missing from sketch` | The sketch folder name does not match the main `.ino` name (clone vs. "Download ZIP" vs. a renamed folder) | Make the folder and the `.ino` name match |

---

## Known limitations

- Sub-20 uA deep sleep is not achievable on this board: the on-board flash, IMU, microphone,
  LDO and charger IC have a quiescent current floor of tens of microamps. The real limit is
  set by the hardware, not by the sketch.
- BLE payload budget: with a 128-bit service UUID and TX power in the advertising packet, the
  device name is truncated in some scanners. Open the device entry to see the full name.
- The 10 s sampling period trades responsiveness for power: a short press can fall between
  two samples. Use the 1 s setting when trigger latency matters.
- The source comments are in Chinese; every public interface (BLE, pins, commands, log
  lines) is documented in English here.

---

## Hire me

I build embedded firmware like this: low-power nRF52/ESP32 designs, BLE services, sensor
pipelines, actuator control, and power optimisation measured on real hardware.

**Upwork:** [https://www.upwork.com/freelancers/~01XXXXXXXXXXXXXXXX](https://www.upwork.com/freelancers/~01XXXXXXXXXXXXXXXX)

**GitHub:** [github.com/libulubulu-byte](https://github.com/libulubulu-byte)

---

## License

No license file is included yet. If you would like to reuse this code, please get in touch
through Upwork.
