# ESP32 Distributed LED Synchronization (Leaderless, Stable, Self-Running)

## Overview

This project implements **synchronized LED blinking across multiple ESP32 devices** using Wi-Fi (ESP-NOW), without requiring a central master node.

Each device:
- blinks an LED on **pin 2**
- uses a **1 Hz cycle** (10 ms ON, 990 ms OFF)
- runs **independently**, even when alone
- **automatically synchronizes** with nearby devices when present

The system is:
- **fully distributed** (no master)
- **robust to restarts**
- **stable over long time periods**
- **self-healing** when devices join or leave

---

## Key Properties

### 1. Always ticking (even alone)
Each ESP32 maintains its own internal clock and continues blinking indefinitely:
- no dependency on network
- no waiting for synchronization packets

### 2. Leaderless synchronization
All devices:
- broadcast their timing
- listen to others
- converge using **consensus (median)**

No device has priority.

### 3. Smooth, stable synchronization
Synchronization is achieved by:
- correcting **phase** (short-term alignment)
- correcting **frequency** (long-term drift)

This avoids jitter and ensures stability.

### 4. Dynamic network
Devices can:
- join at any time
- leave at any time
- restart independently

New devices automatically lock to the group.

---

## How It Works

### Local oscillator (core idea)

Each device behaves like a **free-running clock**:

```

LED ON  at t = 0 ms
LED OFF at t = 10 ms
cycle repeats every 1000 ms

````

Internally:

```cpp
nextTickUs = timestamp of next LED ON event
````

This schedule is always maintained locally.

---

### Network synchronization

Devices exchange timing using **ESP-NOW broadcast packets** every 200 ms.

Each packet contains:

* sender local timestamp
* sender's next scheduled LED ON time
* sequence number and boot ID

---

### Phase estimation

When a device receives a packet:

1. It computes:

   ```
   time remaining until sender's next tick
   ```

2. Converts that into its own clock:

   ```
   estimated sender next tick (local time)
   ```

3. Compares with its own next tick:

   ```
   phase error = sender - self
   ```

---

### Consensus (no master)

Each device collects phase estimates from multiple peers and computes:

```
median phase error
```

Why median:

* robust to outliers
* ignores noisy or delayed packets
* prevents any single device from dominating

---

### Phase correction (short-term)

* On first lock: large correction allowed (snap)
* After lock: small corrections only (slew)

This ensures:

* fast acquisition
* stable steady state (no visible jitter)

---

### Frequency correction (long-term)

Each device also estimates **drift (ppm)**:

```
drift ≈ change in phase error over time
```

Then adjusts its internal period slightly:

```
period ≈ 1,000,000 µs ± small correction
```

This compensates for:

* oscillator differences between boards
* long-term drift

---

### Control model (important)

Each device behaves like:

> “I run my own clock, and I gently adjust it based on others.”

NOT:

* “wait for packet → blink”
* “follow a master”

---

## System Behavior

### Single device

* blinks at 1 Hz
* no synchronization needed

### Multiple devices

* converge to common phase
* remain synchronized over time

### New device joins

* starts blinking immediately
* locks to group within a few seconds

### Device disappears

* remaining devices continue normally

### All devices reboot

* first active devices define phase
* others join and synchronize

---

## Timing Characteristics

### Default configuration

| Parameter       | Value    |
| --------------- | -------- |
| Period          | 1 second |
| LED ON duration | 10 ms    |
| Sync interval   | 200 ms   |
| Control loop    | 100 ms   |

---

### Synchronization time

Typical time for a new device to reach **≤ 1 ms alignment**:

* **best case:** ~0.5–1.5 s
* **typical:** ~2–6 s
* **noisy environments:** longer

This depends on:

* RF conditions
* number of devices
* packet loss

---

### Accuracy limitations

Synchronization precision is limited by:

* Wi-Fi / ESP-NOW latency jitter
* CPU scheduling jitter
* oscillator drift

This implementation minimizes those effects using:

* future timestamp scheduling
* median filtering
* smooth corrections
* frequency trimming

---

## Technical Details

### Communication: ESP-NOW

* connectionless
* broadcast-based
* low latency
* no access point required

All devices must:

* use the same Wi-Fi channel
* be within range

---

### Timing source

Uses:

```cpp
esp_timer_get_time()
```

* microsecond-resolution monotonic clock

---

### Internal control loop

Every 100 ms:

1. collect recent peer data
2. compute median phase error
3. adjust:

   * phase (fast correction)
   * frequency (slow correction)

---

### LED scheduling

* based on **absolute timestamps**
* not tied to packet arrival
* guarantees stable rhythm

---

## Project Structure

```
.
├── platformio.ini
└── src
    └── main.cpp
```

---

## Hardware Requirements

* ESP32 boards (any standard variant)
* LED connected to:

  * GPIO 2 (default)
  * with resistor

---

## PlatformIO Configuration

Example:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

---

## Configuration Parameters

You can tune behavior via constants in `main.cpp`:

### Network

```cpp
WIFI_CHANNEL
SEND_INTERVAL_US
```

### Synchronization

```cpp
ACQUIRE_SNAP_THRESHOLD_US
PHASE_SLEW_LIMIT_US
CONTROL_INTERVAL_US
```

### Stability

```cpp
MAX_TRIM_PPM
PEER_TIMEOUT_US
LOCK_TIMEOUT_US
```

---

## Tuning Guidelines

### Faster synchronization

* decrease `SEND_INTERVAL_US`
* increase `PHASE_SLEW_LIMIT_US`

### More stable output

* decrease `PHASE_SLEW_LIMIT_US`
* increase `CONTROL_INTERVAL_US`

### Better long-term stability

* increase `MAX_TRIM_PPM`
* allow slower drift correction

---

## Limitations

* not suitable for sub-100 µs synchronization requirements
* Wi-Fi interference can degrade performance
* no guaranteed packet delivery (ESP-NOW is best-effort)

---

## Possible Improvements

* hardware timer (GPTimer) for even lower jitter
* two-way timing exchange (NTP-style)
* adaptive control gains
* outlier rejection beyond median
* visual debug (phase error plotting)

---

## Summary

This system implements:

* a **distributed clock synchronization system**
* based on **local oscillators + consensus**
* with **phase and frequency correction**

It ensures:

* continuous operation
* no central dependency
* stable long-term synchronization
* automatic recovery from failures

---

## Conceptual Model

Each device behaves like:

> “I am a clock. I run continuously.
> I listen to others.
> If I am early or late, I gently adjust.”

