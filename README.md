# Rocket Flight Data Logger v4

Arduino Nano ESP32 flight computer with IMU and barometric logging.

## Hardware

| Component | Role |
|-----------|------|
| Arduino Nano ESP32 | MCU, flash storage (FFat) |
| BNO085 (I2C 0x4A) | 9-DOF IMU — accelerometer, gyroscope, rotation vector |
| DPS310 (I2C 0x77) | Barometric pressure + temperature |

## State Machine

| State | Description |
|-------|-------------|
| **BOOT** | Init storage, sensors, calibrate ground pressure |
| **IDLE** | Waiting for `ARM` command (skipped if `REQUIRE_ARMING = false`) |
| **ARMED** | Sampling at 100 Hz, filling ring buffer, logging preflight at 2 Hz, watching for launch |
| **FLIGHT** | Logging all samples to flash, tracking max accel/altitude, watching for landing |
| **LANDED** | Flight file closed, metadata saved, awaiting data download |
| **DOWNLOAD** | WiFi AP active, serving data over HTTP |

## LED Blink Scheme

| State | LED Behavior |
|-------|-------------|
| **BOOT** | Fast blink (100ms) |
| **IDLE** | Slow blink (1000ms) |
| **ARMED** | Quick blink (250ms) |
| **FLIGHT** | Solid ON |
| **LANDED** | Very slow blink (2000ms) |
| **DOWNLOAD** | Medium blink (500ms) |

## Launch Detection

- Triggers when acceleration exceeds **3g (29.4 m/s²)** for **5 consecutive samples** (50ms at 100 Hz)
- On launch, the 2-second ring buffer is drained to flash, capturing pre-launch data

## Landing Detection

Requires all conditions sustained for **5 seconds** (after a minimum 3-second flight):
- Acceleration between 7.0 and 12.5 m/s² (near 1g)
- Pressure variance below threshold (stable altitude)

## Data Recording

- **Preflight**: logged to `/preflight.bin` at 2 Hz
- **Ring buffer**: rolling 2-second window (200 samples) kept in RAM — written to flight file on launch
- **In-flight**: batch-written to `/flight.bin` at 100 Hz (batches of 20, flushed every 100 samples)
- **Metadata**: saved to `/meta.txt` on landing (duration, max accel, max altitude, sample counts)

### Recorded Fields

`timestamp_ms, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, quat_w, quat_x, quat_y, quat_z, pressure_pa, temperature_c, altitude_m`

## Serial Commands (115200 baud)

| Command | Action |
|---------|--------|
| `STATUS` | Print current state, sensor status, live readings, storage usage |
| `ARM` | Arm launch detection (from IDLE state) |
| `DOWNLOAD` | Dump all data as CSV over serial |
| `WIFI` | Start WiFi AP for wireless access (from IDLE or LANDED) |
| `FORMAT` | Erase all stored data |
| `RESET` | Reset state machine to IDLE |
| `TEST [secs]` | Start test flight for given duration (e.g. `TEST 10`). No duration = manual stop only |
| `STOP` | End a test flight |
| `FIRE` | Test fire pyro channel (requires typing `CONFIRM` twice) |

## WiFi Access

When you send the `WIFI` command, the ESP32 creates its own WiFi network — it becomes a mini hotspot. No internet or external router needed. It's a direct connection between the board and your phone/laptop.

WiFi can be started from **IDLE** or **LANDED** states. It is not available during ARMED or FLIGHT to avoid interfering with the timing-critical 100 Hz sampling loop.

### How to Use

1. Type `WIFI` in the serial monitor (115200 baud)
2. On your phone or laptop, open WiFi settings and connect to:
   - **Network**: `RocketLogger`
   - **Password**: `rocketdata`
3. Open a browser and go to `http://192.168.4.1/`
4. The dashboard shows live sensor readings, state, and flash usage (auto-refreshes every 1.5s)
5. Use the buttons to send commands:
   - **ARM** — arms launch detection (shuts down WiFi)
   - **RESET** — resets state machine to IDLE (WiFi stays active)
   - **FORMAT** — erases all stored data (WiFi stays active)
   - **TEST FLIGHT** — starts a test flight for the specified seconds (WiFi stays active during recording)
   - **STOP** — ends a test flight early
   - **FIRE PYRO** — test fires the pyro channel (two confirmation steps)
6. Scroll down for flight metadata and CSV download links:
   - **Flight Data** — the in-flight recording at 100 Hz
   - **Preflight Data** — the slow-rate pre-launch log at 2 Hz

### WiFi API

The dashboard uses a JSON API that can also be called directly (e.g. with `curl`):

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Live dashboard (HTML) |
| `/api/status` | GET | JSON status (state, sensors, readings, flash usage) |
| `/api/arm` | POST | Arm launch detection (WiFi shuts down) |
| `/api/reset` | POST | Reset state machine to IDLE |
| `/api/format` | POST | Erase all stored data |
| `/api/test` | POST | Start test flight (param: `seconds`) |
| `/api/stop` | POST | End test flight |
| `/api/fire` | POST | Test fire pyro (call twice — first arms, second fires) |
| `/api/fire/cancel` | POST | Cancel a pending fire confirmation |
| `/data.csv` | GET | Download flight data as CSV |
| `/preflight.csv` | GET | Download preflight data as CSV |

### How It Works

- `WiFi.softAP()` turns the ESP32 into an access point (the board *is* the network)
- A web server on port 80 handles HTTP requests
- When you download a CSV, the board reads binary data from flash, converts each sample to CSV on the fly, and streams it to your browser
- No data touches the internet — it's a direct local link

### Notes

- The board stops logging while in download mode (state becomes DOWNLOAD)
- You need to be within WiFi range of the board (a few meters)
- Arming via the dashboard shuts down WiFi so the board can focus on launch detection
- To exit download mode without arming, power cycle the board or send `RESET` over serial

## Ground Testing

The `TEST` command lets you record flight data without needing an actual launch. It uses the same recording pipeline as a real flight (ring buffer drain, 100 Hz batch writes, metadata on stop).

- **Serial**: `TEST 10` starts a 10-second test flight. `TEST` with no number runs until you send `STOP`.
- **WiFi dashboard**: Enter seconds in the input field and press **TEST FLIGHT**. Press **STOP** to end early.

On completion (timer expires or `STOP` sent), the board transitions to LANDED with a valid flight file that can be downloaded normally.

## Pyro Channel (Deployment Charge)

Pin D5 controls a pyro channel for deployment charge firing. The pin is held LOW at all times except during an intentional fire event.

### In-Flight Deployment

During a real flight (not test flights), the pyro fires automatically when:
1. The rocket has risen **above 400ft (122m)**
2. The rocket is **descending** (altitude decreasing)
3. Altitude drops **through 400ft** on the way down

The charge fires once per flight — a one-shot flag prevents re-firing. The pulse is 100ms (D5 HIGH then back to LOW).

### Test Fire

The `FIRE` command lets you test the pyro channel on the ground. It is blocked during ARMED and FLIGHT states.

- **Serial**: Type `FIRE`, then `CONFIRM`, then `CONFIRM` again
- **WiFi dashboard**: Press FIRE PYRO (browser confirm dialog), then CONFIRM FIRE (second browser confirm dialog)

### Safety

- D5 is explicitly set LOW at boot
- In-flight firing requires passing above the deploy altitude first, then descending through it
- Test firing requires double confirmation
- Pyro is never fired during test flights
- Pyro state is reset on RESET, FORMAT, and new flight transitions

## Sensor Failure Handling

If a sensor fails to initialize at boot, the system prompts over serial:
- Type `PROCEED` to continue without the missing sensor
- Reset the board to retry

Missing sensor behavior:
- **No BNO085**: accel/gyro/quaternion fields read zero; launch detection will not trigger
- **No DPS310**: pressure/temp/altitude fields read zero; ground pressure calibration is skipped

## Configuration

All tunable parameters are in `config.h`:
- I2C pins and clock speed
- Sensor addresses and report intervals
- Sample rates (fast: 100 Hz, slow: 2 Hz)
- Ring buffer duration (2 seconds)
- Launch threshold (2g) and confirmation window
- Landing detection thresholds and stability duration
- Flash batch size and sync interval
- Pyro pin, pulse duration, and deploy altitude (400ft)
- WiFi AP credentials
- Arming requirement toggle

## Upload

**Sketch > Upload Using Programmer (Esptool)** — uses the default FFat partition.
