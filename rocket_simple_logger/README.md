# Rocket Simple Logger

Autonomous flight data logger for model rockets. No buttons, no serial commands — plug in the battery and it handles the rest.

## Hardware

- **Arduino Nano ESP32**
- **BNO085** — 9-DOF IMU (accelerometer, gyroscope, rotation vector) via I2C at 0x4A
- **DPS310** — Barometric pressure + temperature sensor via I2C at 0x77
- **Pyro channel** — Pin D5, fires deployment charge on descent through 400 ft
- **LED** — Built-in LED used for status feedback

### Wiring

| Signal | Pin |
|--------|-----|
| I2C SDA | A4 |
| I2C SCL | A5 |
| Pyro | D5 |
| LED | LED_BUILTIN |

## How It Works

### Boot Sequence

1. Initializes flash storage (FFat)
2. Attempts to connect BNO085 and DPS310 over I2C
3. If DPS310 is available, calibrates ground pressure (discards 20 warmup readings, averages 50 good readings)
4. Opens preflight log file
5. Transitions directly to **ARMED** — no manual arming required

### LED Status (ARMED State)

The LED pattern tells you sensor status at a glance:

| Pattern | Meaning |
|---------|---------|
| Steady even blink (500ms) | 0/2 sensors connected — something is wrong |
| 2 blinks, pause, repeat | 1/2 sensors connected — partial functionality |
| 3 blinks, pause, repeat | 2/2 sensors connected — ready to fly |

Other states:
- **BOOT** — Fast blink (100ms)
- **FLIGHT** — Solid on
- **LANDED** — Slow blink (1 second)

### Launch Detection

Triggers when accelerometer reads >2g (19.6 m/s²) for 5 consecutive samples (50ms at 100 Hz). On launch:
- The 2-second ring buffer (pre-launch data) is drained to flash
- High-speed logging begins at 100 Hz
- Preflight file is closed

### In-Flight

- All sensor data logged at 100 Hz to internal flash (batch writes of 20 samples, sync every 100)
- Tracks max acceleration, max altitude, min pressure
- **Pyro deployment**: When the rocket rises above 400 ft and then descends back through 400 ft, pin D5 fires a 100ms pulse

### Landing Detection

Triggers when both conditions hold for 5 continuous seconds (after a minimum 3-second flight):
- Acceleration magnitude between 7.0 and 12.5 m/s² (near 1g)
- Pressure variance over the last 100 samples is below threshold

On landing:
- Remaining data is flushed to flash
- Flight metadata is written (duration, sample counts, max g, max altitude)
- WiFi access point starts automatically

### WiFi Data Download

After landing, the board creates a WiFi access point:

| Setting | Value |
|---------|-------|
| SSID | `RocketLogger` |
| Password | `rocketdata` |
| IP | `192.168.4.1` |

Connect to the WiFi network and open a browser to `192.168.4.1`.

**Dashboard features:**
- Live sensor readings (accelerometer, gyroscope, quaternion, pressure, temperature, altitude) — updates every 1.5 seconds
- Flight summary (max g, max altitude, duration, sample counts)
- Download links for flight and preflight CSV data

**Endpoints:**

| URL | Description |
|-----|-------------|
| `GET /` | Web dashboard |
| `GET /api/status` | JSON status (used by dashboard) |
| `GET /data.csv` | Flight data as CSV |
| `GET /preflight.csv` | Preflight data as CSV |

## Data Format

Each data point is a packed binary struct on flash, exported as CSV with these columns:

```
timestamp_ms, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
quat_w, quat_x, quat_y, quat_z, pressure_pa, temperature_c, altitude_m, d5
```

- Accelerometer values in m/s²
- Gyroscope values in rad/s
- Quaternion is the rotation vector (w, i, j, k)
- Pressure in Pascals
- Altitude in meters (relative to ground pressure at boot)
- d5 is the pyro pin state (0 or 1)

## Configuration

All tunable parameters are in `config.h`:

- Sampling rates (default: 100 Hz fast, 2 Hz preflight)
- Launch threshold (default: 2g for 50ms)
- Landing threshold (default: 5 seconds of stability)
- Pyro deploy altitude (default: 400 ft / 121.92 m)
- WiFi SSID and password
- Pin assignments

## Upload

In the Arduino IDE:
- Board: **Arduino Nano ESP32**
- Partition scheme: default (must include FFat)
- Upload method: **Sketch → Upload Using Programmer (Esptool)**

## File Structure

```
rocket_simple_logger/
├── rocket_simple_logger.ino  — Main sketch
├── config.h         — All configuration constants
├── data_types.h     — DataPoint struct, FlightState enum, CSV header
├── ring_buffer.h    — Pre-launch ring buffer (2 seconds)
└── README.md
```
