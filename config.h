#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  ROCKET DATA LOGGER - CONFIGURATION
// ============================================================

// --- I2C Pins (Arduino Nano ESP32) ---
#define I2C_SDA_PIN        A4
#define I2C_SCL_PIN        A5
#define I2C_CLOCK_HZ       400000

// --- BNO085 ---
#define BNO085_I2C_ADDR    0x4A
#define BNO_REPORT_INTERVAL_US  10000

// --- DPS310 ---
#define DPS310_I2C_ADDR    0x77

// --- Sampling ---
#define FAST_RATE_HZ       100
#define SLOW_LOG_HZ        2
#define SAMPLE_INTERVAL_US (1000000 / FAST_RATE_HZ)

// --- Ring Buffer ---
#define RING_BUFFER_SECS   2
#define RING_BUFFER_SIZE   (FAST_RATE_HZ * RING_BUFFER_SECS)

// --- Flash Write Batching ---
#define FLASH_BATCH_SIZE   20
#define FLASH_SYNC_SAMPLES 100

// --- Launch Detection ---
#define LAUNCH_ACCEL_THRESHOLD  19.6f   // 2g in m/s²
#define LAUNCH_CONFIRM_SAMPLES  5       // 50ms at 100Hz

// --- Landing Detection ---
#define LANDING_ACCEL_LOW   7.0f
#define LANDING_ACCEL_HIGH  12.5f
#define LANDING_PRESSURE_VARIANCE  50.0f
#define LANDING_STABLE_SECS  5
#define LANDING_STABLE_SAMPLES (FAST_RATE_HZ * LANDING_STABLE_SECS)
#define MIN_FLIGHT_SECS    3
#define MIN_FLIGHT_SAMPLES (FAST_RATE_HZ * MIN_FLIGHT_SECS)

// --- File Paths ---
#define PREFLIGHT_FILE     "/preflight.bin"
#define FLIGHT_FILE        "/flight.bin"
#define META_FILE          "/meta.txt"

// --- WiFi AP ---
#define WIFI_AP_SSID       "RocketLogger"
#define WIFI_AP_PASS       "rocketdata"
#define WIFI_AP_IP         IPAddress(192, 168, 4, 1)

// --- Pyro Channel ---
#define PYRO_PIN           D5
#define PYRO_PULSE_MS      100
#define PYRO_DEPLOY_ALT_M  121.92f  // 400ft in meters

// --- LED ---
#define LED_PIN            LED_BUILTIN

// --- Arming ---
#define REQUIRE_ARMING     true

#endif
