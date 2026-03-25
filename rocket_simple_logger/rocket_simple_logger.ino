// ============================================================
//  ROCKET SIMPLE LOGGER
//  Arduino Nano ESP32 + BNO085 + DPS310
//
//  Autonomous flight logger - no serial commands needed.
//  Powers on -> ARMED -> detects launch -> logs flight ->
//  detects landing -> starts WiFi for data download.
//
//  LED patterns (ARMED state):
//    Steady blink         = 0/2 sensors connected
//    2 blinks, pause      = 1/2 sensors connected
//    3 blinks, pause      = 2/2 sensors connected (ready)
//
//  Pyro channel (D5) fires on descent through 400ft.
//
//  WiFi (auto-starts on landing at 192.168.4.1):
//    GET  /                    - Live dashboard with flight list
//    GET  /api/status          - JSON status (polled by dashboard)
//    GET  /api/flights         - JSON array of stored flights
//    GET  /data.csv?flight=N   - Download flight N data
//    GET  /preflight.csv?flight=N - Download preflight N data
//    GET  /delete?flight=N     - Delete flight N
// ============================================================

#include <Wire.h>
#include <FFat.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DPS310.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#include "config.h"
#include "data_types.h"
#include "ring_buffer.h"

// ============================================================
//  GLOBALS
// ============================================================

FlightState currentState = STATE_BOOT;

// Sensors
Adafruit_BNO08x bno;
Adafruit_DPS310 dps;
sh2_SensorValue_t bnoValue;
bool bnoAvailable = false;
bool dpsAvailable = false;
int sensorsConnected = 0;

float cur_accel[3]  = {0, 0, 0};
float cur_gyro[3]   = {0, 0, 0};
float cur_quat[4]   = {1, 0, 0, 0};
float cur_pressure  = 0;
float cur_temp      = 0;

// Ring buffer
RingBuffer ringBuf;

// Multi-flight storage
int currentFlightNum = 1;
char flightPath[24];
char preflightPath[28];
char metaPath[20];

File flightFile;
File preflightFile;
DataPoint flashBatch[FLASH_BATCH_SIZE];
int batchIndex = 0;
int flashSyncCounter = 0;

// Flight detection
int launchConfirmCount = 0;
int landingStableCount = 0;
uint32_t flightStartMs = 0;
uint32_t flightSampleCount = 0;

// Landing detection - rolling pressure
#define PRESSURE_WINDOW 100
float pressureWindow[PRESSURE_WINDOW];
int pressureWindowIdx = 0;
int pressureWindowCount = 0;

// Statistics
uint32_t preflightSamples = 0;
uint32_t flightSamplesWritten = 0;
uint32_t ringDrainedCount = 0;
float maxAccelMag = 0;
float minPressure = 999999;
float groundPressure = 0;
bool groundPressureSet = false;
float maxAltitude = 0;

// Timing
uint32_t lastSampleUs = 0;
uint32_t lastSlowLogMs = 0;


// Pyro channel
bool pyroFired = false;
bool pyroArmed = false;
float prevAltitude = 0;
bool pyroActive = false;
uint32_t pyroStartMs = 0;
int pyroConfirmCount = 0;

// WiFi
WebServer* webServer = nullptr;

// ============================================================
//  SENSOR INIT
// ============================================================

bool initBNO085() {
    if (!bno.begin_I2C(BNO085_I2C_ADDR, &Wire)) return false;
    bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, BNO_REPORT_INTERVAL_US);
    bno.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
    return true;
}

bool initDPS310() {
    if (!dps.begin_I2C(DPS310_I2C_ADDR, &Wire)) return false;
    dps.configurePressure(DPS310_64HZ, DPS310_8SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_8SAMPLES);
    dps.setMode(DPS310_CONT_PRESTEMP);
    return true;
}

// ============================================================
//  SENSOR READING
// ============================================================

void enableBNOReports() {
    bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, BNO_REPORT_INTERVAL_US);
    bno.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
}

void readSensors() {
    if (bnoAvailable) {
        if (bno.wasReset()) enableBNOReports();
        while (bno.getSensorEvent(&bnoValue)) {
            switch (bnoValue.sensorId) {
                case SH2_ACCELEROMETER:
                    cur_accel[0] = bnoValue.un.accelerometer.x;
                    cur_accel[1] = bnoValue.un.accelerometer.y;
                    cur_accel[2] = bnoValue.un.accelerometer.z;
                    break;
                case SH2_GYROSCOPE_CALIBRATED:
                    cur_gyro[0] = bnoValue.un.gyroscope.x;
                    cur_gyro[1] = bnoValue.un.gyroscope.y;
                    cur_gyro[2] = bnoValue.un.gyroscope.z;
                    break;
                case SH2_ROTATION_VECTOR:
                    cur_quat[0] = bnoValue.un.rotationVector.real;
                    cur_quat[1] = bnoValue.un.rotationVector.i;
                    cur_quat[2] = bnoValue.un.rotationVector.j;
                    cur_quat[3] = bnoValue.un.rotationVector.k;
                    break;
            }
        }
    }

    if (dpsAvailable) {
        sensors_event_t temp_event, pressure_event;
        if (dps.temperatureAvailable() && dps.pressureAvailable()) {
            dps.getEvents(&temp_event, &pressure_event);
            cur_pressure = pressure_event.pressure * 100.0f;
            cur_temp = temp_event.temperature;
        }
    }
}

// ============================================================
//  ALTITUDE
// ============================================================

float calculateAltitude(float pressure_pa) {
    if (!groundPressureSet || groundPressure <= 0 || pressure_pa <= 0) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / groundPressure, 0.1903f));
}

// ============================================================
//  BUILD DATAPOINT
// ============================================================

DataPoint buildDataPoint() {
    DataPoint dp;
    dp.timestamp_ms = millis();
    dp.accel_x = cur_accel[0];
    dp.accel_y = cur_accel[1];
    dp.accel_z = cur_accel[2];
    dp.gyro_x = cur_gyro[0];
    dp.gyro_y = cur_gyro[1];
    dp.gyro_z = cur_gyro[2];
    dp.quat_w = cur_quat[0];
    dp.quat_x = cur_quat[1];
    dp.quat_y = cur_quat[2];
    dp.quat_z = cur_quat[3];
    dp.pressure_pa = cur_pressure;
    dp.temperature_c = cur_temp;
    dp.altitude_m = calculateAltitude(cur_pressure);
    dp.d5 = digitalRead(PYRO_PIN) ? 1 : 0;
    return dp;
}

// ============================================================
//  FLIGHT DETECTION
// ============================================================

float accelMagnitude() {
    return sqrtf(cur_accel[0] * cur_accel[0] +
                 cur_accel[1] * cur_accel[1] +
                 cur_accel[2] * cur_accel[2]);
}

bool checkLaunch() {
    float mag = accelMagnitude();
    if (mag > LAUNCH_ACCEL_THRESHOLD) {
        launchConfirmCount++;
    } else {
        launchConfirmCount = 0;
    }
    return (launchConfirmCount >= LAUNCH_CONFIRM_SAMPLES);
}

void updatePressureWindow(float pressure) {
    pressureWindow[pressureWindowIdx] = pressure;
    pressureWindowIdx = (pressureWindowIdx + 1) % PRESSURE_WINDOW;
    if (pressureWindowCount < PRESSURE_WINDOW) pressureWindowCount++;
}

float pressureVariance() {
    if (pressureWindowCount < 10) return 99999.0f;
    float mean = 0;
    for (int i = 0; i < pressureWindowCount; i++) mean += pressureWindow[i];
    mean /= pressureWindowCount;
    float var = 0;
    for (int i = 0; i < pressureWindowCount; i++) {
        float diff = pressureWindow[i] - mean;
        var += diff * diff;
    }
    return var / pressureWindowCount;
}

bool checkLanding() {
    if (flightSampleCount < MIN_FLIGHT_SAMPLES) return false;
    float mag = accelMagnitude();
    bool accelStable = (mag > LANDING_ACCEL_LOW && mag < LANDING_ACCEL_HIGH);
    bool pressureStable = (pressureVariance() < LANDING_PRESSURE_VARIANCE);
    if (accelStable && pressureStable) {
        landingStableCount++;
    } else {
        landingStableCount = 0;
    }
    return (landingStableCount >= LANDING_STABLE_SAMPLES);
}

// ============================================================
//  FLASH STORAGE
// ============================================================

bool initStorage() {
    if (!FFat.begin(true)) return false;
    return true;
}

void setFlightPaths(int num) {
    snprintf(flightPath, sizeof(flightPath), FLIGHT_FMT, num);
    snprintf(preflightPath, sizeof(preflightPath), PREFLIGHT_FMT, num);
    snprintf(metaPath, sizeof(metaPath), META_FMT, num);
}

int scanNextFlightNumber() {
    int maxNum = 0;
    File root = FFat.open("/");
    File f = root.openNextFile();
    while (f) {
        String name = f.name();
        if (name.startsWith("flight_") && name.endsWith(".bin")) {
            int num = name.substring(7, 9).toInt();
            if (num > maxNum) maxNum = num;
        }
        f = root.openNextFile();
    }
    return maxNum + 1;
}

void openPreflightFile() {
    preflightFile = FFat.open(preflightPath, "a");
}

void openFlightFile() {
    flightFile = FFat.open(flightPath, "a");
}

void writeFlightSample(const DataPoint& dp) {
    if (flightFile) {
        flightFile.write((const uint8_t*)&dp, sizeof(DataPoint));
    }
}

void writePreflightSample(const DataPoint& dp) {
    if (preflightFile) {
        preflightFile.write((const uint8_t*)&dp, sizeof(DataPoint));
        preflightSamples++;
    }
}

void batchWriteToFlash(const DataPoint& dp) {
    float mag = accelMagnitude();
    if (mag > maxAccelMag) maxAccelMag = mag;
    if (cur_pressure > 0 && cur_pressure < minPressure) minPressure = cur_pressure;
    float alt = calculateAltitude(cur_pressure);
    if (alt > maxAltitude) maxAltitude = alt;

    flashBatch[batchIndex++] = dp;
    flashSyncCounter++;

    if (batchIndex >= FLASH_BATCH_SIZE) {
        if (flightFile) {
            flightFile.write((const uint8_t*)flashBatch,
                             sizeof(DataPoint) * batchIndex);
            flightSamplesWritten += batchIndex;
        }
        batchIndex = 0;
    }

    if (flashSyncCounter >= FLASH_SYNC_SAMPLES) {
        if (flightFile) flightFile.flush();
        flashSyncCounter = 0;
    }
}

void flushBatch() {
    if (batchIndex > 0 && flightFile) {
        flightFile.write((const uint8_t*)flashBatch,
                         sizeof(DataPoint) * batchIndex);
        flightSamplesWritten += batchIndex;
        batchIndex = 0;
    }
    if (flightFile) flightFile.flush();
}

void writeMetadata() {
    File meta = FFat.open(metaPath, "w");
    if (meta) {
        meta.printf("flight_number=%d\n", currentFlightNum);
        meta.printf("flight_start_ms=%u\n", flightStartMs);
        meta.printf("flight_duration_ms=%u\n", millis() - flightStartMs);
        meta.printf("preflight_samples=%u\n", preflightSamples);
        meta.printf("ring_drained_samples=%u\n", ringDrainedCount);
        meta.printf("flight_samples_written=%u\n", flightSamplesWritten);
        meta.printf("total_flight_samples=%u\n", ringDrainedCount + flightSamplesWritten);
        meta.printf("max_accel_ms2=%.2f\n", maxAccelMag);
        meta.printf("max_accel_g=%.2f\n", maxAccelMag / 9.81f);
        meta.printf("min_pressure_pa=%.1f\n", minPressure);
        meta.printf("ground_pressure_pa=%.1f\n", groundPressure);
        meta.printf("max_altitude_m=%.2f\n", maxAltitude);
        meta.close();
    }
}

// ============================================================
//  LED PATTERNS
// ============================================================

void updateLEDPattern() {
    uint32_t now = millis();

    if (currentState == STATE_FLIGHT) {
        digitalWrite(LED_PIN, HIGH);
        return;
    }

    if (currentState == STATE_LANDED) {
        // Slow blink
        uint32_t phase = (now / 1000) % 2;
        digitalWrite(LED_PIN, phase == 0 ? HIGH : LOW);
        return;
    }

    if (currentState == STATE_BOOT) {
        // Fast blink during boot
        uint32_t phase = (now / 100) % 2;
        digitalWrite(LED_PIN, phase == 0 ? HIGH : LOW);
        return;
    }

    // STATE_ARMED — sensor-count pattern
    if (sensorsConnected == 0) {
        // Steady 500ms blink
        uint32_t phase = (now / 500) % 2;
        digitalWrite(LED_PIN, phase == 0 ? HIGH : LOW);
        return;
    }

    // Pulsed blinks: sensorsConnected+1 blinks, then pause
    int pulseCount = sensorsConnected + 1;  // 2 or 3
    // Each blink = LED_BLINK_ON_MS on + LED_BLINK_OFF_MS off
    int blinkCycleMs = LED_BLINK_ON_MS + LED_BLINK_OFF_MS;
    int patternMs = pulseCount * blinkCycleMs + LED_PAUSE_MS;
    int pos = now % patternMs;

    int blinkPhaseEnd = pulseCount * blinkCycleMs;
    if (pos >= blinkPhaseEnd) {
        // In the pause
        digitalWrite(LED_PIN, LOW);
    } else {
        // Which blink are we in?
        int withinBlink = pos % blinkCycleMs;
        if (withinBlink < LED_BLINK_ON_MS) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
    }
}

// ============================================================
//  PYRO CHANNEL
// ============================================================

void firePyro() {
    digitalWrite(PYRO_PIN, HIGH);
    pyroActive = true;
    pyroStartMs = millis();
}

void checkPyroTimeout() {
    if (pyroActive && (millis() - pyroStartMs >= PYRO_PULSE_MS)) {
        digitalWrite(PYRO_PIN, LOW);
        pyroActive = false;
    }
}

void checkPyroDeploy(float altitude) {
    if (pyroFired) return;

    if (altitude > PYRO_DEPLOY_ALT_M) {
        pyroArmed = true;
    }

    if (pyroArmed && altitude <= PYRO_DEPLOY_ALT_M && altitude < prevAltitude) {
        pyroConfirmCount++;
        if (pyroConfirmCount >= 4) {
            firePyro();
            pyroFired = true;
        }
    } else {
        pyroConfirmCount = 0;
    }

    prevAltitude = altitude;
}

// ============================================================
//  WIFI DASHBOARD
// ============================================================

String buildStatusJSON() {
    float mag = accelMagnitude();
    float alt = calculateAltitude(cur_pressure);
    String json = "{";
    json += "\"state\":\"" + String(stateName(currentState)) + "\",";
    json += "\"bno085\":\"" + String(bnoAvailable ? "OK" : "MISSING") + "\",";
    json += "\"dps310\":\"" + String(dpsAvailable ? "OK" : "MISSING") + "\",";
    json += "\"uptime_ms\":" + String(millis()) + ",";
    json += "\"accel_ms2\":" + String(mag, 2) + ",";
    json += "\"accel_g\":" + String(mag / 9.81f, 2) + ",";
    json += "\"accel_x\":" + String(cur_accel[0], 4) + ",";
    json += "\"accel_y\":" + String(cur_accel[1], 4) + ",";
    json += "\"accel_z\":" + String(cur_accel[2], 4) + ",";
    json += "\"gyro_x\":" + String(cur_gyro[0], 4) + ",";
    json += "\"gyro_y\":" + String(cur_gyro[1], 4) + ",";
    json += "\"gyro_z\":" + String(cur_gyro[2], 4) + ",";
    json += "\"quat_w\":" + String(cur_quat[0], 4) + ",";
    json += "\"quat_x\":" + String(cur_quat[1], 4) + ",";
    json += "\"quat_y\":" + String(cur_quat[2], 4) + ",";
    json += "\"quat_z\":" + String(cur_quat[3], 4) + ",";
    json += "\"pressure_pa\":" + String(cur_pressure, 1) + ",";
    json += "\"temperature_c\":" + String(cur_temp, 2) + ",";
    json += "\"altitude_m\":" + String(alt, 2) + ",";
    json += "\"ground_pressure_pa\":" + String(groundPressure, 1) + ",";
    json += "\"max_accel_ms2\":" + String(maxAccelMag, 2) + ",";
    json += "\"max_accel_g\":" + String(maxAccelMag / 9.81f, 2) + ",";
    json += "\"max_altitude_m\":" + String(maxAltitude, 2) + ",";
    json += "\"flight_duration_ms\":" + String(millis() - flightStartMs) + ",";
    json += "\"preflight_samples\":" + String(preflightSamples) + ",";
    json += "\"flight_samples\":" + String(ringDrainedCount + flightSamplesWritten) + ",";
    json += "\"current_flight\":" + String(currentFlightNum) + ",";
    json += "\"flash_used_kb\":" + String(FFat.usedBytes() / 1024) + ",";
    json += "\"flash_total_kb\":" + String(FFat.totalBytes() / 1024) + ",";
    json += "\"flash_free_kb\":" + String(FFat.freeBytes() / 1024);
    json += "}";
    return json;
}

void streamCSV(WebServer& server, const char* path) {
    File f = FFat.open(path, "r");
    if (!f) return;
    DataPoint dp;
    char line[256];
    while (f.read((uint8_t*)&dp, sizeof(DataPoint)) == sizeof(DataPoint)) {
        int len = snprintf(line, sizeof(line),
            "%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.2f,%.2f,%u\n",
            dp.timestamp_ms,
            dp.accel_x, dp.accel_y, dp.accel_z,
            dp.gyro_x, dp.gyro_y, dp.gyro_z,
            dp.quat_w, dp.quat_x, dp.quat_y, dp.quat_z,
            dp.pressure_pa, dp.temperature_c, dp.altitude_m,
            dp.d5);
        server.sendContent(line, len);
    }
    f.close();
}

void startWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_IP, IPAddress(255, 255, 255, 0));
    if (strlen(WIFI_AP_PASS) >= 8) {
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    } else {
        WiFi.softAP(WIFI_AP_SSID);
    }

    webServer = new WebServer(80);

    // --- Dashboard ---
    webServer->on("/", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head><title>Rocket Logger</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>";
        html += "body{font-family:monospace;margin:0;padding:1.5em;background:#111;color:#0f0;}";
        html += "h1{margin-top:0;}h2{color:#0ff;margin-top:1.5em;}";
        html += "#status{background:#1a1a1a;border:1px solid #0f0;padding:1em;margin:1em 0;white-space:pre;font-size:0.95em;line-height:1.6;}";
        html += ".flight{background:#1a1a1a;border:1px solid #333;padding:0.8em;margin:0.5em 0;}";
        html += ".complete{border-color:#0f0;}.partial{border-color:#ff0;}";
        html += "a{color:#0ff;display:inline-block;margin:0.2em 0.5em 0.2em 0;}";
        html += "a:hover{color:#fff;}";
        html += "a.del{color:#f44;}a.del:hover{color:#f88;}";
        html += "</style></head><body>";
        html += "<h1>Rocket Flight Data Logger</h1>";

        // Live sensor status
        html += "<h2>Live Sensor Data</h2>";
        html += "<div id='status'>Loading...</div>";

        // Flight list
        html += "<h2>Stored Flights</h2>";
        html += "<div id='flights'>Loading flights...</div>";

        // JS for live updates + flight list
        html += "<script>";
        html += "function update(){fetch('/api/status').then(r=>r.json()).then(d=>{";
        html += "let s='State: '+d.state+' (Flight #'+d.current_flight+')\\n';";
        html += "s+='Sensors: BNO085='+d.bno085+' DPS310='+d.dps310+'\\n';";
        html += "s+='Uptime: '+(d.uptime_ms/1000).toFixed(1)+' s\\n';";
        html += "s+='\\n--- Accelerometer ---\\n';";
        html += "s+='  Magnitude: '+d.accel_ms2+' m/s2 ('+d.accel_g+'g)\\n';";
        html += "s+='  X='+d.accel_x+' Y='+d.accel_y+' Z='+d.accel_z+'\\n';";
        html += "s+='\\n--- Gyroscope ---\\n';";
        html += "s+='  X='+d.gyro_x+' Y='+d.gyro_y+' Z='+d.gyro_z+'\\n';";
        html += "s+='\\n--- Orientation (Quaternion) ---\\n';";
        html += "s+='  W='+d.quat_w+' X='+d.quat_x+' Y='+d.quat_y+' Z='+d.quat_z+'\\n';";
        html += "s+='\\n--- Barometer ---\\n';";
        html += "s+='  Pressure: '+d.pressure_pa+' Pa\\n';";
        html += "s+='  Temperature: '+d.temperature_c+' C\\n';";
        html += "s+='  Altitude: '+d.altitude_m+' m\\n';";
        html += "s+='  Ground Pressure: '+d.ground_pressure_pa+' Pa\\n';";
        html += "s+='\\n--- Flight Stats ---\\n';";
        html += "s+='  Max Accel: '+d.max_accel_ms2+' m/s2 ('+d.max_accel_g+'g)\\n';";
        html += "s+='  Max Altitude: '+d.max_altitude_m+' m\\n';";
        html += "s+='  Flight Duration: '+(d.flight_duration_ms/1000).toFixed(1)+' s\\n';";
        html += "s+='  Preflight Samples: '+d.preflight_samples+'\\n';";
        html += "s+='  Flight Samples: '+d.flight_samples+'\\n';";
        html += "s+='  Flash: '+d.flash_used_kb+'/'+d.flash_total_kb+' KB ('+d.flash_free_kb+' KB free)';";
        html += "document.getElementById('status').textContent=s;";
        html += "}).catch(()=>{document.getElementById('status').textContent='Connection lost...';})}";
        html += "setInterval(update,1500);update();";

        // Load flight list once
        html += "function loadFlights(){fetch('/api/flights').then(r=>r.json()).then(flights=>{";
        html += "if(flights.length===0){document.getElementById('flights').textContent='No flights recorded.';return;}";
        html += "let h='';flights.sort((a,b)=>b.num-a.num);";
        html += "flights.forEach(f=>{";
        html += "let kb=(f.size/1024).toFixed(1);";
        html += "let cls=f.complete?'flight complete':'flight partial';";
        html += "let tag=f.complete?'Complete':'Partial';";
        html += "h+='<div class=\"'+cls+'\">';";
        html += "h+='Flight #'+f.num+' - '+kb+' KB - '+tag+'<br>';";
        html += "h+='<a href=\"/data.csv?flight='+f.num+'\">Flight CSV</a>';";
        html += "h+='<a href=\"/preflight.csv?flight='+f.num+'\">Preflight CSV</a>';";
        html += "h+='<a class=\"del\" href=\"#\" onclick=\"if(confirm(\\'Delete flight '+f.num+'?\\'))fetch(\\'/delete?flight='+f.num+'\\').then(()=>loadFlights());return false;\">Delete</a>';";
        html += "h+='</div>';});";
        html += "document.getElementById('flights').innerHTML=h;";
        html += "}).catch(()=>{document.getElementById('flights').textContent='Error loading flights.';})}";
        html += "loadFlights();";

        html += "</script></body></html>";
        webServer->send(200, "text/html", html);
    });

    // --- Flights API ---
    webServer->on("/api/flights", HTTP_GET, []() {
        String json = "[";
        File root = FFat.open("/");
        File f = root.openNextFile();
        bool first = true;
        while (f) {
            String name = f.name();
            if (name.startsWith("flight_") && name.endsWith(".bin")) {
                int num = name.substring(7, 9).toInt();
                if (!first) json += ",";
                first = false;
                char mp[20];
                snprintf(mp, sizeof(mp), META_FMT, num);
                bool hasMeta = FFat.exists(mp);
                json += "{\"num\":" + String(num);
                json += ",\"size\":" + String(f.size());
                json += ",\"complete\":" + String(hasMeta ? "true" : "false");
                json += "}";
            }
            f = root.openNextFile();
        }
        json += "]";
        webServer->send(200, "application/json", json);
    });

    // --- CSV endpoints ---
    webServer->on("/data.csv", HTTP_GET, []() {
        int num = webServer->arg("flight").toInt();
        if (num < 1 || num > MAX_FLIGHTS) {
            webServer->send(400, "text/plain", "Invalid flight number");
            return;
        }
        char path[24];
        snprintf(path, sizeof(path), FLIGHT_FMT, num);
        if (!FFat.exists(path)) {
            webServer->send(404, "text/plain", "Flight not found");
            return;
        }
        webServer->setContentLength(CONTENT_LENGTH_UNKNOWN);
        webServer->send(200, "text/csv", "");
        webServer->sendContent(CSV_HEADER "\n");
        streamCSV(*webServer, path);
    });

    webServer->on("/preflight.csv", HTTP_GET, []() {
        int num = webServer->arg("flight").toInt();
        if (num < 1 || num > MAX_FLIGHTS) {
            webServer->send(400, "text/plain", "Invalid flight number");
            return;
        }
        char path[28];
        snprintf(path, sizeof(path), PREFLIGHT_FMT, num);
        if (!FFat.exists(path)) {
            webServer->send(404, "text/plain", "Preflight data not found");
            return;
        }
        webServer->setContentLength(CONTENT_LENGTH_UNKNOWN);
        webServer->send(200, "text/csv", "");
        webServer->sendContent(CSV_HEADER "\n");
        streamCSV(*webServer, path);
    });

    // --- Delete endpoint ---
    webServer->on("/delete", HTTP_GET, []() {
        int num = webServer->arg("flight").toInt();
        if (num < 1 || num > MAX_FLIGHTS) {
            webServer->send(400, "text/plain", "Invalid flight number");
            return;
        }
        char p1[24], p2[28], p3[20];
        snprintf(p1, sizeof(p1), FLIGHT_FMT, num);
        snprintf(p2, sizeof(p2), PREFLIGHT_FMT, num);
        snprintf(p3, sizeof(p3), META_FMT, num);
        if (FFat.exists(p1)) FFat.remove(p1);
        if (FFat.exists(p2)) FFat.remove(p2);
        if (FFat.exists(p3)) FFat.remove(p3);
        webServer->send(200, "text/plain", "Deleted flight " + String(num));
    });

    // --- Status API ---
    webServer->on("/api/status", HTTP_GET, []() {
        webServer->send(200, "application/json", buildStatusJSON());
    });

    webServer->begin();
}

// ============================================================
//  STATE TRANSITIONS
// ============================================================

void transitionToFlight() {
    flightStartMs = millis();
    flightSampleCount = 0;
    landingStableCount = 0;
    pressureWindowCount = 0;
    batchIndex = 0;
    flashSyncCounter = 0;
    ringDrainedCount = 0;
    flightSamplesWritten = 0;
    maxAccelMag = 0;
    minPressure = 999999;
    maxAltitude = 0;
    pyroFired = false;
    pyroArmed = false;
    prevAltitude = 0;
    pyroConfirmCount = 0;

    if (preflightFile) preflightFile.close();
    openFlightFile();

    ringDrainedCount = ringBuf.drain(writeFlightSample);
    if (flightFile) flightFile.flush();

    currentState = STATE_FLIGHT;
}

void transitionToLanded() {
    flushBatch();
    if (flightFile) flightFile.close();
    writeMetadata();

    currentState = STATE_LANDED;

    // Auto-start WiFi for data download
    startWiFi();
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Pyro — explicitly LOW at boot
    pinMode(PYRO_PIN, OUTPUT);
    digitalWrite(PYRO_PIN, LOW);

    // I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    // Storage
    if (!initStorage()) {
        // Fatal — fast blink forever
        while (true) {
            digitalWrite(LED_PIN, (millis() / 100) % 2 ? HIGH : LOW);
        }
    }

    // Determine flight number for this session
    currentFlightNum = scanNextFlightNumber();
    setFlightPaths(currentFlightNum);

    // Sensors
    bnoAvailable = initBNO085();
    dpsAvailable = initDPS310();
    sensorsConnected = (bnoAvailable ? 1 : 0) + (dpsAvailable ? 1 : 0);

    // Ground pressure calibration
    if (dpsAvailable) {
        // Discard warmup readings
        for (int i = 0; i < 20; i++) {
            sensors_event_t t, p;
            if (dps.temperatureAvailable() && dps.pressureAvailable()) {
                dps.getEvents(&t, &p);
            }
            delay(100);
        }
        float pressureSum = 0;
        int goodReadings = 0;
        while (goodReadings < 50) {
            sensors_event_t t, p;
            if (dps.temperatureAvailable() && dps.pressureAvailable()) {
                dps.getEvents(&t, &p);
                float pa = p.pressure * 100.0f;
                if (pa > 50000) {
                    pressureSum += pa;
                    goodReadings++;
                }
            }
            delay(50);
        }
        groundPressure = pressureSum / goodReadings;
        groundPressureSet = true;
    }

    // Open preflight log
    openPreflightFile();

    // Go straight to ARMED
    currentState = STATE_ARMED;
    lastSampleUs = micros();
    lastSlowLogMs = millis();
}

// ============================================================
//  MAIN LOOP
// ============================================================

void checkSerialCommand() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "LAND" && currentState == STATE_ARMED) {
            if (preflightFile) preflightFile.close();
            currentState = STATE_LANDED;
            startWiFi();
        }
    }
}

void loop() {
    uint32_t nowUs = micros();

    if (webServer) webServer->handleClient();

    updateLEDPattern();
    checkPyroTimeout();
    checkSerialCommand();

    // Continue reading sensors even after landing (for live dashboard)
    if (nowUs - lastSampleUs >= SAMPLE_INTERVAL_US) {
        lastSampleUs = nowUs;

        readSensors();
        DataPoint dp = buildDataPoint();

        switch (currentState) {

            case STATE_ARMED:
                ringBuf.push(dp);
                if (millis() - lastSlowLogMs >= (1000 / SLOW_LOG_HZ)) {
                    lastSlowLogMs = millis();
                    writePreflightSample(dp);
                }
                if (checkLaunch()) {
                    transitionToFlight();
                }
                break;

            case STATE_FLIGHT:
                flightSampleCount++;
                batchWriteToFlash(dp);
                updatePressureWindow(cur_pressure);
                checkPyroDeploy(dp.altitude_m);
                if (checkLanding()) {
                    transitionToLanded();
                }
                break;

            case STATE_LANDED:
                // Sensors keep reading for live dashboard — no storage
                break;

            default:
                break;
        }
    }
}
