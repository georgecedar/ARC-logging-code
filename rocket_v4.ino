// ============================================================
//  ROCKET FLIGHT DATA LOGGER v4
//  Arduino Nano ESP32 + BNO085 + DPS310
//
//  Storage: FFat on internal flash (default partition)
//  Upload:  Sketch -> Upload Using Programmer (Esptool)
//
//  Serial Commands (115200 baud):
//    STATUS   - Print current state and stats
//    ARM      - Arm launch detection (if REQUIRE_ARMING)
//    DOWNLOAD - Dump all data as CSV over serial
//    WIFI     - Start WiFi AP for wireless access
//    FORMAT   - Erase all stored data
//    RESET    - Reset state machine to IDLE
//    TEST [s] - Start test flight for [s] seconds (default: no limit)
//    STOP     - End test flight
//    FIRE     - Test fire pyro channel (double confirmation required)
//
//  WiFi API (when AP is active at 192.168.4.1):
//    GET  /              - Live dashboard with status + command buttons
//    GET  /api/status    - JSON status (polled by dashboard)
//    POST /api/arm       - Arm launch detection (shuts down WiFi)
//    POST /api/reset     - Reset state machine
//    POST /api/format    - Erase all stored data
//    POST /api/test      - Start test flight (param: seconds)
//    POST /api/stop      - End test flight
//    POST /api/fire      - Test fire pyro (two POSTs required)
//    POST /api/fire/cancel - Cancel pending fire confirmation
//    GET  /data.csv      - Download flight data
//    GET  /preflight.csv - Download preflight data
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

float cur_accel[3]  = {0, 0, 0};
float cur_gyro[3]   = {0, 0, 0};
float cur_quat[4]   = {1, 0, 0, 0};
float cur_pressure  = 0;
float cur_temp      = 0;

// Ring buffer
RingBuffer ringBuf;

// Flash storage
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
uint32_t lastLedToggleMs = 0;
bool ledState = false;

// Pyro channel
bool pyroFired = false;          // has fired this flight
bool pyroArmed = false;          // passed above deploy altitude
float prevAltitude = 0;          // for descent detection
bool pyroActive = false;         // pin is currently HIGH
uint32_t pyroStartMs = 0;       // when the pulse started

// Test flight
bool testFlightActive = false;
uint32_t testFlightEndMs = 0;  // 0 = no auto-stop (manual STOP only)

// WiFi
WebServer* webServer = nullptr;
bool wifiShutdownPending = false;
int fireConfirmStep = 0;   // 0=idle, 1=awaiting second confirm
int armConfirmStep = 0;    // 0=idle, 1=awaiting confirm

// ============================================================
//  SERIAL CONFIRMATION PROMPT
// ============================================================

bool waitForProceed(const char* sensorName) {
    Serial.printf("Type PROCEED to continue without %s, or reset to retry.\n", sensorName);
    while (true) {
        updateLED();
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            input.toUpperCase();
            if (input == "PROCEED") {
                Serial.printf("Continuing without %s.\n", sensorName);
                return true;
            } else {
                Serial.printf("Unknown input. Type PROCEED to continue without %s.\n", sensorName);
            }
        }
    }
}

// ============================================================
//  SENSOR INIT
// ============================================================

bool initBNO085() {
    Serial.print("Initializing BNO085... ");
    if (!bno.begin_I2C(BNO085_I2C_ADDR, &Wire)) {
        Serial.println("FAILED!");
        return false;
    }
    enableBNOReports();
    Serial.println("OK");
    return true;
}

bool initDPS310() {
    Serial.print("Initializing DPS310... ");
    if (!dps.begin_I2C(DPS310_I2C_ADDR, &Wire)) {
        Serial.println("FAILED!");
        return false;
    }
    dps.configurePressure(DPS310_64HZ, DPS310_8SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_8SAMPLES);
    dps.setMode(DPS310_CONT_PRESTEMP);
    Serial.println("OK");
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
        if (bno.wasReset()) {
            enableBNOReports();
        }
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
    Serial.print("Mounting FFat... ");
    if (!FFat.begin(true)) {
        Serial.println("FAILED!");
        return false;
    }
    size_t total = FFat.totalBytes();
    size_t used = FFat.usedBytes();
    Serial.printf("OK (%u KB total, %u KB used, %u KB free)\n",
                  total / 1024, used / 1024, (total - used) / 1024);
    return true;
}

void openPreflightFile() {
    preflightFile = FFat.open(PREFLIGHT_FILE, "a");
    if (!preflightFile) Serial.println("ERROR: Could not open preflight file");
}

void openFlightFile() {
    flightFile = FFat.open(FLIGHT_FILE, "w");
    if (!flightFile) Serial.println("ERROR: Could not open flight file");
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
    File meta = FFat.open(META_FILE, "w");
    if (meta) {
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
        Serial.println("Metadata saved.");
    }
}

void formatStorage() {
    Serial.println("Formatting FFat...");
    FFat.end();
    bool ok = FFat.format();
    Serial.printf("Format %s\n", ok ? "OK" : "FAILED");
    if (!FFat.begin(true)) {
        Serial.println("ERROR: FFat mount failed after format");
    }
    Serial.printf("Flash after format: %u / %u KB used (%u KB free)\n",
                  FFat.usedBytes() / 1024, FFat.totalBytes() / 1024,
                  (FFat.totalBytes() - FFat.usedBytes()) / 1024);
    Serial.println("Done.");
}

// ============================================================
//  CSV DOWNLOAD - Serial
// ============================================================

void printDataPointCSV(const DataPoint& dp) {
    Serial.printf("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.2f,%.2f,%u\n",
        dp.timestamp_ms,
        dp.accel_x, dp.accel_y, dp.accel_z,
        dp.gyro_x, dp.gyro_y, dp.gyro_z,
        dp.quat_w, dp.quat_x, dp.quat_y, dp.quat_z,
        dp.pressure_pa, dp.temperature_c, dp.altitude_m,
        dp.d5);
}

void dumpFileCSV(const char* path) {
    File f = FFat.open(path, "r");
    if (!f) return;
    DataPoint dp;
    while (f.read((uint8_t*)&dp, sizeof(DataPoint)) == sizeof(DataPoint)) {
        printDataPointCSV(dp);
    }
    f.close();
}

void serialDownload() {
    if (preflightFile) preflightFile.flush();
    if (flightFile) flightFile.flush();

    Serial.println("# ====== ROCKET DATA DOWNLOAD ======");

    // Metadata
    File meta = FFat.open(META_FILE, "r");
    if (meta) {
        while (meta.available()) {
            Serial.write(meta.read());
        }
        meta.close();
    }

    // Sample counts as comments
    File pf = FFat.open(PREFLIGHT_FILE, "r");
    if (pf) {
        Serial.printf("# Preflight: %d samples\n", pf.size() / sizeof(DataPoint));
        pf.close();
    }
    File ff = FFat.open(FLIGHT_FILE, "r");
    if (ff) {
        Serial.printf("# Flight: %d samples\n", ff.size() / sizeof(DataPoint));
        ff.close();
    }

    // CSV header then all data rows
    Serial.println(CSV_HEADER);
    dumpFileCSV(PREFLIGHT_FILE);
    dumpFileCSV(FLIGHT_FILE);

    Serial.println("# ====== END OF DATA ======");
}

// ============================================================
//  CSV DOWNLOAD - WiFi
// ============================================================

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

void stopWiFi() {
    if (webServer) {
        webServer->stop();
        delete webServer;
        webServer = nullptr;
    }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi stopped.");
}

void startWiFiDownload() {
    Serial.println("Starting WiFi Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_IP, IPAddress(255, 255, 255, 0));
    if (strlen(WIFI_AP_PASS) >= 8) {
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    } else {
        WiFi.softAP(WIFI_AP_SSID);
    }

    webServer = new WebServer(80);

    // --- Dashboard page ---
    webServer->on("/", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head><title>Rocket Logger</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>";
        html += "body{font-family:monospace;margin:0;padding:1.5em;background:#111;color:#0f0;}";
        html += "h1{margin-top:0;}";
        html += "#status{background:#1a1a1a;border:1px solid #0f0;padding:1em;margin:1em 0;white-space:pre;font-size:0.95em;line-height:1.6;}";
        html += ".btn{display:inline-block;padding:0.6em 1.2em;margin:0.3em;font-family:monospace;font-size:1em;cursor:pointer;border:2px solid;background:transparent;border-radius:4px;}";
        html += ".btn-arm{color:#ff0;border-color:#ff0;} .btn-arm:hover{background:#ff02;}";
        html += ".btn-reset{color:#f80;border-color:#f80;} .btn-reset:hover{background:#f802;}";
        html += ".btn-format{color:#f00;border-color:#f00;} .btn-format:hover{background:#f002;}";
        html += ".btn-test{color:#0f0;border-color:#0f0;} .btn-test:hover{background:#0f02;}";
        html += ".btn-stop{color:#f0f;border-color:#f0f;} .btn-stop:hover{background:#f0f2;}";
        html += "input[type=number]{background:#1a1a1a;color:#0f0;border:1px solid #0f0;font-family:monospace;padding:0.4em;width:4em;font-size:1em;border-radius:4px;}";
        html += ".btn:disabled{opacity:0.3;cursor:not-allowed;}";
        html += "a{color:#0ff;font-size:1.1em;}";
        html += "#meta{background:#1a1a1a;border:1px solid #555;padding:1em;margin:1em 0;white-space:pre;}";
        html += "#msg{color:#ff0;margin:0.5em 0;min-height:1.4em;}";
        html += "</style></head><body>";
        html += "<h1>Rocket Flight Data Logger</h1>";

        // Status area (updated by JS)
        html += "<div id='status'>Loading...</div>";
        html += "<div id='msg'></div>";

        // Command buttons
        html += "<div style='margin:1em 0;'>";
        html += "<button class='btn btn-arm' id='armBtn' onclick='armStep()'>ARM</button>";
        html += "<button class='btn btn-reset' id='cancelArm' style='display:none;' onclick='cancelArm()'>CANCEL</button>";
        html += "<button class='btn btn-reset' onclick='sendCmd(\"reset\")'>RESET</button>";
        html += "<button class='btn btn-format' onclick='confirmFormat()'>FORMAT</button>";
        html += "</div>";

        // Test flight controls
        html += "<div style='margin:1em 0;'>";
        html += "<input type='number' id='testSecs' value='10' min='1' max='300'> sec ";
        html += "<button class='btn btn-test' onclick='startTest()'>TEST FLIGHT</button> ";
        html += "<button class='btn btn-stop' onclick='sendCmd(\"stop\")'>STOP</button>";
        html += "</div>";

        // Pyro test fire
        html += "<div style='margin:1.5em 0;padding:1em;border:2px solid #f00;background:#1a0000;'>";
        html += "<b style='color:#f00;'>PYRO TEST FIRE</b><br><br>";
        html += "<button class='btn btn-format' id='fireBtn' onclick='fireStep()'>FIRE PYRO</button>";
        html += " <button class='btn btn-reset' id='cancelFire' style='display:none;' onclick='cancelFire()'>CANCEL</button>";
        html += "<div id='fireMsg' style='color:#f00;margin-top:0.5em;'></div>";
        html += "</div>";

        // Flight metadata
        html += "<h2>Flight Metadata</h2><div id='meta'>";
        File meta = FFat.open(META_FILE, "r");
        if (meta) {
            while (meta.available()) html += (char)meta.read();
            meta.close();
        } else {
            html += "No flight data recorded yet.";
        }
        html += "</div>";

        // Download links
        html += "<h2>Download</h2>";
        html += "<a href='/data.csv'>Flight Data (CSV)</a><br><br>";
        html += "<a href='/preflight.csv'>Preflight Data (CSV)</a>";

        // JavaScript for live updates and commands
        html += "<script>";
        html += "function update(){fetch('/api/status').then(r=>r.json()).then(d=>{";
        html += "let s='State: '+d.state+'\\n';";
        html += "s+='Sensors: BNO085='+d.bno085+' DPS310='+d.dps310+'\\n';";
        html += "s+='Uptime: '+(d.uptime_ms/1000).toFixed(1)+' s\\n';";
        html += "s+='Accel: '+d.accel_ms2+' m/s2 ('+d.accel_g+'g)\\n';";
        html += "s+='  X='+d.accel_x+' Y='+d.accel_y+' Z='+d.accel_z+'\\n';";
        html += "s+='Gyro: X='+d.gyro_x+' Y='+d.gyro_y+' Z='+d.gyro_z+'\\n';";
        html += "s+='Quat: W='+d.quat_w+' X='+d.quat_x+' Y='+d.quat_y+' Z='+d.quat_z+'\\n';";
        html += "s+='Pressure: '+d.pressure_pa+' Pa\\n';";
        html += "s+='Temperature: '+d.temperature_c+' C\\n';";
        html += "s+='Altitude: '+d.altitude_m+' m\\n';";
        html += "s+='Ring buffer: '+d.ring_buffer+'/'+d.ring_buffer_max+'\\n';";
        html += "s+='Preflight samples: '+d.preflight_samples+'\\n';";
        html += "s+='Flight samples: '+d.flight_samples+'\\n';";
        html += "s+='Flash: '+d.flash_used_kb+'/'+d.flash_total_kb+' KB';";
        html += "document.getElementById('status').textContent=s;";
        html += "}).catch(()=>{document.getElementById('status').textContent='Connection lost...';})}";
        html += "setInterval(update,1500);update();";
        html += "function showMsg(t){document.getElementById('msg').textContent=t;setTimeout(()=>{document.getElementById('msg').textContent='';},4000);}";
        html += "function sendCmd(c){fetch('/api/'+c,{method:'POST'}).then(r=>r.json()).then(d=>{showMsg(d.message);update();}).catch(()=>showMsg('Error'));}";
        html += "function confirmFormat(){if(confirm('Erase ALL stored data?'))sendCmd('format');}";
        html += "var armState=0;";
        html += "function armStep(){";
        html += "if(armState==0){if(!confirm('Arming will enable launch detection and pyro channel. Continue?'))return;}";
        html += "else if(armState==1){if(!confirm('CONFIRM: Arm the rocket? WiFi will shut down.'))return;}";
        html += "fetch('/api/arm',{method:'POST'}).then(r=>r.json()).then(d=>{";
        html += "showMsg(d.message);";
        html += "if(d.step==1){armState=1;document.getElementById('armBtn').textContent='CONFIRM ARM';document.getElementById('cancelArm').style.display='inline-block';}";
        html += "else{armState=0;document.getElementById('armBtn').textContent='ARM';document.getElementById('cancelArm').style.display='none';}";
        html += "}).catch(()=>showMsg('Error'));}";
        html += "function cancelArm(){fetch('/api/arm/cancel',{method:'POST'}).then(r=>r.json()).then(d=>{showMsg(d.message);armState=0;document.getElementById('armBtn').textContent='ARM';document.getElementById('cancelArm').style.display='none';});}";
        html += "function startTest(){var s=document.getElementById('testSecs').value||10;fetch('/api/test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'seconds='+s}).then(r=>r.json()).then(d=>{showMsg(d.message);update();}).catch(()=>showMsg('Error'));}";
        html += "var fireState=0;";
        html += "function fireStep(){";
        html += "if(fireState==0){if(!confirm('WARNING: This will fire the pyro channel on pin D5. Continue?'))return;}";
        html += "else if(fireState==1){if(!confirm('FINAL CONFIRMATION: Are you sure you want to FIRE?'))return;}";
        html += "fetch('/api/fire',{method:'POST'}).then(r=>r.json()).then(d=>{";
        html += "document.getElementById('fireMsg').textContent=d.message;";
        html += "if(d.step==1){fireState=1;document.getElementById('fireBtn').textContent='CONFIRM FIRE';document.getElementById('cancelFire').style.display='inline-block';}";
        html += "else{fireState=0;document.getElementById('fireBtn').textContent='FIRE PYRO';document.getElementById('cancelFire').style.display='none';}";
        html += "}).catch(()=>{document.getElementById('fireMsg').textContent='Error';});}";
        html += "function cancelFire(){fetch('/api/fire/cancel',{method:'POST'}).then(r=>r.json()).then(d=>{document.getElementById('fireMsg').textContent=d.message;fireState=0;document.getElementById('fireBtn').textContent='FIRE PYRO';document.getElementById('cancelFire').style.display='none';});}";
        html += "</script></body></html>";
        webServer->send(200, "text/html", html);
    });

    // --- CSV download endpoints ---
    webServer->on("/data.csv", HTTP_GET, []() {
        webServer->setContentLength(CONTENT_LENGTH_UNKNOWN);
        webServer->send(200, "text/csv", "");
        webServer->sendContent(CSV_HEADER "\n");
        streamCSV(*webServer, FLIGHT_FILE);
    });

    webServer->on("/preflight.csv", HTTP_GET, []() {
        webServer->setContentLength(CONTENT_LENGTH_UNKNOWN);
        webServer->send(200, "text/csv", "");
        webServer->sendContent(CSV_HEADER "\n");
        streamCSV(*webServer, PREFLIGHT_FILE);
    });

    // --- API endpoints ---
    webServer->on("/api/status", HTTP_GET, []() {
        webServer->send(200, "application/json", buildStatusJSON());
    });

    webServer->on("/api/arm", HTTP_POST, []() {
        if (currentState != STATE_IDLE && currentState != STATE_DOWNLOAD) {
            armConfirmStep = 0;
            webServer->send(200, "application/json",
                "{\"message\":\"Cannot arm in state " + String(stateName(currentState)) + "\",\"step\":0}");
            return;
        }
        if (armConfirmStep == 0) {
            armConfirmStep = 1;
            Serial.println("WiFi cmd: ARM step 1 - awaiting confirmation");
            webServer->send(200, "application/json",
                "{\"message\":\"Arming will enable launch detection and pyro. Confirm to ARM.\",\"step\":1}");
        } else if (armConfirmStep == 1) {
            armConfirmStep = 0;
            String result = cmdArm();
            Serial.println("WiFi cmd: ARM step 2 -> " + result);
            webServer->send(200, "application/json", "{\"message\":\"" + result + "\",\"step\":2}");
            if (currentState == STATE_ARMED) {
                wifiShutdownPending = true;
            }
        }
    });

    webServer->on("/api/arm/cancel", HTTP_POST, []() {
        armConfirmStep = 0;
        webServer->send(200, "application/json", "{\"message\":\"Arm cancelled.\"}");
    });

    webServer->on("/api/reset", HTTP_POST, []() {
        String result = cmdReset();
        Serial.println("WiFi cmd: RESET -> " + result);
        // Keep WiFi running — stay in DOWNLOAD state
        currentState = STATE_DOWNLOAD;
        webServer->send(200, "application/json", "{\"message\":\"" + result + "\"}");
    });

    webServer->on("/api/format", HTTP_POST, []() {
        String result = cmdFormat();
        Serial.println("WiFi cmd: FORMAT -> " + result);
        // Keep WiFi running — stay in DOWNLOAD state
        currentState = STATE_DOWNLOAD;
        webServer->send(200, "application/json", "{\"message\":\"" + result + "\"}");
    });

    webServer->on("/api/test", HTTP_POST, []() {
        int secs = 0;
        if (webServer->hasArg("seconds")) {
            secs = webServer->arg("seconds").toInt();
        }
        String result = cmdTest(secs);
        Serial.println("WiFi cmd: TEST -> " + result);
        webServer->send(200, "application/json", "{\"message\":\"" + result + "\"}");
    });

    webServer->on("/api/stop", HTTP_POST, []() {
        String result = cmdStop();
        Serial.println("WiFi cmd: STOP -> " + result);
        // Return to download state so WiFi stays active
        if (currentState == STATE_LANDED) {
            currentState = STATE_DOWNLOAD;
        }
        webServer->send(200, "application/json", "{\"message\":\"" + result + "\"}");
    });

    // Test fire — two-step confirmation via HTTP
    webServer->on("/api/fire", HTTP_POST, []() {
        if (currentState == STATE_ARMED || currentState == STATE_FLIGHT) {
            fireConfirmStep = 0;
            webServer->send(200, "application/json",
                "{\"message\":\"DENIED: Cannot test fire in state " + String(stateName(currentState)) + "\",\"step\":0}");
            return;
        }
        if (fireConfirmStep == 0) {
            fireConfirmStep = 1;
            Serial.println("WiFi cmd: FIRE step 1 - awaiting final confirmation");
            webServer->send(200, "application/json",
                "{\"message\":\"WARNING: About to fire pyro channel. Confirm again to FIRE.\",\"step\":1}");
        } else if (fireConfirmStep == 1) {
            fireConfirmStep = 0;
            Serial.println("WiFi cmd: FIRE step 2 - FIRING (recording 2s)");
            cmdTest(2);
            firePyro();
            Serial.println("Pyro fired via WiFi. Recording for 2 seconds...");
            webServer->send(200, "application/json",
                "{\"message\":\"PYRO FIRED. Recording 2s of data.\",\"step\":2}");
        }
    });

    // Cancel a pending fire confirmation
    webServer->on("/api/fire/cancel", HTTP_POST, []() {
        fireConfirmStep = 0;
        webServer->send(200, "application/json", "{\"message\":\"Fire cancelled.\"}");
    });

    webServer->begin();
    Serial.printf("WiFi AP started: SSID='%s'\n", WIFI_AP_SSID);
    Serial.printf("Connect and browse to http://%s/\n", WiFi.softAPIP().toString().c_str());
}

// ============================================================
//  LED
// ============================================================

void updateLED() {
    uint32_t now = millis();
    uint32_t interval;
    switch (currentState) {
        case STATE_BOOT:     interval = 100;  break;
        case STATE_IDLE:     interval = 1000; break;
        case STATE_ARMED:    interval = 250;  break;
        case STATE_FLIGHT:   interval = 0;    break;
        case STATE_LANDED:   interval = 2000; break;
        case STATE_DOWNLOAD: interval = 500;  break;
        default:             interval = 1000; break;
    }
    if (interval == 0) { digitalWrite(LED_PIN, HIGH); return; }
    if (now - lastLedToggleMs >= interval) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        lastLedToggleMs = now;
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

    // Must have risen above the deploy altitude first
    if (altitude > PYRO_DEPLOY_ALT_M) {
        pyroArmed = true;
    }

    // Fire on descent through deploy altitude
    if (pyroArmed && altitude <= PYRO_DEPLOY_ALT_M && altitude < prevAltitude) {
        Serial.println("*** PYRO FIRED - deployment charge ***");
        firePyro();
        pyroFired = true;
    }

    prevAltitude = altitude;
}

// ============================================================
//  COMMAND HANDLERS (shared by Serial and WiFi)
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
    json += "\"ring_buffer\":" + String(ringBuf.count()) + ",";
    json += "\"ring_buffer_max\":" + String(RING_BUFFER_SIZE) + ",";
    json += "\"preflight_samples\":" + String(preflightSamples) + ",";
    json += "\"flight_samples\":" + String(flightSamplesWritten) + ",";
    json += "\"flash_used_kb\":" + String(FFat.usedBytes() / 1024) + ",";
    json += "\"flash_total_kb\":" + String(FFat.totalBytes() / 1024) + ",";
    json += "\"test_flight\":" + String(testFlightActive ? "true" : "false");
    json += "}";
    return json;
}

// Returns result message. Caller handles WiFi teardown if needed.
String cmdArm() {
    if (currentState == STATE_IDLE || currentState == STATE_DOWNLOAD) {
        currentState = STATE_ARMED;
        return "ARMED - launch detection active";
    }
    return "Cannot arm in state " + String(stateName(currentState));
}

String cmdFormat() {
    if (currentState == STATE_FLIGHT || currentState == STATE_ARMED) {
        return "Cannot format in state " + String(stateName(currentState));
    }
    if (preflightFile) preflightFile.close();
    if (flightFile) flightFile.close();
    FFat.remove(PREFLIGHT_FILE);
    FFat.remove(FLIGHT_FILE);
    FFat.remove(META_FILE);
    formatStorage();
    preflightSamples = 0;
    flightSamplesWritten = 0;
    ringDrainedCount = 0;
    maxAccelMag = 0;
    minPressure = 999999;
    maxAltitude = 0;
    openPreflightFile();
    currentState = REQUIRE_ARMING ? STATE_IDLE : STATE_ARMED;
    return "Formatted. State: " + String(stateName(currentState));
}

String cmdReset() {
    if (currentState == STATE_FLIGHT) {
        return "Cannot reset during flight";
    }
    if (flightFile) flightFile.close();
    if (preflightFile) preflightFile.close();
    currentState = STATE_IDLE;
    launchConfirmCount = 0;
    landingStableCount = 0;
    flightSampleCount = 0;
    maxAccelMag = 0;
    minPressure = 999999;
    maxAltitude = 0;
    pyroFired = false;
    pyroArmed = false;
    prevAltitude = 0;
    openPreflightFile();
    return "Reset to IDLE. Ready.";
}

String cmdTest(int durationSecs) {
    if (currentState == STATE_FLIGHT) {
        return "Already in flight";
    }
    if (currentState != STATE_IDLE && currentState != STATE_ARMED &&
        currentState != STATE_LANDED && currentState != STATE_DOWNLOAD) {
        return "Cannot test in state " + String(stateName(currentState));
    }

    testFlightActive = true;
    if (durationSecs > 0) {
        testFlightEndMs = millis() + (uint32_t)durationSecs * 1000;
    } else {
        testFlightEndMs = 0;  // manual stop only
    }

    // Use the real flight transition pipeline
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

    if (preflightFile) preflightFile.close();
    openFlightFile();

    ringDrainedCount = ringBuf.drain(writeFlightSample);
    if (flightFile) flightFile.flush();

    currentState = STATE_FLIGHT;

    if (durationSecs > 0) {
        return "TEST FLIGHT started (" + String(durationSecs) + "s). Send STOP to end early.";
    }
    return "TEST FLIGHT started (no limit). Send STOP to end.";
}

String cmdStop() {
    if (!testFlightActive || currentState != STATE_FLIGHT) {
        return "No test flight in progress";
    }
    testFlightActive = false;
    testFlightEndMs = 0;
    // Use the real landing transition
    transitionToLanded();
    return "Test flight stopped. " + String(ringDrainedCount + flightSamplesWritten) + " samples recorded.";
}

// ============================================================
//  SERIAL COMMANDS
// ============================================================

void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "STATUS") {
        Serial.printf("State: %s\n", stateName(currentState));
        Serial.printf("Sensors: BNO085=%s, DPS310=%s\n",
                      bnoAvailable ? "OK" : "MISSING",
                      dpsAvailable ? "OK" : "MISSING");
        Serial.printf("Uptime: %u ms\n", millis());
        Serial.printf("Accel magnitude: %.2f m/s2 (%.2fg)\n",
                      accelMagnitude(), accelMagnitude() / 9.81f);
        Serial.printf("Pressure: %.1f Pa\n", cur_pressure);
        Serial.printf("Temperature: %.1f C\n", cur_temp);
        Serial.printf("Altitude: %.2f m\n", calculateAltitude(cur_pressure));
        Serial.printf("Ring buffer: %d / %d samples\n", ringBuf.count(), RING_BUFFER_SIZE);
        Serial.printf("Preflight samples: %u\n", preflightSamples);
        Serial.printf("Flight samples written: %u\n", flightSamplesWritten);
        Serial.printf("Flash: %u / %u KB used\n",
                      FFat.usedBytes() / 1024, FFat.totalBytes() / 1024);

    } else if (cmd == "ARM") {
        if (currentState != STATE_IDLE && currentState != STATE_DOWNLOAD) {
            Serial.printf("Cannot arm in state %s\n", stateName(currentState));
        } else {
            Serial.println("Arming will enable launch detection and pyro channel.");
            Serial.println("Type CONFIRM to arm:");
            String c = "";
            while (c == "") {
                if (Serial.available()) {
                    c = Serial.readStringUntil('\n');
                    c.trim(); c.toUpperCase();
                }
                updateLED();
            }
            if (c == "CONFIRM") {
                String result = cmdArm();
                if (currentState == STATE_ARMED) stopWiFi();
                Serial.println(result);
            } else {
                Serial.println("Aborted.");
            }
        }

    } else if (cmd == "DOWNLOAD") {
        serialDownload();

    } else if (cmd == "WIFI") {
        if (currentState == STATE_IDLE || currentState == STATE_LANDED) {
            currentState = STATE_DOWNLOAD;
            startWiFiDownload();
        } else {
            Serial.printf("Cannot start WiFi in state %s\n", stateName(currentState));
        }

    } else if (cmd == "FORMAT") {
        Serial.println(cmdFormat());

    } else if (cmd == "RESET") {
        stopWiFi();
        Serial.println(cmdReset());

    } else if (cmd.startsWith("TEST")) {
        int secs = 0;
        if (cmd.length() > 4) {
            secs = cmd.substring(4).toInt();
        }
        stopWiFi();
        Serial.println(cmdTest(secs));

    } else if (cmd == "STOP") {
        Serial.println(cmdStop());

    } else if (cmd == "FIRE") {
        if (currentState == STATE_ARMED || currentState == STATE_FLIGHT) {
            Serial.println("DENIED: Cannot test fire in state " + String(stateName(currentState)));
        } else {
            Serial.println("!!! WARNING: This will fire the pyro channel on pin D5 !!!");
            Serial.println("Type CONFIRM to proceed:");
            String c1 = "";
            while (c1 == "") {
                if (Serial.available()) {
                    c1 = Serial.readStringUntil('\n');
                    c1.trim(); c1.toUpperCase();
                }
                updateLED();
            }
            if (c1 != "CONFIRM") {
                Serial.println("Aborted.");
            } else {
                Serial.println("!!! FINAL CONFIRMATION - Type CONFIRM again to FIRE !!!");
                String c2 = "";
                while (c2 == "") {
                    if (Serial.available()) {
                        c2 = Serial.readStringUntil('\n');
                        c2.trim(); c2.toUpperCase();
                    }
                    updateLED();
                }
                if (c2 != "CONFIRM") {
                    Serial.println("Aborted.");
                } else {
                    Serial.println("*** FIRING PYRO CHANNEL (recording 2s) ***");
                    stopWiFi();
                    cmdTest(2);
                    firePyro();
                    Serial.println("Pyro fired. Recording for 2 seconds...");
                }
            }
        }

    } else {
        Serial.println("Commands: STATUS, ARM, DOWNLOAD, WIFI, FORMAT, RESET, TEST [secs], STOP, FIRE");
    }
}

// ============================================================
//  STATE TRANSITIONS
// ============================================================

void transitionToFlight() {
    Serial.println("*** LAUNCH DETECTED ***");
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

    if (preflightFile) preflightFile.close();
    openFlightFile();

    Serial.printf("Draining ring buffer (%d samples)...\n", ringBuf.count());
    ringDrainedCount = ringBuf.drain(writeFlightSample);
    if (flightFile) flightFile.flush();
    Serial.printf("Ring buffer flushed: %u samples written\n", ringDrainedCount);

    currentState = STATE_FLIGHT;
}

void transitionToLanded() {
    Serial.println("*** LANDING DETECTED ***");
    flushBatch();
    if (flightFile) flightFile.close();
    writeMetadata();

    currentState = STATE_LANDED;
    Serial.printf("Flight complete. Duration: %u ms\n", millis() - flightStartMs);
    Serial.printf("Total samples: %u (ring: %u + live: %u)\n",
                  ringDrainedCount + flightSamplesWritten,
                  ringDrainedCount, flightSamplesWritten);
    Serial.printf("Max accel: %.1f m/s2 (%.1fg)\n", maxAccelMag, maxAccelMag / 9.81f);
    Serial.printf("Max altitude: %.2f m\n", maxAltitude);
    Serial.println("Type DOWNLOAD for serial CSV or WIFI for wireless download.");
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  ROCKET FLIGHT DATA LOGGER v4");
    Serial.println("  Arduino Nano ESP32");
    Serial.println("  BNO085 + DPS310");
    Serial.println("========================================");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Pyro channel — explicitly LOW at boot
    pinMode(PYRO_PIN, OUTPUT);
    digitalWrite(PYRO_PIN, LOW);

    // I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    // Storage
    if (!initStorage()) {
        Serial.println("FATAL: Storage init failed. Halting.");
        while (true) { delay(100); }
    }

    // *** ONE-TIME CLEANUP — uncomment these 3 lines, upload once,
    //     then comment them out again and re-upload ***
    // FFat.remove(PREFLIGHT_FILE);
    // FFat.remove(FLIGHT_FILE);
    // FFat.remove(META_FILE);

    // Sensors
    bnoAvailable = initBNO085();
    if (!bnoAvailable) {
        Serial.println("WARNING: BNO085 init failed.");
        waitForProceed("BNO085");
    }

    dpsAvailable = initDPS310();
    if (!dpsAvailable) {
        Serial.println("WARNING: DPS310 init failed.");
        waitForProceed("DPS310");
    }

    // Calibrate ground pressure — discard warmup then average
    if (dpsAvailable) {
        Serial.print("Calibrating ground pressure... ");
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
        Serial.printf("%.1f Pa\n", groundPressure);
    } else {
        Serial.println("Skipping ground pressure calibration (no DPS310).");
    }

    // Open preflight log
    openPreflightFile();

    // State
    if (REQUIRE_ARMING) {
        currentState = STATE_IDLE;
        Serial.println("State: IDLE - type ARM to enable launch detection.");
    } else {
        currentState = STATE_ARMED;
        Serial.println("State: ARMED - launch detection active.");
    }

    Serial.println("Type STATUS for info, DOWNLOAD for data.");
    Serial.println("========================================");

    lastSampleUs = micros();
    lastSlowLogMs = millis();
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
    uint32_t nowUs = micros();

    handleSerialCommands();

    if (webServer) webServer->handleClient();

    // Deferred WiFi shutdown (after ARM via HTTP, so response gets sent)
    if (wifiShutdownPending) {
        wifiShutdownPending = false;
        stopWiFi();
    }

    updateLED();
    checkPyroTimeout();

    if (nowUs - lastSampleUs >= SAMPLE_INTERVAL_US) {
        lastSampleUs = nowUs;

        readSensors();
        DataPoint dp = buildDataPoint();

        switch (currentState) {

            case STATE_IDLE:
                ringBuf.push(dp);
                if (millis() - lastSlowLogMs >= (1000 / SLOW_LOG_HZ)) {
                    lastSlowLogMs = millis();
                    writePreflightSample(dp);
                }
                break;

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
                if (!testFlightActive) {
                    checkPyroDeploy(dp.altitude_m);
                }
                if (testFlightActive) {
                    // Auto-stop test flight when duration expires
                    if (testFlightEndMs > 0 && millis() >= testFlightEndMs) {
                        Serial.println(cmdStop());
                    }
                } else if (checkLanding()) {
                    transitionToLanded();
                }
                break;

            case STATE_LANDED:
                break;

            case STATE_DOWNLOAD:
                break;

            default:
                break;
        }
    }
}
