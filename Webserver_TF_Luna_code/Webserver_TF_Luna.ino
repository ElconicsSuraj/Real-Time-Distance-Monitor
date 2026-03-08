/*
 *  TF‑Luna → ESP32 on UART2 (GPIO16 / GPIO17) + Soft‑AP Web UI (192.168.4.1)
 *  ------------------------------------------------------------------------
 *  ‑ Sensor  TX  →  ESP32 GPIO16   (RX2)
 *  ‑ Sensor  RX  →  ESP32 GPIO17   (TX2)   ← optional, only for commands
 *  ‑ GNDs common, sensor VCC 5 V (or 3 V3 if your module allows)
 *
 *  After flashing:
 *    1. Connect to Wi‑Fi SSID "Elconics"  (password  Elconics@123)
 *    2. Browse to  http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

/* ----------  Wi‑Fi Soft‑AP credentials ---------- */
#define AP_SSID "Elconics"
#define AP_PASS "Elconics@123"        // ≥8 chars (leave "" for open AP)

/* ----------  LiDAR UART2 pins & constants ---------- */
const int RX_PIN = 16;            // ESP32 UART2 RX (sensor TX)
const int TX_PIN = 17;            // ESP32 UART2 TX (sensor RX) – set -1 if unused
const uint8_t HEADER = 0x59;
HardwareSerial lidar(2);          // use UART2 (Serial2)

/* ----------  Shared readings ---------- */
volatile uint16_t gDistance = 0;
volatile uint16_t gStrength = 0;
volatile float    gTempC    = 0.0;

/* ----------  Mutex for atomic access ---------- */
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/* ----------  Web server ---------- */
WebServer server(80);

/* ----------------  HTML page  ---------------- */
void handleRoot()
{
  static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<title>TF-Luna LiDAR</title>
<style>
 body{font-family:Arial,Helvetica,sans-serif;text-align:center;margin:40px;background:#fafafa}
 h1{margin-bottom:30px}
 .value{font-size:2.5rem;font-weight:bold;color:#0070f3}
</style></head><body>
<h1>TF‑Luna LiDAR Readings</h1>
<p>Distance: <span id="dist" class="value">--</span> cm</p>
<p>Strength: <span id="str"  class="value">--</span></p>
<p>Temperature: <span id="temp" class="value">--</span> °C</p>
<script>
async function poll(){
  try{
    const r = await fetch('/data');
    const j = await r.json();
    dist.textContent = j.distance;
    str.textContent  = j.strength;
    temp.textContent = j.temperature.toFixed(1);
  }catch(e){console.error(e);}
  setTimeout(poll,1000);
}
poll();
</script></body></html>)rawliteral";

  server.send_P(200, "text/html", INDEX_HTML);
}

/* -------------  JSON endpoint  ------------- */
void handleData()
{
  uint16_t d, s; float t;
  portENTER_CRITICAL(&timerMux);
    d = gDistance;  s = gStrength;  t = gTempC;
  portEXIT_CRITICAL(&timerMux);

  char json[80];
  snprintf(json, sizeof(json),
           R"({"distance":%u,"strength":%u,"temperature":%.1f})", d, s, t);
  server.send(200, "application/json", json);
}

/* -------------  Wi‑Fi Soft‑AP  ------------- */
void setupWeb()
{
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("Soft‑AP \"%s\" started. Browse to http://192.168.4.1\n", AP_SSID);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
}

/* ----------  LiDAR helpers ---------- */
bool checkChecksum(const uint8_t *p)
{
  uint8_t sum = 0;
  for (int i = 0; i < 8; ++i) sum += p[i];
  return sum == p[8];
}

void decodeFrame(const uint8_t *p)
{
  uint16_t dist   = p[2] | (p[3] << 8);
  uint16_t stren  = p[4] | (p[5] << 8);
  uint16_t tmpRaw = p[6] | (p[7] << 8);
  float    tempC  = tmpRaw / 8.0f - 256;

  portENTER_CRITICAL(&timerMux);
    gDistance = dist;
    gStrength = stren;
    gTempC    = tempC;
  portEXIT_CRITICAL(&timerMux);
}

/* ==========  SETUP  ========== */
void setup()
{
  Serial.begin(115200);
  lidar.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // UART2 115 200 baud
  setupWeb();
}

/* ==========  LOOP  ========== */
void loop()
{
  /* ---- Read LiDAR stream ---- */
  static uint8_t buf[9];
  static uint8_t idx = 0;

  while (lidar.available()) {
    uint8_t b = lidar.read();
    if (idx == 0 && b != HEADER) continue;
    if (idx == 1 && b != HEADER) { idx = 0; continue; }

    buf[idx++] = b;
    if (idx == 9) {
      idx = 0;
      if (checkChecksum(buf)) decodeFrame(buf);
    }
  }

  /* ---- Serve web clients ---- */
  server.handleClient();
}
