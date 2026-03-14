/*
 TF-Luna → ESP32 + OLED + Web UI
 --------------------------------
 Sensor TX → GPIO16
 Sensor RX → GPIO17

 OLED SDA → GPIO21
 OLED SCL → GPIO22
*/

#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <U8g2lib.h>

/* ---------- WiFi Soft AP ---------- */

#define AP_SSID "Elconics"
#define AP_PASS "Elconics@123"

/* ---------- LiDAR UART ---------- */

const int RX_PIN = 16;
const int TX_PIN = 17;

const uint8_t HEADER = 0x59;

HardwareSerial lidar(2);

/* ---------- OLED ---------- */

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

/* ---------- Shared readings ---------- */

volatile uint16_t gDistance = 0;
volatile uint16_t gStrength = 0;
volatile float gTempC = 0;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/* ---------- Web Server ---------- */

WebServer server(80);

/* ---------- HTML Page ---------- */

void handleRoot()
{
  static const char PROGMEM PAGE[] = R"rawliteral(

<!DOCTYPE html>
<html>
<head>
<title>TF Luna</title>
<style>
body{font-family:Arial;text-align:center;margin-top:50px}
.value{font-size:40px;color:#0070f3}
</style>
</head>

<body>

<h1>TF-Luna LiDAR</h1>

Distance
<div id="d" class="value">--</div>

Strength
<div id="s" class="value">--</div>

Temp
<div id="t" class="value">--</div>

<script>

async function update()
{
 const r = await fetch('/data');
 const j = await r.json();

 d.innerHTML = j.distance + " cm";
 s.innerHTML = j.strength;
 t.innerHTML = j.temperature;

 setTimeout(update,1000);
}

update();

</script>

</body>
</html>

)rawliteral";

  server.send_P(200,"text/html",PAGE);
}

/* ---------- JSON Data ---------- */

void handleData()
{
  uint16_t d,s;
  float t;

  portENTER_CRITICAL(&timerMux);
  d = gDistance;
  s = gStrength;
  t = gTempC;
  portEXIT_CRITICAL(&timerMux);

  char json[100];

  sprintf(json,
  "{\"distance\":%u,\"strength\":%u,\"temperature\":%.1f}",
  d,s,t);

  server.send(200,"application/json",json);
}

/* ---------- Setup Web ---------- */

void setupWeb()
{
  WiFi.softAP(AP_SSID,AP_PASS);

  Serial.println("WiFi Started");
  Serial.println("Connect to Elconics");
  Serial.println("Open 192.168.4.1");

  server.on("/",handleRoot);
  server.on("/data",handleData);

  server.begin();
}

/* ---------- Checksum ---------- */

bool checkChecksum(uint8_t *p)
{
  uint8_t sum=0;

  for(int i=0;i<8;i++)
  sum+=p[i];

  return sum==p[8];
}

/* ---------- Decode Frame ---------- */

void decodeFrame(uint8_t *p)
{
  uint16_t dist = p[2] | (p[3]<<8);
  uint16_t str  = p[4] | (p[5]<<8);
  uint16_t tmp  = p[6] | (p[7]<<8);

  float temp = tmp/8.0 - 256;

  portENTER_CRITICAL(&timerMux);

  gDistance = dist;
  gStrength = str;
  gTempC = temp;

  portEXIT_CRITICAL(&timerMux);
}

/* ---------- OLED Display ---------- */

void updateOLED()
{
  uint16_t d;

  portENTER_CRITICAL(&timerMux);
  d = gDistance;
  portEXIT_CRITICAL(&timerMux);

  float ft = d * 0.0328084;

  oled.clearBuffer();

  oled.setFont(u8g2_font_ncenB08_tr);

  oled.drawStr(0,12,"ELCONICS LIDAR");

  char cm[20];
  sprintf(cm,"Distance: %d cm",d);
  oled.drawStr(0,35,cm);

  char feet[20];
  sprintf(feet,"Feet: %.2f ft",ft);
  oled.drawStr(0,55,feet);

  oled.sendBuffer();
}

/* ---------- Setup ---------- */

void setup()
{
  Serial.begin(115200);

  lidar.begin(115200,SERIAL_8N1,RX_PIN,TX_PIN);

  Wire.begin(21,22);

  oled.begin();

  /* Boot Screen */

  oled.clearBuffer();
  oled.setFont(u8g2_font_logisoso24_tr);
  oled.drawStr(5,40,"Elconics");
  oled.sendBuffer();

  delay(2000);

  /* WiFi Instructions */

  oled.clearBuffer();

  oled.setFont(u8g2_font_6x12_tr);

  oled.drawStr(0,12,"Turn ON WiFi");
  oled.drawStr(0,26,"Connect: Elconics");
  oled.drawStr(0,40,"Pass: Elconics@123");
  oled.drawStr(0,60,"Open 192.168.4.1");

  oled.sendBuffer();

  delay(4000);

  setupWeb();
}

/* ---------- Loop ---------- */

void loop()
{
  static uint8_t buf[9];
  static uint8_t idx=0;

  while(lidar.available())
  {
    uint8_t b = lidar.read();

    if(idx==0 && b!=HEADER)
    continue;

    if(idx==1 && b!=HEADER)
    {
      idx=0;
      continue;
    }

    buf[idx++]=b;

    if(idx==9)
    {
      idx=0;

      if(checkChecksum(buf))
      decodeFrame(buf);
    }
  }

  server.handleClient();

  static unsigned long lastOLED=0;

  if(millis()-lastOLED>500)
  {
    updateOLED();
    lastOLED=millis();
  }
}