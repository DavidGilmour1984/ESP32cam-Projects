#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

/* ================= CAMERA MODEL ================= */

#define CAMERA_MODEL_M5STACK_ESP32CAM

#if defined(CAMERA_MODEL_M5STACK_ESP32CAM)

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM     5
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     22
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       39
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       33
#define Y6_GPIO_NUM       27
#define Y5_GPIO_NUM       12
#define Y4_GPIO_NUM       35
#define Y3_GPIO_NUM       14
#define Y2_GPIO_NUM        2
#define VSYNC_GPIO_NUM    18
#define HREF_GPIO_NUM     36
#define PCLK_GPIO_NUM     26

#endif

/* ================= ACCESS POINT ================= */

const char* ssid = "ESP32-CAM";
const char* password = "";

WebServer server(80);

/* ================= CAMERA INIT ================= */

void initCamera() {

  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("CAM_FAIL");
    while(true) delay(1000);
  }

  sensor_t * s = esp_camera_sensor_get();

  if(s->id.PID == OV5640_PID){
    Serial.println("OV5640 OK");
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 12);
  }

  Serial.println("CAM_OK");
}

/* ================= WEB PAGE ================= */

void handleRoot() {

  const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OV5640 Stream</title>

<style>
body{
font-family:Helvetica;
background:#f5f7fb;
text-align:center;
padding:20px;
}

.card{
background:white;
padding:20px;
border-radius:16px;
box-shadow:0 10px 25px rgba(0,0,0,0.15);
max-width:700px;
margin:auto;
}

img{
width:100%;
border-radius:12px;
}
</style>
</head>

<body>

<div class="card">
<h2>OV5640 Live Stream</h2>
<img src="/stream">
</div>

</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

/* ================= STREAM ================= */

void handleStream() {

  WiFiClient client = server.client();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) continue;

    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.println("Content-Length: " + String(fb->len));
    client.println();

    client.write(fb->buf, fb->len);
    client.println();

    esp_camera_fb_return(fb);

    delay(30);
  }
}

/* ================= SETUP ================= */

void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);

  initCamera();

  WiFi.softAP(ssid, password);

  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
}

/* ================= LOOP ================= */

void loop() {
  server.handleClient();
}
