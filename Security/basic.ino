#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

// =======================
// CAMERA PINS (AI Thinker)
// =======================
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// =======================
// WIFI
// =======================
const char* ssid     = "G2.4";
const char* password = "";

// =======================
// STATIC IP
// =======================
IPAddress local_IP(192,168,1,30);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

// =======================
// SERVER
// =======================
WebServer server(80);

// =======================
// CAMERA INIT
// =======================
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    while(true) delay(1000);
  }
}

// =======================
// ROOT PAGE (NO REFRESH)
// =======================
void handleRoot() {
  const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Live</title>
<style>
body{font-family:Helvetica;background:#f5f7fb;text-align:center;padding:20px}
.card{background:#fff;padding:16px;border-radius:14px;
box-shadow:0 6px 18px rgba(0,0,0,.1);max-width:700px;margin:auto}
img{width:100%;border-radius:12px}
</style>
</head>
<body>
<div class="card">
<h2>ESP32-CAM Live Stream</h2>
<img id="cam">
</div>

<script>
const img = document.getElementById("cam");

function loadFrame(){
  img.src = "/stream?t=" + Date.now();
}

img.onload = () => {
  setTimeout(loadFrame, 150);
};

img.onerror = () => {
  setTimeout(loadFrame, 500);
};

loadFrame();
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// =======================
// SINGLE FRAME ENDPOINT
// =======================
void handleStream() {
  camera_fb_t *fb = esp_camera_fb_get();
  if(!fb){
    server.send(500,"text/plain","Camera error");
    return;
  }

  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: image/jpeg");
  client.println("Content-Length: " + String(fb->len));
  client.println("Cache-Control: no-cache");
  client.println("Connection: close");
  client.println();

  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);

  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
  }

  initCamera();

  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.begin();
}

// =======================
// LOOP
// =======================
void loop() {
  server.handleClient();
}
