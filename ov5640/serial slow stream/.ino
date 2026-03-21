#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "base64.h"

/* ================= CAMERA PINS ================= */

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

/* ================= SETTINGS ================= */

#define BAUD 115200
#define PKT_SIZE 70
#define TIMEOUT_MS 2000

/* ================= STATE ================= */

String fullData = "";
int totalPackets = 0;
int currentPacket = 0;
bool sending = false;

/* ================= CHECKSUM ================= */

uint8_t checksum(const String &s){
  uint16_t sum = 0;
  for(int i=0;i<s.length();i++) sum += s[i];
  return sum & 0xFF;
}

/* ================= CAMERA INIT ================= */

void initCamera(){

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

  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 14;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("CAM_FAIL");
    while(true) delay(1000);
  }

  Serial.println("CAM_OK");
}

/* ================= SEND ONE PACKET ================= */

void sendPacket(){

  int start = currentPacket * PKT_SIZE;
  int end = start + PKT_SIZE;
  if(end > fullData.length()) end = fullData.length();

  String payload = fullData.substring(start, end);
  uint8_t chk = checksum(payload);

  Serial.print("PKT,");
  Serial.print(currentPacket);
  Serial.print(",");
  Serial.print(chk);
  Serial.print(",");
  Serial.println(payload);
}

/* ================= CAPTURE ================= */

void startCapture(){

  camera_fb_t *fb = esp_camera_fb_get();

  if(!fb){
    Serial.println("ERR");
    return;
  }

  fullData = base64::encode(fb->buf, fb->len);

  totalPackets = (fullData.length() + PKT_SIZE - 1) / PKT_SIZE;
  currentPacket = 0;
  sending = true;

  Serial.print("START,");
  Serial.println(totalPackets);

  esp_camera_fb_return(fb);

  sendPacket(); // send first packet immediately
}

/* ================= SETUP ================= */

void setup(){

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(BAUD);
  delay(2000);

  Serial.println("BOOT");

  initCamera();

  Serial.println("READY");
}

/* ================= LOOP ================= */

void loop(){

  static String cmd = "";
  static unsigned long lastSendTime = 0;

  while(Serial.available()){

    char c = Serial.read();

    if(c == '\n'){

      cmd.trim();

      /* ===== START CAPTURE ===== */
      if(cmd == "CAPTURE"){
        sending = false;   // force reset if stuck
        startCapture();
      }

      /* ===== ACK ===== */
      else if(cmd.startsWith("ACK")){

        int c1 = cmd.indexOf(',');
        int c2 = cmd.indexOf(',', c1+1);

        int idx = cmd.substring(c1+1, c2).toInt();
        int chk = cmd.substring(c2+1).toInt();

        int start = currentPacket * PKT_SIZE;
        int end = start + PKT_SIZE;
        if(end > fullData.length()) end = fullData.length();

        String payload = fullData.substring(start, end);
        uint8_t expectedChk = checksum(payload);

        if(idx == currentPacket && chk == expectedChk){

          currentPacket++;

          if(currentPacket >= totalPackets){
            Serial.println("END");
            sending = false;
          } else {
            sendPacket();
            lastSendTime = millis();
          }
        }
      }

      /* ===== NACK ===== */
      else if(cmd.startsWith("NACK")){
        sendPacket(); // resend same packet
        lastSendTime = millis();
      }

      cmd = "";
    }
    else{
      cmd += c;
    }
  }

  /* ===== TIMEOUT RESEND ===== */
  if(sending && millis() - lastSendTime > TIMEOUT_MS){
    sendPacket();
    lastSendTime = millis();
  }
}
