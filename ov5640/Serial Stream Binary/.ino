#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

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
#define PKT_SIZE 64
#define TIMEOUT_MS 2000

/* ================= BINARY PACKET FORMAT =================
   Byte 0  = 0xAA
   Byte 1  = 0x55
   Byte 2  = packet index high byte
   Byte 3  = packet index low byte
   Byte 4  = payload length
   Byte 5.. = payload bytes
   Last    = checksum

   START line is still text:
   START,<totalPackets>,<imageBytes>

   END line is still text:
   END

   ACK/NACK from receiver remain text:
   ACK,<packetIndex>,<checksum>
   NACK,<packetIndex>
========================================================= */

/* ================= STATE ================= */

camera_fb_t *currentFb = NULL;
uint8_t *imageData = NULL;
size_t imageLen = 0;

int totalPackets = 0;
int currentPacket = 0;
bool sending = false;

/* ================= CHECKSUM ================= */

uint8_t packetChecksum(uint16_t idx, uint8_t len, const uint8_t *data) {
  uint16_t sum = 0;

  sum += (idx >> 8) & 0xFF;
  sum += idx & 0xFF;
  sum += len;

  for (int i = 0; i < len; i++) {
    sum += data[i];
  }

  return sum & 0xFF;
}

/* ================= CLEANUP ================= */

void releaseFrame() {
  if (currentFb != NULL) {
    esp_camera_fb_return(currentFb);
    currentFb = NULL;
  }

  imageData = NULL;
  imageLen = 0;
  totalPackets = 0;
  currentPacket = 0;
  sending = false;
}

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

  config.frame_size = FRAMESIZE_QVGA;  // 320x240     // 1600x1200
  config.jpeg_quality = 4;                // very high quality
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("CAM_FAIL");
    while (true) delay(1000);
  }

  Serial.println("CAM_OK");
}

/* ================= SEND ONE PACKET ================= */

void sendPacket() {

  if (!sending || imageData == NULL || imageLen == 0) return;
  if (currentPacket >= totalPackets) return;

  int start = currentPacket * PKT_SIZE;
  int len = PKT_SIZE;

  if ((start + len) > (int)imageLen) {
    len = imageLen - start;
  }

  uint8_t chk = packetChecksum((uint16_t)currentPacket, (uint8_t)len, imageData + start);

  uint8_t header[5];
  header[0] = 0xAA;
  header[1] = 0x55;
  header[2] = (currentPacket >> 8) & 0xFF;
  header[3] = currentPacket & 0xFF;
  header[4] = len & 0xFF;

  Serial.write(header, 5);
  Serial.write(imageData + start, len);
  Serial.write(chk);
}

/* ================= CAPTURE ================= */

void startCapture() {

  releaseFrame();

  /* ===== FLUSH OLD FRAMES ===== */
  for (int i = 0; i < 3; i++) {
    camera_fb_t *fbFlush = esp_camera_fb_get();
    if (fbFlush) {
      esp_camera_fb_return(fbFlush);
    }
    delay(30);
  }

  /* ===== REAL CAPTURE ===== */
  currentFb = esp_camera_fb_get();

  if (!currentFb) {
    Serial.println("ERR");
    releaseFrame();
    return;
  }

  imageData = currentFb->buf;
  imageLen = currentFb->len;

  totalPackets = (imageLen + PKT_SIZE - 1) / PKT_SIZE;
  currentPacket = 0;
  sending = true;

  Serial.print("START,");
  Serial.print(totalPackets);
  Serial.print(",");
  Serial.println(imageLen);

  sendPacket();
}

/* ================= SETUP ================= */

void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(BAUD);
  delay(2000);

  Serial.println("BOOT");

  initCamera();

  Serial.println("READY");
}

/* ================= LOOP ================= */

void loop() {

  static String cmd = "";
  static unsigned long lastSendTime = 0;

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {

      cmd.trim();

      /* ===== DEBUG PRINT ===== */
      Serial.print("RX: ");
      Serial.println(cmd);

      /* ===== START CAPTURE ===== */
      if (cmd.startsWith("CAPTURE")) {
        Serial.println("CMD_OK");
        startCapture();
        lastSendTime = millis();
      }

      /* ===== ACK ===== */
      else if (cmd.startsWith("ACK")) {

        int c1 = cmd.indexOf(',');
        int c2 = cmd.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > c1) {
          int idx = cmd.substring(c1 + 1, c2).toInt();
          int chk = cmd.substring(c2 + 1).toInt();

          if (sending && idx == currentPacket) {

            int start = currentPacket * PKT_SIZE;
            int len = PKT_SIZE;

            if ((start + len) > (int)imageLen) {
              len = imageLen - start;
            }

            uint8_t expectedChk = packetChecksum((uint16_t)currentPacket, (uint8_t)len, imageData + start);

            if (chk == expectedChk) {
              currentPacket++;

              if (currentPacket >= totalPackets) {
                Serial.println("END");
                releaseFrame();
              } else {
                sendPacket();
                lastSendTime = millis();
              }
            }
          }
        }
      }

      /* ===== NACK ===== */
      else if (cmd.startsWith("NACK")) {

        int c1 = cmd.indexOf(',');

        if (c1 > 0) {
          int idx = cmd.substring(c1 + 1).toInt();

          if (sending && idx == currentPacket) {
            sendPacket();
            lastSendTime = millis();
          }
        } else {
          if (sending) {
            sendPacket();
            lastSendTime = millis();
          }
        }
      }

      cmd = "";
    }
    else {
      cmd += c;
    }
  }

  /* ===== TIMEOUT RESEND ===== */
  if (sending && millis() - lastSendTime > TIMEOUT_MS) {
    sendPacket();
    lastSendTime = millis();
  }
}
