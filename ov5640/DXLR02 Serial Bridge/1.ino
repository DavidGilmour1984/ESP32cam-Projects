/*
  ================================================================
  ESP32 LoRa Camera — Blast-and-Repair Protocol
  ================================================================

  PROTOCOL OVERVIEW:
  ------------------
  Unlike stop-and-wait (one packet → wait for ACK → next), this
  firmware blasts ALL packets out first, then the host requests
  any that were missing. This is dramatically faster over LoRa
  where round-trip latency is high.

  COMMAND SET (host → ESP32):
  ---------------------------
  CONFIG,<framesize>,<quality>,<pktsize>
      Set camera and transfer parameters before capture.
      framesize: 0=QQVGA 1=QVGA 2=VGA 3=SVGA 4=UXGA
      quality:   4–63 (lower = better quality, larger file)
      pktsize:   16–240 (characters per packet, keep ≤100 for LoRa)

  CAPTURE
      Take a photo with current settings and begin blast.

  REPAIR,<idx>,<idx>,...
      Request retransmission of specific packet indices.
      Host sends this after receiving END to fill in gaps.

  STATUS
      Returns current config as STATUS,<framesize>,<quality>,<pktsize>

  PROTOCOL FLOW:
  ------------------
  Host sends: CAPTURE
  ESP32 sends: START,<totalPackets>,<totalB64bytes>
  ESP32 sends: PKT,<idx>,<chk>,<data>   (all packets, no waiting)
  ESP32 sends: END
  Host checks for gaps, sends: REPAIR,3,7,12
  ESP32 retransmits: PKT,3,... PKT,7,... PKT,12,...
  ESP32 sends: END
  (repeat REPAIR/END until host confirms complete)

  ================================================================
*/

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "base64.h"

/* ================= CAMERA PINS ================= */

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM    5
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    22
#define SIOC_GPIO_NUM    23
#define Y9_GPIO_NUM      39
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      33
#define Y6_GPIO_NUM      27
#define Y5_GPIO_NUM      12
#define Y4_GPIO_NUM      35
#define Y3_GPIO_NUM      14
#define Y2_GPIO_NUM       2
#define VSYNC_GPIO_NUM   18
#define HREF_GPIO_NUM    36
#define PCLK_GPIO_NUM    26

/* ================= DEFAULTS ================= */

/*
  LORA BAUD NOTE:
  The DX-LR02 is configured via AT commands to a fixed UART baud rate.
  Set BAUD here to match whatever your DX-LR02 is configured to.
  9600 is the factory default for most DX-LR02 modules and is
  recommended — the LoRa radio can't keep up with 115200 anyway.
  Use AT+BAUD=5 to set the module to 115200 if you prefer.
*/
#define BAUD              9600

/*
  LORA INTER-PACKET DELAY:
  The DX-LR02 buffers incoming UART bytes and fires a LoRa radio frame
  when it detects a UART idle gap (typically ~5ms at 9600 baud).
  If you send lines back-to-back with no gap, the module batches them
  into one oversized LoRa frame which gets dropped.
  This delay goes AFTER each Serial.println() to force one LoRa frame
  per packet line. At 9600 baud, 80 bytes takes ~83ms to clock out,
  so 100ms gives the module time to detect the idle and fire.
  Increase to 150ms if you still see drops.
*/
#define LORA_PKT_DELAY_MS  100

#define DEFAULT_PKT_SIZE  48   // Keep lines short: "PKT,NNN,NNN,<48 chars>\n" = ~60 bytes
#define DEFAULT_QUALITY   12   // Higher = smaller file = fewer packets = faster
#define DEFAULT_FRAMESIZE  FRAMESIZE_QQVGA  // 160x120 — start here, step up once working

/* ================= STATE ================= */

String   fullData     = "";
int      totalPackets = 0;
int      pktSize      = DEFAULT_PKT_SIZE;
int      jpegQuality  = DEFAULT_QUALITY;
framesize_t frameSize = DEFAULT_FRAMESIZE;

/* ================= HELPERS ================= */

uint8_t checksum(const String &s) {
  uint16_t sum = 0;
  for (int i = 0; i < (int)s.length(); i++) sum += (uint8_t)s[i];
  return sum & 0xFF;
}

String getPacket(int idx) {
  int start = idx * pktSize;
  int end   = start + pktSize;
  if (end > (int)fullData.length()) end = fullData.length();
  return fullData.substring(start, end);
}

void sendPacket(int idx) {
  String payload = getPacket(idx);
  uint8_t chk = checksum(payload);
  Serial.print("PKT,");
  Serial.print(idx);
  Serial.print(",");
  Serial.print(chk);
  Serial.print(",");
  Serial.println(payload);
  Serial.flush();                  // wait until all bytes are clocked out to the DX-LR02
  delay(LORA_PKT_DELAY_MS);        // idle gap triggers DX-LR02 to fire one LoRa frame
}

/* ================= FRAMESIZE MAPPING ================= */

framesize_t indexToFramesize(int idx) {
  switch (idx) {
    case 0: return FRAMESIZE_QQVGA;  // 160x120
    case 1: return FRAMESIZE_QVGA;   // 320x240
    case 2: return FRAMESIZE_VGA;    // 640x480
    case 3: return FRAMESIZE_SVGA;   // 800x600
    case 4: return FRAMESIZE_UXGA;   // 1600x1200
    default: return FRAMESIZE_QVGA;
  }
}

int framesizeToIndex(framesize_t fs) {
  switch (fs) {
    case FRAMESIZE_QQVGA: return 0;
    case FRAMESIZE_QVGA:  return 1;
    case FRAMESIZE_VGA:   return 2;
    case FRAMESIZE_SVGA:  return 3;
    case FRAMESIZE_UXGA:  return 4;
    default:              return 1;
  }
}

/* ================= CAMERA INIT ================= */

bool initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0    = Y2_GPIO_NUM;
  config.pin_d1    = Y3_GPIO_NUM;
  config.pin_d2    = Y4_GPIO_NUM;
  config.pin_d3    = Y5_GPIO_NUM;
  config.pin_d4    = Y6_GPIO_NUM;
  config.pin_d5    = Y7_GPIO_NUM;
  config.pin_d6    = Y8_GPIO_NUM;
  config.pin_d7    = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = frameSize;
  config.jpeg_quality = jpegQuality;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("CAM_FAIL");
    return false;
  }

  Serial.println("CAM_OK");
  return true;
}

/* Apply frame size / quality change to running sensor */
void applyCameraSettings() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("ERR,NO_SENSOR");
    return;
  }
  s->set_framesize(s, frameSize);
  s->set_quality(s, jpegQuality);
  Serial.println("SETTINGS_OK");
}

/* ================= CAPTURE ================= */

void doCapture() {
  camera_fb_t *fb;

  // Discard stale buffered frame
  fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  delay(80);

  // Real capture
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERR,CAPTURE_FAIL");
    return;
  }

  fullData     = base64::encode(fb->buf, fb->len);
  totalPackets = (fullData.length() + pktSize - 1) / pktSize;

  esp_camera_fb_return(fb);

  // Announce — flush so START is its own LoRa frame
  Serial.print("START,");
  Serial.print(totalPackets);
  Serial.print(",");
  Serial.println(fullData.length());
  Serial.flush();
  delay(LORA_PKT_DELAY_MS);

  // Blast all packets (each sendPacket has its own flush+delay)
  for (int i = 0; i < totalPackets; i++) {
    sendPacket(i);
  }

  // END gets its own LoRa frame
  Serial.println("END");
  Serial.flush();
}

/* ================= REPAIR ================= */

void doRepair(String &args) {
  // args = "3,7,12,..."
  int start = 0;
  while (true) {
    int comma = args.indexOf(',', start);
    String token = (comma == -1)
      ? args.substring(start)
      : args.substring(start, comma);
    token.trim();
    if (token.length() > 0) {
      int idx = token.toInt();
      if (idx >= 0 && idx < totalPackets) {
        sendPacket(idx);
      }
    }
    if (comma == -1) break;
    start = comma + 1;
  }
  Serial.println("END");
  Serial.flush();
}

/* ================= CONFIG ================= */

void doConfig(String &args) {
  // args = "<framesizeIdx>,<quality>,<pktsize>"
  int c1 = args.indexOf(',');
  int c2 = args.indexOf(',', c1 + 1);

  if (c1 == -1 || c2 == -1) {
    Serial.println("ERR,BAD_CONFIG");
    return;
  }

  int fsIdx = args.substring(0, c1).toInt();
  int qual  = args.substring(c1 + 1, c2).toInt();
  int psz   = args.substring(c2 + 1).toInt();

  // Clamp values
  fsIdx = constrain(fsIdx, 0, 4);
  qual  = constrain(qual, 4, 63);
  psz   = constrain(psz, 16, 240);

  frameSize   = indexToFramesize(fsIdx);
  jpegQuality = qual;
  pktSize     = psz;

  applyCameraSettings();
}

void doStatus() {
  Serial.print("STATUS,");
  Serial.print(framesizeToIndex(frameSize));
  Serial.print(",");
  Serial.print(jpegQuality);
  Serial.print(",");
  Serial.println(pktSize);
}

/* ================= SETUP ================= */

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(BAUD);
  delay(2000);
  Serial.println("BOOT");
  initCamera();
  Serial.println("READY");
  doStatus();  // Send initial config so UI can sync on connect
}

/* ================= LOOP ================= */

void loop() {
  static String cmd = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      cmd.trim();

      if (cmd.startsWith("CAPTURE")) {
        doCapture();

      } else if (cmd.startsWith("REPAIR,")) {
        String args = cmd.substring(7);
        doRepair(args);

      } else if (cmd.startsWith("CONFIG,")) {
        String args = cmd.substring(7);
        doConfig(args);

      } else if (cmd.startsWith("STATUS")) {
        doStatus();
      }

      cmd = "";
    } else {
      cmd += c;
    }
  }
}
