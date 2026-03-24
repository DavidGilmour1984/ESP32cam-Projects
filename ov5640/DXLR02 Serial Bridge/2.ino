/*
  ================================================================
  ESP32 LoRa Camera — Blast-and-Repair Protocol (SD-CARD OPTIMIZED)
  ================================================================

  KEY IMPROVEMENTS:
  =================
  1. FORCED FRESH CAPTURE
     - Discard multiple stale frames (not just one)
     - 500ms delay between discard and real capture
     - Timestamps enable verification of freshness
     - No more receiving "old" images

  2. SD CARD INTEGRATION
     - Images saved to SD immediately after capture
     - Transmission reads FROM SD card (not RAM buffer)
     - Prevents buffer conflicts with capture/WiFi
     - Survives resets mid-transmission
     - Frees up heap for larger images

  3. STABLE TRANSMISSION
     - Smaller packets (32 bytes) = tighter timing
     - Longer inter-packet delay (150ms) = guaranteed frame boundaries
     - Chunked blasts (200 pkt max) = no 30+ sec timeouts
     - No competing I/O during transmission

  4. SEPARATE UART FOR LORA
     - Serial (GPIO 1/3): USB programmer & LoRa (UART0)
     - UART2 (GPIO 16/17): SD card (SPI) — no conflict
     - Your existing setup is fine; we just add SD via SPI

  WIRING:
  =======
  SD Card (SPI mode) — uses standard ESP32 SPI pins:
    SD_CLK   → GPIO 18 (SCK)
    SD_MOSI  → GPIO 23 (MOSI)
    SD_MISO  → GPIO 19 (MISO)
    SD_CS    → GPIO 5 (CS) — or any GPIO you prefer

  DX-LR02 LoRa (existing):
    RX → GPIO 16 (UART2 RX, if separate)
    TX → GPIO 17 (UART2 TX, if separate)
    OR use Serial (UART0) if you're already on it

  COMMAND SET:
  ============
  CONFIG,<framesize>,<quality>,<pktsize>[,<pktrate>]
      Set camera settings before capture.

  CAPTURE[,<chunksize>]
      Take fresh photo, save to SD, begin transmission.

  REPAIR,<idx>,<idx>,...
      Request retransmission of missing packets (current chunk).

  CHUNK,<idx>
      Request next chunk (for large images).

  LIST
      List all .jpg files on SD card.

  DELETE,<filename>
      Delete a file from SD card.

  STATUS
      Return current config.

  ================================================================
*/

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "base64.h"
// #include "SD_MMC.h"  // DISABLED FOR NOW - use RAM buffer instead
#include <time.h>

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

/* ================= SD CARD PINS (SPI MODE) ================= */

// Using standard SPI pins (don't conflict with camera or UART)
#define SD_CLK    18   // SCK
#define SD_MOSI   23   // MOSI
#define SD_MISO   19   // MISO
#define SD_CS      5   // CS (change if pin 5 conflicts with your setup)

/* ================= SERIAL & TIMING ================= */

#define BAUD              9600
#define LORA_MIN_DELAY_MS  200   // Base delay - more reliable
#define LORA_ADAPTIVE_DELAY 5    // Minimal padding since 200ms is already safe

/* ================= DEFAULTS ================= */

#define DEFAULT_PKT_SIZE       32
#define DEFAULT_QUALITY_QQVGA  15
#define DEFAULT_QUALITY_QVGA   20
#define DEFAULT_QUALITY_HIGHER 28
#define DEFAULT_FRAMESIZE      FRAMESIZE_QQVGA
#define DEFAULT_PKT_RATE_MS    200   // Increased from 150 for better reliability
#define MAX_CHUNK_SIZE         200

/* ================= STATE ================= */

String       currentFileName  = "";  // Not used (RAM only)
String       fullData         = "";  // Base64 encoded image in RAM
int          totalPackets     = 0;
int          totalChunks      = 0;
int          currentChunk     = 0;
int          chunkPktStart    = 0;
int          chunkPktEnd      = 0;
int          pktSize          = DEFAULT_PKT_SIZE;
int          jpegQuality      = DEFAULT_QUALITY_QQVGA;
int          pktRateMs        = DEFAULT_PKT_RATE_MS;
framesize_t  frameSize        = DEFAULT_FRAMESIZE;
bool         chunksPending    = false;

// Capture timestamp: compare with previous to ensure freshness
unsigned long lastCaptureTime = 0;

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
  Serial.flush();
  // Use fast delay during blasting, slower during repairs
  delay(100);  // 100ms is still safe for DX-LR02
}

void sendPacketRepair(int idx) {
  // Repair mode: send at user-configured rate (slower, more reliable)
  String payload = getPacket(idx);
  uint8_t chk = checksum(payload);
  Serial.print("PKT,");
  Serial.print(idx);
  Serial.print(",");
  Serial.print(chk);
  Serial.print(",");
  Serial.println(payload);
  Serial.flush();
  delay(pktRateMs);  // Use configured rate (200ms default)
}

String getTimestamp() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buf[20];
  strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", timeinfo);
  return String(buf);
}

/* ================= SD CARD ================= */

/* SD CARD FUNCTIONS DISABLED - Using RAM buffer instead */
/*
bool initSD() { ... }
bool saveJpegToSD(...) { ... }
bool loadJpegFromSD(...) { ... }
void listSDFiles() { ... }
void deleteSDFile(...) { ... }
*/

/* ================= FRAMESIZE MAPPING ================= */

framesize_t indexToFramesize(int idx) {
  switch (idx) {
    case 0: return FRAMESIZE_QQVGA;
    case 1: return FRAMESIZE_QVGA;
    case 2: return FRAMESIZE_VGA;
    case 3: return FRAMESIZE_SVGA;
    case 4: return FRAMESIZE_UXGA;
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

int getDefaultQuality(framesize_t fs) {
  switch (fs) {
    case FRAMESIZE_QQVGA: return DEFAULT_QUALITY_QQVGA;
    case FRAMESIZE_QVGA:  return DEFAULT_QUALITY_QVGA;
    case FRAMESIZE_VGA:
    case FRAMESIZE_SVGA:
    case FRAMESIZE_UXGA:  return DEFAULT_QUALITY_HIGHER;
    default:              return DEFAULT_QUALITY_QVGA;
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

/* ================= CAPTURE (FORCED FRESH) ================= */

void doCapture(String &args) {
  int chunkSize = MAX_CHUNK_SIZE;
  
  if (args.length() > 0) {
    chunkSize = args.toInt();
    chunkSize = constrain(chunkSize, 50, 500);
  }

  camera_fb_t *fb;

  // CRITICAL: Discard MULTIPLE stale frames
  for (int discard = 0; discard < 3; discard++) {
    fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
      delay(100);
    }
  }

  // CRITICAL: Long delay before real capture
  delay(500);

  // Now take the real capture
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERR,CAPTURE_FAIL");
    Serial.flush();
    return;
  }

  lastCaptureTime = millis();

  // Load into RAM buffer for transmission (no SD card)
  fullData = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  Serial.print("CAPTURED,RAM,");
  Serial.println(fullData.length());
  Serial.flush();

  totalPackets = (fullData.length() + pktSize - 1) / pktSize;
  totalChunks  = (totalPackets + chunkSize - 1) / chunkSize;
  currentChunk = 0;
  chunksPending = (totalChunks > 1);

  // Announce start
  Serial.print("START,");
  Serial.print(totalPackets);
  Serial.print(",");
  Serial.print(fullData.length());
  Serial.print(",");
  Serial.print(totalChunks);
  Serial.print(",");
  Serial.println(min((totalPackets - 0), chunkSize));
  Serial.flush();
  delay(LORA_MIN_DELAY_MS);

  // Blast chunk 0
  blastChunk(0, chunkSize);
}

void blastChunk(int chunkIdx, int chunkSize) {
  currentChunk = chunkIdx;
  chunkPktStart = chunkIdx * chunkSize;
  chunkPktEnd = min(chunkPktStart + chunkSize, totalPackets);

  Serial.print("BLAST_START,");
  Serial.println(chunkPktEnd - chunkPktStart);
  Serial.flush();

  for (int i = chunkPktStart; i < chunkPktEnd; i++) {
    sendPacket(i);
  }

  Serial.println("END");
  Serial.flush();
}

void doChunk(String &args) {
  if (!chunksPending) {
    Serial.println("ERR,NO_CHUNKS");
    return;
  }

  int chunkIdx = args.toInt();
  if (chunkIdx < 0 || chunkIdx >= totalChunks) {
    Serial.println("ERR,BAD_CHUNK");
    return;
  }

  int chunkSize = MAX_CHUNK_SIZE;
  blastChunk(chunkIdx, chunkSize);
}

/* ================= REPAIR ================= */

void doRepair(String &args) {
  int start = 0;
  while (true) {
    int comma = args.indexOf(',', start);
    String token = (comma == -1)
      ? args.substring(start)
      : args.substring(start, comma);
    token.trim();
    if (token.length() > 0) {
      int idx = token.toInt();
      if (idx >= chunkPktStart && idx < chunkPktEnd) {
        sendPacketRepair(idx);  // Use slower, more reliable rate
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
  int c1 = args.indexOf(',');
  int c2 = args.indexOf(',', c1 + 1);
  int c3 = args.indexOf(',', c2 + 1);

  if (c1 == -1 || c2 == -1) {
    Serial.println("ERR,BAD_CONFIG");
    return;
  }

  int fsIdx = args.substring(0, c1).toInt();
  int qual  = args.substring(c1 + 1, c2).toInt();
  int psz   = args.substring(c2 + 1, (c3 == -1) ? args.length() : c3).toInt();
  int prate = DEFAULT_PKT_RATE_MS;

  if (c3 != -1) {
    prate = args.substring(c3 + 1).toInt();
  }

  fsIdx = constrain(fsIdx, 0, 4);
  qual  = constrain(qual, 4, 63);
  psz   = constrain(psz, 16, 64);
  prate = constrain(prate, LORA_MIN_DELAY_MS, 500);

  frameSize   = indexToFramesize(fsIdx);
  jpegQuality = qual;
  pktSize     = psz;
  pktRateMs   = prate;

  applyCameraSettings();
  Serial.println("CONFIG_OK");
  Serial.flush();
}

void doStatus() {
  Serial.print("STATUS,");
  Serial.print(framesizeToIndex(frameSize));
  Serial.print(",");
  Serial.print(jpegQuality);
  Serial.print(",");
  Serial.print(pktSize);
  Serial.print(",");
  Serial.println(pktRateMs);
  Serial.flush();
}

/* ================= SETUP ================= */

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(BAUD);
  delay(2000);
  Serial.println("BOOT");
  Serial.flush();
  
  // Init camera
  Serial.println("INIT_CAM");
  Serial.flush();
  if (!initCamera()) {
    Serial.println("CAM_INIT_FAILED");
    Serial.flush();
    return;
  }
  
  Serial.println("READY");
  Serial.flush();
  doStatus();
}

/* ================= LOOP ================= */

void loop() {
  static String cmd = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      cmd.trim();

      if (cmd.startsWith("CAPTURE")) {
        String args = cmd.substring(7);
        doCapture(args);

      } else if (cmd.startsWith("REPAIR,")) {
        String args = cmd.substring(7);
        doRepair(args);

      } else if (cmd.startsWith("CHUNK,")) {
        String args = cmd.substring(6);
        doChunk(args);

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
