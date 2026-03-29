/*
 * ═══════════════════════════════════════════════════════════════════════
 * ASCEND — ESP32-CAM Optical Flow Firmware
 * Team Bayes Frontier · IIT Dharwad · IRoC-U 2026
 * ═══════════════════════════════════════════════════════════════════════
 *
 * Hardware : AI Thinker ESP32-CAM (OV2640)
 * Frame    : 160×120 QQVGA grayscale
 * Algorithm: Block-matching SAD, 6×5 grid of 16×16 blocks, ±8 px search
 *
 * Two independent UART outputs:
 *   UART0 (GPIO1 TX / GPIO3 RX) @ 115200  → RPi ttyAMA2
 *       8-byte optical flow packets at ~25–30 Hz
 *
 *   UART2 (GPIO16 RX / GPIO17 TX) @ 921600 → RPi ttyAMA3
 *       Raw 160×120 grayscale frames at ~4.8 fps
 *       Sent on Core 0 via FreeRTOS task (non-blocking to flow loop)
 *
 * ─── IMPORTANT: AI Thinker Board Notes ───
 * GPIO16 is shared with PSRAM CS on AI Thinker ESP32-CAM.
 * GPIO17 is NOT broken out on the standard AI Thinker header.
 *
 * Options:
 *   A) Use a board variant where GPIO17 is accessible (recommended)
 *   B) Remap UART2 TX to GPIO12 or GPIO13 (if SD card not used):
 *        Change UART2_TX_PIN below to 12 or 13
 *   C) Solder a wire directly to the ESP32 module GPIO17 pad
 *
 * In Arduino IDE:
 *   Board   : "AI Thinker ESP32-CAM"
 *   PSRAM   : "Disabled"  (QQVGA fits in internal DRAM)
 *   CPU     : 240 MHz
 *   Partition: "Huge APP (3MB No OTA)"
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "esp_camera.h"

/* ── UART2 Pin Configuration ──────────────────────────────────────── */
/* Change these if GPIO16/17 are unavailable on your board variant    */
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17

/* ── AI Thinker ESP32-CAM Camera Pins ─────────────────────────────── */
#define PWDN_GPIO 32
#define RESET_GPIO -1
#define XCLK_GPIO 0
#define SIOD_GPIO 26
#define SIOC_GPIO 27
#define Y9_GPIO 35
#define Y8_GPIO 34
#define Y7_GPIO 39
#define Y6_GPIO 36
#define Y5_GPIO 21
#define Y4_GPIO 19
#define Y3_GPIO 18
#define Y2_GPIO 5
#define VSYNC_GPIO 25
#define HREF_GPIO 23
#define PCLK_GPIO 22
#define FLASH_LED 4 /* White flash LED on AI Thinker board */

/* ── Frame Parameters ─────────────────────────────────────────────── */
#define FRAME_W 160
#define FRAME_H 120
#define FRAME_PIXELS (FRAME_W * FRAME_H) /* 19200 */

/* ── Block-Matching Parameters ────────────────────────────────────── */
#define BLK_SIZE 16
#define SEARCH_RANGE 8
#define GRID_COLS 6
#define GRID_ROWS 5
#define NUM_BLOCKS (GRID_COLS * GRID_ROWS) /* 30 */
#define SAD_THRESH 4000 /* max SAD for a "valid" block match */

/* ── Protocol Constants ───────────────────────────────────────────── */
#define FLOW_HDR1 0xAB
#define FLOW_HDR2 0xCD
#define FRAME_SYNC1 0xAA
#define FRAME_SYNC2 0x55

/* ── Buffers ──────────────────────────────────────────────────────── */
static uint8_t prev_frame[FRAME_PIXELS];
static bool has_prev = false;

/* Shared buffer for UART2 task (Core 0) */
static uint8_t uart2_buf[FRAME_PIXELS];
static volatile bool frame_pending = false;
static SemaphoreHandle_t frame_mutex;

/* ── Block Grid Positions (precomputed, ≥8 px from edges) ─────────── */
static const uint8_t bx[GRID_COLS] = {8, 34, 59, 85, 110, 136};
static const uint8_t by[GRID_ROWS] = {8, 30, 52, 74, 96};

/* Per-block results */
static int8_t blk_dx[NUM_BLOCKS];
static int8_t blk_dy[NUM_BLOCKS];
static bool blk_ok[NUM_BLOCKS];

/* ═════════════════════════════════════════════════════════════════════
 * Camera Initialisation
 * ═════════════════════════════════════════════════════════════════════*/
bool init_camera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO;
  cfg.pin_d1 = Y3_GPIO;
  cfg.pin_d2 = Y4_GPIO;
  cfg.pin_d3 = Y5_GPIO;
  cfg.pin_d4 = Y6_GPIO;
  cfg.pin_d5 = Y7_GPIO;
  cfg.pin_d6 = Y8_GPIO;
  cfg.pin_d7 = Y9_GPIO;
  cfg.pin_xclk = XCLK_GPIO;
  cfg.pin_pclk = PCLK_GPIO;
  cfg.pin_vsync = VSYNC_GPIO;
  cfg.pin_href = HREF_GPIO;
  cfg.pin_sccb_sda = SIOD_GPIO;
  cfg.pin_sccb_scl = SIOC_GPIO;
  cfg.pin_pwdn = PWDN_GPIO;
  cfg.pin_reset = RESET_GPIO;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_GRAYSCALE;
  cfg.frame_size = FRAMESIZE_QQVGA; /* 160×120 */
  cfg.jpeg_quality = 10;
  cfg.fb_count = 2;
  cfg.fb_location = CAMERA_FB_IN_DRAM; /* no PSRAM needed */
  cfg.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&cfg) != ESP_OK)
    return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_gainceiling(s, (gainceiling_t)GAINCEILING_4X);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
  }
  return true;
}

/* ═════════════════════════════════════════════════════════════════════
 * Block-Matching SAD (with early termination)
 * ═════════════════════════════════════════════════════════════════════*/
static uint32_t sad_block(const uint8_t *cur, const uint8_t *prv, int ox,
                          int oy, int dx, int dy, uint32_t best) {
  uint32_t sad = 0;
  for (int r = 0; r < BLK_SIZE; r++) {
    int cy = oy + r;
    int py = cy + dy;
    if (py < 0 || py >= FRAME_H)
      return UINT32_MAX;
    int px0 = ox + dx;
    if (px0 < 0 || px0 + BLK_SIZE > FRAME_W)
      return UINT32_MAX;

    const uint8_t *cr = &cur[cy * FRAME_W + ox];
    const uint8_t *pr = &prv[py * FRAME_W + px0];

    for (int c = 0; c < BLK_SIZE; c++) {
      int d = (int)cr[c] - (int)pr[c];
      sad += (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
    }
    if (sad >= best)
      return UINT32_MAX; /* early exit */
  }
  return sad;
}

static void block_match(const uint8_t *cur, const uint8_t *prv) {
  int idx = 0;
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      int ox = bx[col], oy = by[row];
      uint32_t best_sad = UINT32_MAX;
      int best_dx = 0, best_dy = 0;

      /* Spiral-ish search: test (0,0) first for early termination */
      best_sad = sad_block(cur, prv, ox, oy, 0, 0, best_sad);
      /* Full ±8 search */
      for (int sy = -SEARCH_RANGE; sy <= SEARCH_RANGE; sy++) {
        for (int sx = -SEARCH_RANGE; sx <= SEARCH_RANGE; sx++) {
          if (sx == 0 && sy == 0)
            continue; /* already tested */
          uint32_t s = sad_block(cur, prv, ox, oy, sx, sy, best_sad);
          if (s < best_sad) {
            best_sad = s;
            best_dx = sx;
            best_dy = sy;
          }
        }
      }

      blk_ok[idx] = (best_sad < SAD_THRESH);
      blk_dx[idx] = (int8_t)best_dx;
      blk_dy[idx] = (int8_t)best_dy;
      idx++;
    }
  }
}

/* ═════════════════════════════════════════════════════════════════════
 * Median (insertion sort, N ≤ 30)
 * ═════════════════════════════════════════════════════════════════════*/
static int16_t median_i8(int8_t *a, int n) {
  for (int i = 1; i < n; i++) {
    int8_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
  return (n & 1) ? (int16_t)a[n / 2]
                 : (int16_t)((int16_t)a[n / 2 - 1] + a[n / 2]) / 2;
}

/* ═════════════════════════════════════════════════════════════════════
 * Send 8-byte Flow Packet on UART0
 *   [0xAB][0xCD][fx_lo][fx_hi][fy_lo][fy_hi][quality][xor_chk]
 * ═════════════════════════════════════════════════════════════════════*/
static void send_flow(int16_t fx100, int16_t fy100, uint8_t quality) {
  uint8_t pkt[8];
  pkt[0] = FLOW_HDR1;
  pkt[1] = FLOW_HDR2;
  pkt[2] = (uint8_t)(fx100 & 0xFF);
  pkt[3] = (uint8_t)((fx100 >> 8) & 0xFF);
  pkt[4] = (uint8_t)(fy100 & 0xFF);
  pkt[5] = (uint8_t)((fy100 >> 8) & 0xFF);
  pkt[6] = quality;
  pkt[7] = pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5] ^ pkt[6];
  Serial.write(pkt, 8);
}

/* ═════════════════════════════════════════════════════════════════════
 * Send Raw Frame on UART2
 *   [0xAA][0x55][19200 bytes pixels][xor_checksum]
 * ═════════════════════════════════════════════════════════════════════*/
static void send_frame_uart2(const uint8_t *px, size_t len) {
  Serial2.write(FRAME_SYNC1);
  Serial2.write(FRAME_SYNC2);
  Serial2.write(px, len);
  uint8_t chk = 0;
  for (size_t i = 0; i < len; i++)
    chk ^= px[i];
  Serial2.write(chk);
}

/* ═════════════════════════════════════════════════════════════════════
 * Core 0 Task — raw frame UART2 sender (~4.8 fps)
 * Runs independently so Core 1 flow loop stays at ~30 Hz.
 * ═════════════════════════════════════════════════════════════════════*/
static void frame_task(void *param) {
  (void)param;
  for (;;) {
    if (frame_pending) {
      if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        send_frame_uart2(uart2_buf, FRAME_PIXELS);
        frame_pending = false;
        xSemaphoreGive(frame_mutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ═════════════════════════════════════════════════════════════════════
 * Arduino Setup
 * ═════════════════════════════════════════════════════════════════════*/
void setup() {
  /* UART0 — flow packets */
  Serial.begin(115200);

  /* UART2 — raw frames (see pin defines at top) */
  Serial2.begin(921600, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  delay(500);

  /* Camera */
  if (!init_camera()) {
    pinMode(33, OUTPUT);
    for (;;) {
      digitalWrite(33, LOW);
      delay(200);
      digitalWrite(33, HIGH);
      delay(200);
    }
  }

  /* Flash LED ON — visual indicator that system is active */
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, HIGH);

  /* Warm-up: discard first 5 frames */
  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb)
      esp_camera_fb_return(fb);
    delay(50);
  }

  /* Frame sender task on Core 0 */
  frame_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(frame_task, "FrmTX", 4096, NULL, 1, NULL, 0);

  has_prev = false;
}

/* ═════════════════════════════════════════════════════════════════════
 * Arduino Loop (Core 1) — capture + block-match + flow packet
 * ═════════════════════════════════════════════════════════════════════*/
void loop() {
  unsigned long t0 = millis();

  /* ── Capture ──────────────────────────────────────────────────── */
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(10);
    return;
  }
  if (fb->len != FRAME_PIXELS) {
    esp_camera_fb_return(fb);
    delay(10);
    return;
  }

  const uint8_t *cur = fb->buf;

  /* ── Block-matching flow ──────────────────────────────────────── */
  if (has_prev) {
    block_match(cur, prev_frame);

    int8_t vdx[NUM_BLOCKS], vdy[NUM_BLOCKS];
    uint8_t nv = 0;
    for (int i = 0; i < NUM_BLOCKS; i++) {
      if (blk_ok[i]) {
        vdx[nv] = blk_dx[i];
        vdy[nv] = blk_dy[i];
        nv++;
      }
    }

    int16_t fx100 = 0, fy100 = 0;
    if (nv >= 2) {
      fx100 = median_i8(vdx, nv) * 100;
      fy100 = median_i8(vdy, nv) * 100;
    }
    send_flow(fx100, fy100, nv);
  }

  /* ── Copy frame for UART2 task (non-blocking) ─────────────────── */
  if (!frame_pending && xSemaphoreTake(frame_mutex, 0) == pdTRUE) {
    memcpy(uart2_buf, cur, FRAME_PIXELS);
    frame_pending = true;
    xSemaphoreGive(frame_mutex);
  }

  /* ── Store previous ───────────────────────────────────────────── */
  memcpy(prev_frame, cur, FRAME_PIXELS);
  has_prev = true;
  esp_camera_fb_return(fb);

  /* ── Rate limit ~30 Hz ────────────────────────────────────────── */
  unsigned long elapsed = millis() - t0;
  if (elapsed < 33)
    delay(33 - elapsed);
}
