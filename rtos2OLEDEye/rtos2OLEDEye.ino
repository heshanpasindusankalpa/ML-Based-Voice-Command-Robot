#include <voiceCommandRobot_inferencing.h>
#include <driver/i2s.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------- I2S settings -------------------
#define I2S_WS   25
#define I2S_SD   32
#define I2S_SCK  33
#define SR       16000

// Action pins
#define AI2  2
#define BI2   4
#define AI1  13
#define BI1  12



// Window & stride
static const size_t WIN_SAMPLES    = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
static const size_t STRIDE_SAMPLES = WIN_SAMPLES / 5;

// I2S chunk size
static const size_t I2S_CHUNK = 512;

// Ping-pong frames
static int16_t frameA[WIN_SAMPLES];
static int16_t frameB[WIN_SAMPLES];

typedef int16_t* frame_ptr_t;
static QueueHandle_t qFrames;

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Forward declarations
void TaskAudio(void* arg);
void TaskInfer(void* arg);

// ------------------- DC Blocker -------------------
static inline void dc_block(int16_t* x, size_t n) {
  static float yprev = 0, xprev = 0;
  for (size_t i = 0; i < n; i++) {
    float xi = (float)x[i];
    float y  = xi - xprev + 0.995f * yprev;
    xprev = xi; yprev = y;
    if (y > 32767) y = 32767; else if (y < -32768) y = -32768;
    x[i] = (int16_t)y;
  }
}

// ------------------- I2S Setup -------------------
void setupI2S() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SR,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, SR, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
}

// ------------------- OLED Eye Functions -------------------
void drawNeutralEyes() {
  display.clearDisplay();
  
  // Emo robot style - large oval eyes with white highlights
  int eyeWidth = 40;
  int eyeHeight = 25;
  int leftEyeX = 32;
  int rightEyeX = 96;
  int eyeY = 30;
  
  // Draw large black oval eyes (inverse of typical emo eyes)
  display.fillRoundRect(leftEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);
  display.fillRoundRect(rightEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);
  
  // Draw pupils (black circles inside white eyes)
  int pupilSize = 12;
  display.fillCircle(leftEyeX, eyeY, pupilSize, SSD1306_BLACK);
  display.fillCircle(rightEyeX, eyeY, pupilSize, SSD1306_BLACK);
  
  // Add white highlights (sparkles) - characteristic of emo/anime eyes
  display.fillCircle(leftEyeX - 4, eyeY - 4, 3, SSD1306_WHITE);
  display.fillCircle(rightEyeX - 4, eyeY - 4, 3, SSD1306_WHITE);
  
  // Optional: Add some eyelid-like curves for expression
  display.drawLine(leftEyeX - 15, eyeY - 13, leftEyeX + 15, eyeY - 13, SSD1306_BLACK);
  display.drawLine(rightEyeX - 15, eyeY - 13, rightEyeX + 15, eyeY - 13, SSD1306_BLACK);
  
  display.display();
}

void drawLeftEyes() {
  display.clearDisplay();
  
  // Eyes looking left - emo style
  int eyeWidth = 40;
  int eyeHeight = 25;
  int leftEyeX = 27;  // Shifted left
  int rightEyeX = 91; // Shifted left
  int eyeY = 30;
  
  // Large white oval eyes
  display.fillRoundRect(leftEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);
  display.fillRoundRect(rightEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);
  
  // Pupils looking left
  int pupilSize = 12;
  display.fillCircle(leftEyeX - 6, eyeY, pupilSize, SSD1306_BLACK);
  display.fillCircle(rightEyeX - 6, eyeY, pupilSize, SSD1306_BLACK);
  
  // White highlights
  display.fillCircle(leftEyeX - 10, eyeY - 4, 3, SSD1306_WHITE);
  display.fillCircle(rightEyeX - 10, eyeY - 4, 3, SSD1306_WHITE);
  
  // Eyelids with expression
  display.drawLine(leftEyeX - 15, eyeY - 13, leftEyeX + 15, eyeY - 13, SSD1306_BLACK);
  display.drawLine(rightEyeX - 15, eyeY - 13, rightEyeX + 15, eyeY - 13, SSD1306_BLACK);
  
  // Add surprised/curious expression with raised eyebrows
  display.drawLine(leftEyeX - 12, eyeY - 18, leftEyeX - 5, eyeY - 15, SSD1306_BLACK);
  display.drawLine(rightEyeX - 12, eyeY - 18, rightEyeX - 5, eyeY - 15, SSD1306_BLACK);
  
  display.display();
}

void drawRightEyes() {
  display.clearDisplay();
  
  // Eyes looking right - emo style
  int eyeWidth = 40;
  int eyeHeight = 25;
  int leftEyeX = 37;  // Shifted right
  int rightEyeX = 101; // Shifted right
  int eyeY = 30;
  
  // Large white oval eyes
  display.fillRoundRect(leftEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);
  display.fillRoundRect(rightEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);
  
  // Pupils looking right
  int pupilSize = 12;
  display.fillCircle(leftEyeX + 6, eyeY, pupilSize, SSD1306_BLACK);
  display.fillCircle(rightEyeX + 6, eyeY, pupilSize, SSD1306_BLACK);
  
  // White highlights
  display.fillCircle(leftEyeX + 10, eyeY - 4, 3, SSD1306_WHITE);
  display.fillCircle(rightEyeX + 10, eyeY - 4, 3, SSD1306_WHITE);
  
  // Eyelids with expression
  display.drawLine(leftEyeX - 15, eyeY - 13, leftEyeX + 15, eyeY - 13, SSD1306_BLACK);
  display.drawLine(rightEyeX - 15, eyeY - 13, rightEyeX + 15, eyeY - 13, SSD1306_BLACK);
  
  // Add surprised/curious expression with raised eyebrows
  display.drawLine(leftEyeX + 5, eyeY - 15, leftEyeX + 12, eyeY - 18, SSD1306_BLACK);
  display.drawLine(rightEyeX + 5, eyeY - 15, rightEyeX + 12, eyeY - 18, SSD1306_BLACK);
  
  display.display();
}

void drawBlinkEyes() {
  display.clearDisplay();
  
  // Blinking animation - horizontal lines for closed eyes
  int leftEyeX = 32;
  int rightEyeX = 96;
  int eyeY = 30;
  
  // Draw closed eyes (horizontal lines)
  display.drawLine(leftEyeX - 20, eyeY, leftEyeX + 20, eyeY, SSD1306_WHITE);
  display.drawLine(rightEyeX - 20, eyeY, rightEyeX + 20, eyeY, SSD1306_WHITE);
  
  // Add some curved ends for cute blink effect
  display.drawLine(leftEyeX - 20, eyeY - 1, leftEyeX - 20, eyeY + 1, SSD1306_WHITE);
  display.drawLine(leftEyeX + 20, eyeY - 1, leftEyeX + 20, eyeY + 1, SSD1306_WHITE);
  display.drawLine(rightEyeX - 20, eyeY - 1, rightEyeX - 20, eyeY + 1, SSD1306_WHITE);
  display.drawLine(rightEyeX + 20, eyeY - 1, rightEyeX + 20, eyeY + 1, SSD1306_WHITE);
  
  display.display();
}

void showEyeMovement(const char* direction) {
  if (strcmp(direction, "left") == 0) {
    // Quick blink before looking left
    drawBlinkEyes();
    delay(100);
    drawNeutralEyes();
    delay(50);
    drawLeftEyes();
    delay(800);
    drawNeutralEyes();
  } 
  else if (strcmp(direction, "right") == 0) {
    // Quick blink before looking right
    drawBlinkEyes();
    delay(100);
    drawNeutralEyes();
    delay(50);
    drawRightEyes();
    delay(800);
    drawNeutralEyes();
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  setupI2S();

  pinMode(AI2, OUTPUT);
  pinMode(BI2, OUTPUT);
  digitalWrite(AI2, LOW);
  digitalWrite(BI2, LOW);

  qFrames = xQueueCreate(3, sizeof(frame_ptr_t));

  xTaskCreatePinnedToCore(TaskAudio, "TaskAudio", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskInfer, "TaskInfer", 7168, NULL, 2, NULL, 1);

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  drawNeutralEyes(); // start with neutral emo eyes
  Serial.println("System ready.");
}

// ------------------- Fill int16 -------------------
static void i2s_fill_int16(int16_t* dst, size_t need) {
  size_t filled = 0;
  int32_t raw32[I2S_CHUNK];
  int16_t conv16[I2S_CHUNK];

  while (filled < need) {
    size_t br = 0;
    i2s_read(I2S_NUM_0, (void*)raw32, sizeof(raw32), &br, portMAX_DELAY);
    size_t n = br / sizeof(int32_t);

    for (size_t i = 0; i < n; i++) {
      int32_t s = raw32[i] >> 11;
      if (s > 32767) s = 32767; else if (s < -32768) s = -32768;
      conv16[i] = (int16_t)s;
    }

    dc_block(conv16, n);

    size_t room = need - filled;
    size_t take = (n < room) ? n : room;
    memcpy(&dst[filled], conv16, take * sizeof(int16_t));
    filled += take;
  }
}

// ------------------- Audio Task -------------------
void TaskAudio(void* arg) {
  i2s_fill_int16(frameA, WIN_SAMPLES);
  frame_ptr_t first = frameA;
  xQueueSend(qFrames, &first, portMAX_DELAY);

  bool useA = false;

  for (;;) {
    int16_t* dst = useA ? frameA : frameB;
    int16_t* src = useA ? frameB : frameA;

    memmove(dst, &src[STRIDE_SAMPLES], (WIN_SAMPLES - STRIDE_SAMPLES) * sizeof(int16_t));
    i2s_fill_int16(&dst[WIN_SAMPLES - STRIDE_SAMPLES], STRIDE_SAMPLES);

    frame_ptr_t p = dst;
    xQueueSend(qFrames, &p, 0);

    useA = !useA;
    taskYIELD();
  }
}

// ------------------- EI get_data wrapper -------------------
static int get_audio_signal_data_cb(size_t offset, size_t length, float *out_ptr, const int16_t* data) {
  for (size_t i = 0; i < length; i++) out_ptr[i] = (float)data[offset + i] / 32768.0f;
  return 0;
}

static frame_ptr_t __current_frame = nullptr;

// ------------------- Inference Task -------------------
void TaskInfer(void* arg) {
  for (;;) {
    frame_ptr_t frame = nullptr;
    if (xQueueReceive(qFrames, &frame, portMAX_DELAY) == pdTRUE && frame != nullptr) {
      ei::signal_t signal;
      signal.total_length = WIN_SAMPLES;
      signal.get_data = [](size_t off, size_t len, float* out) -> int {
        extern frame_ptr_t __current_frame;
        return get_audio_signal_data_cb(off, len, out, __current_frame);
      };

      __current_frame = frame;

      ei_impulse_result_t result;
      if (run_classifier(&signal, &result) == EI_IMPULSE_OK) {
        const char* best = "";
        float bestp = 0.f;
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
          float v = result.classification[i].value;
          if (v > bestp) { bestp = v; best = result.classification[i].label; }
        }

        const float CONF = 0.80f;
        bool is_right = (strcmp(best, "right") == 0);
        bool is_left  = (strcmp(best, "left") == 0);

        digitalWrite(AI2, (bestp > CONF && is_right) ? HIGH : LOW);
        digitalWrite(BI2,  (bestp > CONF && is_left)  ? HIGH : LOW);

        // --- Eye interaction ---
        
      }
    }
  }
}

// ------------------- Loop -------------------
void loop() {}