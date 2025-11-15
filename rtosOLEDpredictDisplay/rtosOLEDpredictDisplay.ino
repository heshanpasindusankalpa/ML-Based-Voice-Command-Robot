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
#define BI2  4
#define AI1  13
#define BI1  12

// Window & stride
static const size_t WIN_SAMPLES    = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
static const size_t STRIDE_SAMPLES = WIN_SAMPLES / 5;
static const size_t I2S_CHUNK = 512;

// Ping-pong frames
static int16_t frameA[WIN_SAMPLES];
static int16_t frameB[WIN_SAMPLES];
typedef int16_t* frame_ptr_t;
static QueueHandle_t qFrames;

// OLED display
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

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  setupI2S();

  pinMode(AI2, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(BI1, OUTPUT);

  digitalWrite(AI2, LOW);
  digitalWrite(BI2, LOW);
  digitalWrite(AI1, LOW);
  digitalWrite(BI1, LOW);

  qFrames = xQueueCreate(1, sizeof(frame_ptr_t));

  xTaskCreatePinnedToCore(TaskAudio, "TaskAudio", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskInfer, "TaskInfer", 7168, NULL, 2, NULL, 1);

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 10);
  // display.println("Voice Command Robot");
  // display.setCursor(0, 25);
  // display.println("System Ready...");
  // display.display();

  // Serial.println("System ready.");
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

        // Show prediction on OLED
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 10);
        //display.print("Cmd:");
        display.println(best);
        display.setTextSize(1);
        display.setCursor(0, 55);
        display.print("Conf:  ");
        display.print(bestp * 100, 1);
       // display.println("%");
        display.display();

        Serial.printf("Prediction: %s (%.2f)\n", best, bestp);

        // --- Motor Control ---
        const float CONF = 0.50f;
        bool is_right = (strcmp(best, "right") == 0);
        bool is_left  = (strcmp(best, "left") == 0);
        bool is_forward = (strcmp(best, "forward") == 0);
        bool is_backward  = (strcmp(best, "backward") == 0);

        digitalWrite(AI1, LOW);
        digitalWrite(BI1, LOW);
        digitalWrite(AI2, LOW);
        digitalWrite(BI2, LOW);

        if (bestp > CONF) {
          if (is_forward) {
            digitalWrite(AI2, LOW);
            digitalWrite(AI1, HIGH);
            digitalWrite(BI2, LOW);
            digitalWrite(BI1, HIGH);
          } else if (is_backward) {
            digitalWrite(AI2, HIGH);
            digitalWrite(AI1, LOW);
            digitalWrite(BI2, HIGH);
            digitalWrite(BI1, LOW);
            
          } else if (is_left) {
            digitalWrite(AI2, HIGH);
          } else if (is_right) {
            digitalWrite(BI2, HIGH);
          }
        }
      }
    }
  }
}


// ------------------- Loop -------------------
void loop() {}
