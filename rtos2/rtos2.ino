#include <voiceCommandRobot_inferencing.h>
#include <driver/i2s.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <Arduino.h>  // for pinMode/digitalWrite/Serial

#define I2S_WS   25
#define I2S_SD   32
#define I2S_SCK  33
#define SR       16000

// Action pins
#define PIN_RIGHT  2   // goes HIGH when label == "right"
#define PIN_LEFT   4   // goes HIGH when label == "left"

// Window & stride (tweak stride for overlap)
static const size_t WIN_SAMPLES    = EI_CLASSIFIER_RAW_SAMPLE_COUNT; // e.g., 16000
static const size_t STRIDE_SAMPLES = WIN_SAMPLES / 5;                // 200 ms if win=1s

// Chunk size to read from I2S per loop (DMA friendly, small stack)
static const size_t I2S_CHUNK = 512;

// Two ping-pong frames used for inference windows (no ring buffer needed)
static int16_t frameA[WIN_SAMPLES];
static int16_t frameB[WIN_SAMPLES];

typedef int16_t* frame_ptr_t;
static QueueHandle_t qFrames;

// ---- Forward decl
void TaskAudio(void* arg);
void TaskInfer(void* arg);

// Simple DC blocker (cheap, effective)
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

// I2S configuration
void setupI2S() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SR,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,      // change to ONLY_RIGHT if LR pin selects right
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = true,     // tighter clock; set false if you prefer
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

void setup() {
  Serial.begin(115200);
  setupI2S();

  // Action pins
  pinMode(PIN_RIGHT, OUTPUT);
  pinMode(PIN_LEFT, OUTPUT);
  digitalWrite(PIN_RIGHT, LOW);
  digitalWrite(PIN_LEFT, LOW);

  // Queue holds pointers to ready frames
  qFrames = xQueueCreate(3, sizeof(frame_ptr_t));

  // Create tasks: Audio high priority on core 0, Infer lower on core 1
  xTaskCreatePinnedToCore(TaskAudio, "TaskAudio", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskInfer, "TaskInfer", 7168, NULL, 2, NULL, 1); // 7 KB usually enough

  Serial.println("System ready.");
}

// Helper: fill exactly 'need' samples (int16) by reading 32-bit I2S chunks and converting on the fly
static void i2s_fill_int16(int16_t* dst, size_t need) {
  size_t filled = 0;
  int32_t raw32[I2S_CHUNK];
  int16_t conv16[I2S_CHUNK];

  while (filled < need) {
    size_t br = 0;
    // Read a 32-bit chunk from I2S
    i2s_read(I2S_NUM_0, (void*)raw32, sizeof(raw32), &br, portMAX_DELAY);
    size_t n = br / sizeof(int32_t);

    // Convert 32->16 with scaling (tune shift 10–13 by amplitude if needed)
    for (size_t i = 0; i < n; i++) {
      int32_t s = raw32[i] >> 11;
      if (s > 32767) s = 32767; else if (s < -32768) s = -32768;
      conv16[i] = (int16_t)s;
    }

    // DC block the chunk (optional but helpful)
    dc_block(conv16, n);

    // Copy as much as we still need
    size_t room = need - filled;
    size_t take = (n < room) ? n : room;
    memcpy(&dst[filled], conv16, take * sizeof(int16_t));
    filled += take;
  }
}

// Audio Task: produce overlapping inference windows using sliding-window without a ring buffer
void TaskAudio(void* arg) {
  // 1) Prime: fill the first full window into frameA and send it
  i2s_fill_int16(frameA, WIN_SAMPLES);
  frame_ptr_t first = frameA;
  xQueueSend(qFrames, &first, portMAX_DELAY);

  // 2) Then alternate between A and B:
  bool useA = false;

  for (;;) {
    int16_t* dst = useA ? frameA : frameB;
    int16_t* src = useA ? frameB : frameA;

    // Slide: keep the last (WIN - STRIDE) samples from the previous frame
    memmove(dst, &src[STRIDE_SAMPLES], (WIN_SAMPLES - STRIDE_SAMPLES) * sizeof(int16_t));

    // Fill only the tail with new STRIDE samples
    i2s_fill_int16(&dst[WIN_SAMPLES - STRIDE_SAMPLES], STRIDE_SAMPLES);

    // Send for inference
    frame_ptr_t p = dst;
    xQueueSend(qFrames, &p, 0);  // non-blocking; switch to portMAX_DELAY if you prefer no drops

    useA = !useA;
    taskYIELD(); // let scheduler run other tasks
  }
}

// EI get_data wrapper for a provided frame pointer
static int get_audio_signal_data_cb(size_t offset, size_t length, float *out_ptr, const int16_t* data) {
  for (size_t i = 0; i < length; i++) out_ptr[i] = (float)data[offset + i] / 32768.0f;
  return 0;
}

// Store current frame pointer for the lambda (compact demo approach)
static frame_ptr_t __current_frame = nullptr;

// Inference Task: consume frames and run Edge Impulse
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

      extern frame_ptr_t __current_frame;
      __current_frame = frame;

      ei_impulse_result_t result;
      if (run_classifier(&signal, &result) == EI_IMPULSE_OK) {
        // Pick best label
        const char* best = "";
        float bestp = 0.f, secondp = 0.f;
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
          float v = result.classification[i].value;
          if (v > bestp) { secondp = bestp; bestp = v; best = result.classification[i].label; }
          else if (v > secondp) { secondp = v; }
        }

        // Threshold (same as before; change if you like)
        const float CONF = 0.70f;

        bool is_right = (strcmp(best, "right") == 0);
        bool is_left  = (strcmp(best, "left")  == 0);

        if (bestp > CONF && is_right) {
          digitalWrite(PIN_RIGHT, HIGH);
        } else {
          digitalWrite(PIN_RIGHT, LOW);
        }

        if (bestp > CONF && is_left) {
          digitalWrite(PIN_LEFT, HIGH);
        } else {
          digitalWrite(PIN_LEFT, LOW);
        }

        // Optional: print for debugging
        // Serial.printf("Pred: %s (%.2f)\n", best, bestp);
      }
    }
  }
}

void loop() {} // FreeRTOS tasks run forever
