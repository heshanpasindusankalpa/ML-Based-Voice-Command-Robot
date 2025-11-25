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

// Action pins (Motors)
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

// Eye animation states
enum EyeState { SLEEPING, NEUTRAL, LOOKING_LEFT, LOOKING_RIGHT, BLINKING };
static EyeState currentEyeState = SLEEPING;
static unsigned long lastEyeUpdate = 0;
static unsigned long lastCommandTime = 0;
static const unsigned long COMMAND_DISPLAY_TIME = 2000;
static const unsigned long EYE_UPDATE_INTERVAL = 100;
static const unsigned long AWAKE_DURATION = 5000; // 5s awake after wake word
static int currentPupilOffset = 0;
static int targetPupilOffset = 0;
static bool blinking = false;
static unsigned long blinkStartTime = 0;
static const unsigned long BLINK_DURATION = 150;

// ------------------- Sleep Animation Variables -------------------
static unsigned long sleepAnimTime = 0;
static int zOffsetY = 0;
static bool zGoingUp = true;
static int eyelidOffset = 0;
static bool eyelidClosing = true;

// Forward declarations
void TaskAudio(void* arg);
void TaskInfer(void* arg);
void updateEyes();
void handleCommand(const char* command, float confidence);

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

// ------------------- Draw Eyes -------------------
void drawEyes(int pupilOffset, bool blink) {
  display.clearDisplay();
  int leftEyeX = 32, rightEyeX = 96;
  int eyeY = 40;

  if (currentEyeState == SLEEPING) {
    // ------------------- Animated Sleeping -------------------
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(60, 10 + zOffsetY); display.print("Z");
    display.setCursor(72, 4 + zOffsetY);  display.print("Z");
    display.setCursor(84, -2 + zOffsetY); display.print("Z");

    // Draw closed eyes with gentle breathing
    int eyeWidth = 40, eyeHeight = 10;
    display.fillRoundRect(leftEyeX - eyeWidth/2, eyeY - eyeHeight/2 - eyelidOffset, eyeWidth, eyeHeight + eyelidOffset*2, 10, SSD1306_WHITE);
    display.fillRoundRect(rightEyeX - eyeWidth/2, eyeY - eyeHeight/2 - eyelidOffset, eyeWidth, eyeHeight + eyelidOffset*2, 10, SSD1306_WHITE);
  } else {
    // ------------------- Awake Eyes -------------------
    int eyeWidth = 40, eyeHeight = 25;
    display.fillRoundRect(leftEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);
    display.fillRoundRect(rightEyeX - eyeWidth/2, eyeY - eyeHeight/2, eyeWidth, eyeHeight, 10, SSD1306_WHITE);

    if (!blink) {
      int pupilSize = 12;
      int leftPupilX = constrain(leftEyeX + pupilOffset, leftEyeX - 8, leftEyeX + 8);
      int rightPupilX = constrain(rightEyeX + pupilOffset, rightEyeX - 8, rightEyeX + 8);
      display.fillCircle(leftPupilX, eyeY, pupilSize, SSD1306_BLACK);
      display.fillCircle(rightPupilX, eyeY, pupilSize, SSD1306_BLACK);
      display.fillCircle(leftPupilX - 3, eyeY - 3, 2, SSD1306_WHITE);
      display.fillCircle(rightPupilX - 3, eyeY - 3, 2, SSD1306_WHITE);
    } else {
      display.fillRoundRect(leftEyeX - eyeWidth/2, eyeY - 2, eyeWidth, 4, 2, SSD1306_WHITE);
      display.fillRoundRect(rightEyeX - eyeWidth/2, eyeY - 2, eyeWidth, 4, 2, SSD1306_WHITE);
    }
  }
  display.display();
}

// ------------------- Update Eyes -------------------
void updateEyes() {
  unsigned long currentTime = millis();

  if (currentEyeState != SLEEPING && currentTime - lastCommandTime > AWAKE_DURATION) {
    currentEyeState = SLEEPING;
    targetPupilOffset = 0;
    blinking = false;
  }

  // Awake blinking
  if (currentEyeState != SLEEPING) {
    if (!blinking && random(1000) < 5) { blinking = true; blinkStartTime = currentTime; }
    if (blinking && currentTime - blinkStartTime > BLINK_DURATION) blinking = false;
    if (currentPupilOffset != targetPupilOffset) currentPupilOffset += (currentPupilOffset < targetPupilOffset ? 1 : -1);
    else if (currentEyeState == NEUTRAL && random(100) < 3) targetPupilOffset = random(-3, 4);
  }

  // Sleeping animation
  if (currentEyeState == SLEEPING) {
    if (zGoingUp) zOffsetY--; else zOffsetY++;
    if (zOffsetY < -4) zGoingUp = false;
    else if (zOffsetY > 4) zGoingUp = true;

    if (eyelidClosing) eyelidOffset++; else eyelidOffset--;
    if (eyelidOffset > 4) eyelidClosing = false;
    else if (eyelidOffset < 0) eyelidClosing = true;
  }

  drawEyes(currentPupilOffset, blinking);
}

// ------------------- Command Handler -------------------
void handleCommand(const char* command, float confidence) {
  // Only "hey nova" can wake up
  if (strcmp(command, "nova") == 0 && confidence > 0.65f) {
    currentEyeState = NEUTRAL;
    lastCommandTime = millis();
   // Serial.println("Waking up!");
    return;
  }

  // Ignore all other commands if sleeping
  if (currentEyeState == SLEEPING) return;

  // Awake commands
  lastCommandTime = millis();
  if (strcmp(command, "left") == 0) {
    currentEyeState = LOOKING_LEFT;
    targetPupilOffset = -8;
  } else if (strcmp(command, "right") == 0) {
    currentEyeState = LOOKING_RIGHT;
    targetPupilOffset = 8;
  } else if (strcmp(command, "forward") == 0 || strcmp(command, "backward") == 0) {
    currentEyeState = NEUTRAL;
    targetPupilOffset = 0;
    blinking = true;
    blinkStartTime = millis();
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  setupI2S();

  pinMode(AI2, OUTPUT); pinMode(BI2, OUTPUT);
  pinMode(AI1, OUTPUT); pinMode(BI1, OUTPUT);
  digitalWrite(AI2, LOW); digitalWrite(BI2, LOW);
  digitalWrite(AI1, LOW); digitalWrite(BI1, LOW);

  qFrames = xQueueCreate(1, sizeof(frame_ptr_t));

  xTaskCreatePinnedToCore(TaskAudio, "TaskAudio", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskInfer, "TaskInfer", 7168, NULL, 2, NULL, 1);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed")); for(;;);
  }

  drawEyes(0, false);
  Serial.println("System ready.");
  lastEyeUpdate = millis();
}

// ------------------- Fill int16 -------------------
static void i2s_fill_int16(int16_t* dst, size_t need) {
  size_t filled = 0;
  int32_t raw32[I2S_CHUNK]; int16_t conv16[I2S_CHUNK];
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
  frame_ptr_t first = frameA; xQueueSend(qFrames, &first, portMAX_DELAY);
  bool useA = false;
  for (;;) {
    int16_t* dst = useA ? frameA : frameB;
    int16_t* src = useA ? frameB : frameA;
    memmove(dst, &src[STRIDE_SAMPLES], (WIN_SAMPLES - STRIDE_SAMPLES) * sizeof(int16_t));
    i2s_fill_int16(&dst[WIN_SAMPLES - STRIDE_SAMPLES], STRIDE_SAMPLES);
    frame_ptr_t p = dst; xQueueSend(qFrames, &p, 0);
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
        float bestp = 0.f;
        const char* best = "";
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
          if (result.classification[i].value > bestp) { bestp = result.classification[i].value; best = result.classification[i].label; }
        }
        Serial.printf("Prediction: %s (%.2f)\n", best, bestp);
        handleCommand(best, bestp);

        // Motor control only if awake
        digitalWrite(AI1, LOW); digitalWrite(BI1, LOW); digitalWrite(AI2, LOW); digitalWrite(BI2, LOW);
        if (currentEyeState != SLEEPING && bestp > 0.65f) {
          if (strcmp(best, "forward") == 0) { digitalWrite(AI1, HIGH); digitalWrite(BI1, HIGH); }
          else if (strcmp(best, "backward") == 0) { digitalWrite(AI2, HIGH); digitalWrite(BI2, HIGH); }
          else if (strcmp(best, "left") == 0) { digitalWrite(AI2, HIGH); }
          else if (strcmp(best, "right") == 0) { digitalWrite(BI2, HIGH); }
        }
      }
    }
    if (millis() - lastEyeUpdate >= EYE_UPDATE_INTERVAL) { updateEyes(); lastEyeUpdate = millis(); }
    taskYIELD();
  }
}

// ------------------- Main Loop -------------------
void loop() {
  if (millis() - lastEyeUpdate >= EYE_UPDATE_INTERVAL) { updateEyes(); lastEyeUpdate = millis(); }
  delay();
}
