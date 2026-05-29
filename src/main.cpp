
#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"

static constexpr int PIN_TRIGGER_10HZ = 4;
static constexpr int PIN_SYNC_1HZ = 5;
static constexpr int PIN_GPRMC_TX = 17;
static constexpr int PIN_GPRMC_RX = 18;

#if defined(LED_BUILTIN)
static constexpr int PIN_STATUS_LED = LED_BUILTIN;
#else
static constexpr int PIN_STATUS_LED = 2;
#endif

static constexpr uint32_t GPRMC_BAUD = 9600;
static constexpr uint32_t CONSOLE_BAUD = 115200;

HardwareSerial SerialGPRMC(1);

static volatile bool s_sync_high = false;
static volatile uint8_t s_hh = 0;
static volatile uint8_t s_mm = 0;
static volatile uint8_t s_ss = 0;
static esp_timer_handle_t s_sync_timer = nullptr;

static void increment_time_hms() {
  s_ss++;
  if (s_ss >= 60) {
    s_ss = 0;
    s_mm++;
    if (s_mm >= 60) {
      s_mm = 0;
      s_hh++;
      if (s_hh >= 24) {
        s_hh = 0;
      }
    }
  }
}

static void emit_gprmc() {
  char body[96];
  snprintf(body, sizeof(body),
           "GPRMC,%02u%02u%02u.00,A,"
           "2237.496474,N,11356.089515,E,"
           "0.0,225.5,230520,2.3,W,A",
           s_hh, s_mm, s_ss);

  uint8_t cs = 0;
  for (const char *p = body; *p; p++) {
    cs ^= static_cast<uint8_t>(*p);
  }

  char full[120];
  snprintf(full, sizeof(full), "$%s*%02X\r\n", body, cs);
  SerialGPRMC.print(full);
}

static void sync_timer_callback(void *arg) {
  (void)arg;

  s_sync_high = !s_sync_high;
  gpio_set_level(static_cast<gpio_num_t>(PIN_SYNC_1HZ), s_sync_high ? 1 : 0);

  if (s_sync_high) {
    ledc_timer_rst(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    increment_time_hms();
    emit_gprmc();
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));

    Serial.printf("[1Hz] %02u:%02u:%02u - GPRMC emitted, 10Hz phase reset\n",
                  s_hh, s_mm, s_ss);
  }
}

void setup() {
  Serial.begin(CONSOLE_BAUD);
  delay(100);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  SerialGPRMC.begin(GPRMC_BAUD, SERIAL_8N1, PIN_GPRMC_RX, PIN_GPRMC_TX);

  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << PIN_SYNC_1HZ);
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(static_cast<gpio_num_t>(PIN_SYNC_1HZ), 0);

  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = 10;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_ch = {};
  ledc_ch.gpio_num = PIN_TRIGGER_10HZ;
  ledc_ch.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_ch.channel = LEDC_CHANNEL_0;
  ledc_ch.timer_sel = LEDC_TIMER_0;
  ledc_ch.duty = 512;
  ledc_ch.hpoint = 0;
  ledc_ch.intr_type = LEDC_INTR_DISABLE;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch));

  const esp_timer_create_args_t sync_timer_args = {
      .callback = &sync_timer_callback,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "sync_500ms",
  };
  ESP_ERROR_CHECK(esp_timer_create(&sync_timer_args, &s_sync_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(s_sync_timer, 500000));

  Serial.println();
  Serial.println("=== ESP32-S3 Phase 1 Timing Firmware ===");
  Serial.println("Board: DFRobot FireBeetle2 ESP32-S3");
  Serial.printf("10Hz PWM trigger: GPIO %d (LEDC low-speed timer0, 50%% duty)\n", PIN_TRIGGER_10HZ);
  Serial.printf("1Hz sync pulse:   GPIO %d (esp_timer 500ms toggle, 50%% duty)\n", PIN_SYNC_1HZ);
  Serial.printf("GPRMC UART1:      TX GPIO %d, RX GPIO %d @ %lu baud\n", PIN_GPRMC_TX, PIN_GPRMC_RX,
                static_cast<unsigned long>(GPRMC_BAUD));
  Serial.println("Invariant: on each 1Hz rising edge -> LEDC timer reset + monotonic time increment + GPRMC emit");
}

void loop() {
  static uint32_t last_heartbeat_ms = 0;
  const uint32_t now = millis();

  if ((now - last_heartbeat_ms) >= 10000) {
    last_heartbeat_ms = now;
    Serial.printf("[HB] uptime=%lus time=%02u:%02u:%02u\n",
                  static_cast<unsigned long>(now / 1000), s_hh, s_mm, s_ss);
  }

  delay(20);
}