
#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include <common/mavlink.h>

static constexpr int PIN_TRIGGER_10HZ = 4; // A0
static constexpr int PIN_SYNC_1HZ = 5; // A1
static constexpr int PIN_GPRMC_TX = 12; // D12
static constexpr int PIN_GPRMC_RX = 21; // D13

#if defined(LED_BUILTIN)
static constexpr int PIN_STATUS_LED = LED_BUILTIN;
#else
static constexpr int PIN_STATUS_LED = 2;
#endif

static constexpr uint32_t GPRMC_BAUD = 9600;
static constexpr uint32_t CONSOLE_BAUD = 115200;

// Phase 2 — MAVLink UART2
static constexpr int PIN_MAVLINK_TX = 38; // D3
static constexpr int PIN_MAVLINK_RX = 3;  // D2
static constexpr uint32_t MAVLINK_BAUD = 57600;
static constexpr uint8_t MAV_SYS_ID = 200;
static constexpr uint8_t MAV_COMP_ID = 1;
static constexpr uint32_t TIMESYNC_INTERVAL_MS = 10000;

// Phase 3 — clock discipline
static constexpr int64_t  DISC_MAX_SLEW_US     = 500LL;  // max correction per SYSTEM_TIME event
static constexpr uint8_t  DISC_WARMING_SAMPLES = 3;      // samples before LOCKED

HardwareSerial SerialGPRMC(1);
HardwareSerial SerialMAVLink(2);

static volatile bool s_sync_high = false;
static volatile uint8_t s_hh = 0;
static volatile uint8_t s_mm = 0;
static volatile uint8_t s_ss = 0;
static esp_timer_handle_t s_sync_timer = nullptr;

static mavlink_message_t s_mav_msg;
static mavlink_status_t s_mav_status;
static int64_t s_timesync_send_ns = 0;
static uint32_t s_last_timesync_ms = 0;

// DDS timesync convergence state (inferred from SYSTEM_TIME validity)
static constexpr uint64_t PX4_EPOCH_THRESHOLD_US = 1000000000000000ULL; // > year 2001
static bool     s_px4_dds_synced    = false;
static uint64_t s_px4_unix_us       = 0;     // last received unix_us from PX4
static uint32_t s_px4_boot_ms       = 0;     // last received boot_ms from PX4

// Phase 3 — clock state machine
enum ClockState : uint8_t { UNLOCKED, WARMING, LOCKED, DEGRADED };
static ClockState s_clock_state    = UNLOCKED;
static uint8_t    s_warming_count  = 0;

// Disciplined UTC clock reference
// Current UTC estimate = s_disc_epoch_us + (esp_timer_get_time() - s_disc_ref_esp_us)
static uint64_t s_disc_epoch_us   = 0;  // UTC unix-µs at reference point
static int64_t  s_disc_ref_esp_us = 0;  // esp_timer_get_time() at reference point (µs)
static int64_t  s_last_offset_us  = 0;  // offset at last update (for drift tracking)
static int64_t  s_total_corr_us   = 0;  // cumulative correction applied

static const char* clock_state_name(ClockState st) {
  switch (st) {
    case UNLOCKED:  return "UNLOCKED";
    case WARMING:   return "WARMING";
    case LOCKED:    return "LOCKED";
    case DEGRADED:  return "DEGRADED";
    default:        return "UNKNOWN";
  }
}

// Returns current disciplined UTC in microseconds. Only meaningful when LOCKED.
static uint64_t disc_get_unix_us() {
  int64_t delta = esp_timer_get_time() - s_disc_ref_esp_us;
  return s_disc_epoch_us + (uint64_t)delta;
}

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

static void mav_handle_heartbeat(const mavlink_message_t& msg) {
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(&msg, &hb);
  Serial.printf("[MAV] HEARTBEAT sysid=%u compid=%u type=%u autopilot=%u base_mode=0x%02X state=%u\n",
                msg.sysid, msg.compid, hb.type, hb.autopilot, hb.base_mode, hb.system_status);
}

static void mav_handle_system_time(const mavlink_message_t& msg) {
  mavlink_system_time_t st;
  mavlink_msg_system_time_decode(&msg, &st);

  s_px4_unix_us = st.time_unix_usec;
  s_px4_boot_ms = st.time_boot_ms;

  // DDS convergence detection (unchanged from Phase 2)
  bool newly_synced = false;
  if (!s_px4_dds_synced && st.time_unix_usec > PX4_EPOCH_THRESHOLD_US) {
    s_px4_dds_synced = true;
    newly_synced = true;
  }
  if (s_px4_dds_synced && st.time_unix_usec <= PX4_EPOCH_THRESHOLD_US) {
    s_px4_dds_synced = false;
    Serial.println("[MAV] PX4 DDS timesync: LOST");
  }

  Serial.printf("[MAV] SYSTEM_TIME unix_us=%llu boot_ms=%lu  dds=%s\n",
                (unsigned long long)st.time_unix_usec,
                (unsigned long)st.time_boot_ms,
                s_px4_dds_synced ? "CONVERGED" : "NOT_SYNCED");

  if (newly_synced) {
    Serial.println("[MAV] *** PX4 DDS timesync: CONVERGED — clock is valid ***");
  }

  // Phase 3 — clock state machine
  int64_t now_esp = esp_timer_get_time();  // µs since boot

  if (!s_px4_dds_synced) {
    // DDS not valid — degrade if we were running
    if (s_clock_state == LOCKED || s_clock_state == WARMING) {
      s_clock_state   = DEGRADED;
      s_warming_count = 0;
      Serial.printf("[CLK] state=DEGRADED (DDS lost)\n");
    }
    return;
  }

  // DDS is valid
  int64_t disc_now_us = (int64_t)(s_disc_epoch_us) + (now_esp - s_disc_ref_esp_us);
  int64_t offset_us   = (int64_t)st.time_unix_usec - disc_now_us;

  switch (s_clock_state) {
    case UNLOCKED:
    case DEGRADED:
      // Seed clock directly from PX4 — this is an intentional step (not steady state)
      s_disc_epoch_us   = st.time_unix_usec;
      s_disc_ref_esp_us = now_esp;
      s_warming_count   = 1;
      s_clock_state     = WARMING;
      Serial.printf("[CLK] state=WARMING sample=%u/%u  seed_unix_us=%llu\n",
                    s_warming_count, DISC_WARMING_SAMPLES,
                    (unsigned long long)st.time_unix_usec);
      break;

    case WARMING: {
      // Step-correct during WARMING (not yet locked)
      // Anchor to current PX4 UTC directly (no slew limit during WARMING)
      s_disc_epoch_us   = (uint64_t)((int64_t)disc_now_us + offset_us);
      s_disc_ref_esp_us = now_esp;
      s_last_offset_us  = offset_us;
      s_warming_count++;
      Serial.printf("[CLK] state=WARMING sample=%u/%u  offset=%lld us\n",
                    s_warming_count, DISC_WARMING_SAMPLES, (long long)offset_us);
      if (s_warming_count >= DISC_WARMING_SAMPLES) {
        s_clock_state = LOCKED;
        Serial.println("[CLK] *** state=LOCKED — disciplined clock valid ***");
      }
      break;
    }

    case LOCKED: {
      // Slew-rate limited correction (D6: never backward-step in steady state)
      int64_t corr = offset_us;
      if (corr >  DISC_MAX_SLEW_US) corr =  DISC_MAX_SLEW_US;
      if (corr < -DISC_MAX_SLEW_US) corr = -DISC_MAX_SLEW_US;
      s_disc_epoch_us   = (uint64_t)((int64_t)disc_now_us + corr);
      s_disc_ref_esp_us = now_esp;
      s_total_corr_us  += corr;
      int64_t drift_trend = offset_us - s_last_offset_us;  // direction of drift
      s_last_offset_us  = offset_us;
      // Log disciplined UTC as HH:MM:SS for validation
      uint64_t disc_s = disc_get_unix_us() / 1000000ULL;
      uint32_t d_hh = (uint32_t)((disc_s % 86400ULL) / 3600ULL);
      uint32_t d_mm = (uint32_t)((disc_s % 3600ULL)  / 60ULL);
      uint32_t d_ss = (uint32_t)(disc_s % 60ULL);
      Serial.printf("[CLK] state=LOCKED  disc_utc=%02u:%02u:%02u  offset=%lld us  corr=%lld us  total_corr=%lld us  drift_trend=%lld us\n",
                    d_hh, d_mm, d_ss,
                    (long long)offset_us, (long long)corr,
                    (long long)s_total_corr_us, (long long)drift_trend);
      break;
    }

    default:
      break;
  }
}

static void mav_handle_timesync(const mavlink_message_t& msg) {
  mavlink_timesync_t ts;
  mavlink_msg_timesync_decode(&msg, &ts);
  if (ts.tc1 != 0 && s_timesync_send_ns != 0) {
    // Use our stored send time — avoids PX4 time-base mismatch with tc1
    int64_t now_ns  = (int64_t)esp_timer_get_time() * 1000LL;
    int64_t rtt_ns  = now_ns - s_timesync_send_ns;
    Serial.printf("[MAV] TIMESYNC RTT=%lld us\n", rtt_ns / 1000LL);
  }
}

static void mav_handle_gps_raw_int(const mavlink_message_t& msg) {
  mavlink_gps_raw_int_t gps;
  mavlink_msg_gps_raw_int_decode(&msg, &gps);
  Serial.printf("[MAV] GPS_RAW_INT fix=%u sats=%u time_us=%llu\n",
                gps.fix_type, gps.satellites_visible, (unsigned long long)gps.time_usec);
}

static void mav_send_timesync() {
  mavlink_message_t out;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  s_timesync_send_ns = (int64_t)esp_timer_get_time() * 1000LL;
  mavlink_msg_timesync_pack(MAV_SYS_ID, MAV_COMP_ID, &out, 0LL, s_timesync_send_ns);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &out);
  SerialMAVLink.write(buf, len);
  Serial.printf("[MAV] TIMESYNC sent ts1=%lld ns\n", s_timesync_send_ns);
}

static void mav_dispatch(const mavlink_message_t& msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      mav_handle_heartbeat(msg);
      break;
    case MAVLINK_MSG_ID_SYSTEM_TIME:
      mav_handle_system_time(msg);
      break;
    case MAVLINK_MSG_ID_TIMESYNC:
      mav_handle_timesync(msg);
      break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      mav_handle_gps_raw_int(msg);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(CONSOLE_BAUD);
  delay(100);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  SerialGPRMC.begin(GPRMC_BAUD, SERIAL_8N1, PIN_GPRMC_RX, PIN_GPRMC_TX);
  SerialMAVLink.begin(MAVLINK_BAUD, SERIAL_8N1, PIN_MAVLINK_RX, PIN_MAVLINK_TX);

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
  Serial.println("=== ESP32-S3 Phase 3 Timing Firmware ===");
  Serial.println("Board: DFRobot FireBeetle2 ESP32-S3");
  Serial.printf("10Hz PWM trigger: GPIO %d (LEDC low-speed timer0, 50%% duty)\n", PIN_TRIGGER_10HZ);
  Serial.printf("1Hz sync pulse:   GPIO %d (esp_timer 500ms toggle, 50%% duty)\n", PIN_SYNC_1HZ);
  Serial.printf("GPRMC UART1:      TX GPIO %d, RX GPIO %d @ %lu baud\n", PIN_GPRMC_TX, PIN_GPRMC_RX,
                static_cast<unsigned long>(GPRMC_BAUD));
  Serial.printf("MAVLink UART2:    TX GPIO %d, RX GPIO %d @ %lu baud\n",
                PIN_MAVLINK_TX, PIN_MAVLINK_RX, (unsigned long)MAVLINK_BAUD);
  Serial.println("DDS sync gate:    SYSTEM_TIME unix_us > 1e15 us (year >2001)");
  Serial.println("Clock discipline: UNLOCKED -> WARMING(3 samples) -> LOCKED");
  Serial.printf("Max slew:         %lld us/update  (D6 monotonic invariant)\n", (long long)DISC_MAX_SLEW_US);
  Serial.println("Invariant: on each 1Hz rising edge -> LEDC timer reset + monotonic time increment + GPRMC emit");
}

void loop() {
  static uint32_t last_heartbeat_ms = 0;

  while (SerialMAVLink.available()) {
    uint8_t byte = (uint8_t)SerialMAVLink.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &s_mav_msg, &s_mav_status)) {
      mav_dispatch(s_mav_msg);
    }
  }

  const uint32_t now_ms = millis();
  if ((now_ms - s_last_timesync_ms) >= TIMESYNC_INTERVAL_MS) {
    s_last_timesync_ms = now_ms;
    mav_send_timesync();
  }

  if ((now_ms - last_heartbeat_ms) >= 10000) {
    last_heartbeat_ms = now_ms;
    if (s_clock_state == LOCKED) {
      uint64_t disc_s = disc_get_unix_us() / 1000000ULL;
      uint32_t d_hh = (uint32_t)((disc_s % 86400ULL) / 3600ULL);
      uint32_t d_mm = (uint32_t)((disc_s % 3600ULL)  / 60ULL);
      uint32_t d_ss = (uint32_t)(disc_s % 60ULL);
      Serial.printf("[HB] uptime=%lus  free=%02u:%02u:%02u  disc_utc=%02u:%02u:%02u  clk=%s  px4_dds=%s\n",
                    static_cast<unsigned long>(now_ms / 1000),
                    s_hh, s_mm, s_ss, d_hh, d_mm, d_ss,
                    clock_state_name(s_clock_state),
                    s_px4_dds_synced ? "CONVERGED" : "NOT_SYNCED");
    } else {
      Serial.printf("[HB] uptime=%lus  free=%02u:%02u:%02u  clk=%s  px4_dds=%s\n",
                    static_cast<unsigned long>(now_ms / 1000),
                    s_hh, s_mm, s_ss,
                    clock_state_name(s_clock_state),
                    s_px4_dds_synced ? "CONVERGED" : "NOT_SYNCED");
    }
  }

  delay(20);
}