#include "mgos.h"
#include "mgos_app.h"
#include "mgos_config.h"
#include "mgos_gpio.h"
#include "mgos_http_server.h"
#include "mgos_mqtt.h"
#include "mgos_sys_config.h"
#include "mgos_system.h"
#include "mgos_timers.h"
#include "mgos_i2c.h"
#include "mongoose.h"

#include "app_aht10.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ctype.h>

// C-хелперы из http_reply.c
int app_http_method_is(void *ev_data, const char *method);
void app_http_reply(struct mg_connection *c, int code, const char *body);

// ============================================================
// 1. Датчики (AHT10 даёт 2 значения: температуру и влажность)
// ============================================================
struct app_sensor {
  const char *id;
  const char *type;
  const char *unit;
  double last;
  uint32_t seq;
};

static struct app_sensor s_sensors[] = {
  { .id = "t1", .type = "aht10.t", .unit = "C" },
  { .id = "h1", .type = "aht10.h", .unit = "%" },
};
#define SENSOR_COUNT ((int) (sizeof(s_sensors) / sizeof(s_sensors[0])))

#define AHT10_I2C_ADDR 0x38

static struct app_aht10 *s_aht = NULL;

// ============================================================
// 2. Идентичность — из MAC, с самовосстановлением
// ============================================================
static char s_dev_id[24];
static char s_base[56];       // home/sensors/esp-xxxxxx
static char s_will_topic[64];

static void make_dev_id(void) {
  const char *mac = mgos_sys_ro_vars_get_mac_address();
  char hex[16] = {0};
  int j = 0;
  for (int i = 0; mac != NULL && mac[i] != '\0' && j < (int) sizeof(hex) - 1; i++) {
    if (mac[i] == ':') continue;
    hex[j++] = (char) tolower((unsigned char) mac[i]);
  }
  hex[j] = '\0';
  const char *tail = (j >= 6) ? (hex + j - 6) : "000000";
  snprintf(s_dev_id, sizeof(s_dev_id), "esp-%s", tail);
  
  // ВАЖНО: используем home/sensors/, чтобы попасть под ACL брокера
  snprintf(s_base, sizeof(s_base), "home/sensors/%s", s_dev_id);
  snprintf(s_will_topic, sizeof(s_will_topic), "%s/status", s_base);
}

static void config_fix_and_maybe_reboot(void) {
  bool need_fix = false;

  const char *cur_dev    = mgos_sys_config_get_device_id();
  const char *cur_client = mgos_sys_config_get_mqtt_client_id();
  const char *cur_will   = mgos_sys_config_get_mqtt_will_topic();

  if (cur_dev == NULL || strcmp(cur_dev, s_dev_id) != 0) {
    LOG(LL_INFO, ("Fix: device.id '%s' -> '%s'", cur_dev ? cur_dev : "", s_dev_id));
    mgos_sys_config_set_device_id(s_dev_id);
    need_fix = true;
  }
  if (cur_client == NULL || strcmp(cur_client, s_dev_id) != 0) {
    LOG(LL_INFO, ("Fix: mqtt.client_id '%s' -> '%s'", cur_client ? cur_client : "", s_dev_id));
    mgos_sys_config_set_mqtt_client_id(s_dev_id);
    need_fix = true;
  }
  if (cur_will == NULL || strcmp(cur_will, s_will_topic) != 0) {
    LOG(LL_INFO, ("Fix: mqtt.will_topic -> '%s'", s_will_topic));
    mgos_sys_config_set_mqtt_will_topic(s_will_topic);
    mgos_sys_config_set_mqtt_will_message("offline");
    mgos_sys_config_set_mqtt_will_retain(true);
    need_fix = true;
  }

  if (!need_fix) {
    if (mgos_sys_config_get_app_fix_count() != 0) {
      mgos_sys_config_set_app_fix_count(0);
      mgos_sys_config_save(&mgos_sys_config, false, NULL);
    }
    return;
  }

  if (!mgos_sys_config_save(&mgos_sys_config, false, NULL)) {
    LOG(LL_ERROR, ("Cfg save failed — NOT rebooting (would loop)"));
    return;
  }

  int cnt = mgos_sys_config_get_app_fix_count() + 1;
  mgos_sys_config_set_app_fix_count(cnt);
  mgos_sys_config_save(&mgos_sys_config, false, NULL);

  if (cnt > 3) {
    LOG(LL_ERROR, ("Fix did not stick after %d attempts, staying up", cnt));
    return;
  }
  LOG(LL_INFO, ("Rebooting to apply (attempt %d)", cnt));
  mgos_system_restart_after(300);
}

// ============================================================
// 3. WiFi STA/AP
// ============================================================
static void switch_wifi(bool sta_on, bool ap_on) {
  char ap_ssid[40];
  snprintf(ap_ssid, sizeof(ap_ssid), "aht10-%s", s_dev_id);
  mgos_sys_config_set_wifi_sta_enable(sta_on);
  mgos_sys_config_set_wifi_ap_enable(ap_on);
  mgos_sys_config_set_wifi_ap_ssid(ap_ssid);
  bool ok = mgos_sys_config_save(&mgos_sys_config, false, NULL);
  LOG(LL_INFO, ("WiFi: sta=%d ap=%d ssid='%s' saved=%d", sta_on, ap_on, ap_ssid, ok));
  if (ok) mgos_system_restart_after(500);
}

// ============================================================
// 4. MQTT
// ============================================================
static void publish_birth(void) {
  char topic[80];
  snprintf(topic, sizeof(topic), "%s/status", s_base);
  mgos_mqtt_pub(topic, "online", 6, 1, true);

  char meta[512];
  int n = snprintf(meta, sizeof(meta),
                   "{\"fw\":\"1.0.0\",\"dev\":\"%s\",\"mac\":\"%s\",\"sensors\":[",
                   s_dev_id, mgos_sys_ro_vars_get_mac_address());
  for (int i = 0; i < SENSOR_COUNT && n < (int) sizeof(meta) - 64; i++) {
    n += snprintf(meta + n, sizeof(meta) - n,
                  "%s{\"id\":\"%s\",\"type\":\"%s\",\"unit\":\"%s\"}",
                  (i == 0 ? "" : ","), s_sensors[i].id, s_sensors[i].type, s_sensors[i].unit);
  }
  snprintf(meta + n, sizeof(meta) - n, "]}");

  snprintf(topic, sizeof(topic), "%s/meta", s_base);
  mgos_mqtt_pub(topic, meta, strlen(meta), 1, true);
  LOG(LL_INFO, ("Published birth + meta"));
}

static void mqtt_cmd_cb(struct mg_connection *c, const char *topic, int topic_len,
                        const char *msg, int msg_len, void *ud) {
  (void) c; (void) topic_len; (void) ud;
  char buf[32] = {0};
  int n = msg_len < (int) sizeof(buf) - 1 ? msg_len : (int) sizeof(buf) - 1;
  memcpy(buf, msg, n);
  LOG(LL_INFO, ("cmd %s -> %s", topic, buf));

  if (strcmp(buf, "ap") == 0)  switch_wifi(false, true);
  if (strcmp(buf, "sta") == 0) switch_wifi(true, false);
}

// ============================================================
// 5. HTTP
// ============================================================
static void http_sensor_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "GET")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }

  char body[512];
  int n = snprintf(body, sizeof(body), "{\"device\":\"%s\",\"ts\":%lld,\"sensors\":{", s_dev_id, (long long) time(NULL));
  for (int i = 0; i < SENSOR_COUNT && n < (int) sizeof(body) - 32; i++) {
    if (isnan(s_sensors[i].last)) {
      n += snprintf(body + n, sizeof(body) - n, "%s\"%s\":null", (i == 0 ? "" : ","), s_sensors[i].id);
    } else {
      n += snprintf(body + n, sizeof(body) - n, "%s\"%s\":%.2f", (i == 0 ? "" : ","), s_sensors[i].id, s_sensors[i].last);
    }
  }
  snprintf(body + n, sizeof(body) - n, "}}");
  app_http_reply(c, 200, body);
}

static void http_back_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "POST")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }
  app_http_reply(c, 200, "{\"ok\":true}");
  switch_wifi(true, false);
}

// ============================================================
// 6. Телеметрия (AHT10)
// ============================================================
static void publish_sensor(struct app_sensor *s, double v) {
  char topic[80];
  snprintf(topic, sizeof(topic), "%s/sensor/%s/state", s_base, s->id);

  time_t t = time(NULL);
  int64_t ts = (t > 1600000000) ? (int64_t) t : 0;

  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"v\":%.2f,\"u\":\"%s\",\"ts\":%lld,\"seq\":%u}",
           v, s->unit ? s->unit : "", (long long) ts, (unsigned) s->seq++);

  bool ok = mgos_mqtt_pub(topic, payload, strlen(payload), 0, true);
  LOG(LL_INFO, ("pub %s %s -> %d", topic, payload, ok));
}

static void telemetry_tick_cb(void *arg) {
  (void) arg;
  if (s_aht == NULL) return;

  double temp = NAN, hum = NAN;
  if (!app_aht10_read(s_aht, &temp, &hum)) {
    LOG(LL_WARN, ("AHT10 read failed"));
    return;
  }

  // Обновляем локальное состояние и публикуем
  s_sensors[0].last = temp;
  s_sensors[1].last = hum;

  if (mgos_mqtt_global_is_connected()) {
    publish_sensor(&s_sensors[0], temp);
    publish_sensor(&s_sensors[1], hum);
  }
}

// ============================================================
// 7. Birth после реконнекта
// ============================================================
static bool s_connected_prev = false;
static void conn_state_tick_cb(void *arg) {
  (void) arg;
  bool now = mgos_mqtt_global_is_connected();
  if (now && !s_connected_prev) publish_birth();
  s_connected_prev = now;
}

// ============================================================
// 8. init
// ============================================================
enum mgos_app_init_result mgos_app_init(void) {
  make_dev_id();
  LOG(LL_INFO, ("devId=%s base=%s", s_dev_id, s_base));

  config_fix_and_maybe_reboot();

  // Инициализация AHT10
  s_aht = app_aht10_create(AHT10_I2C_ADDR);
  if (s_aht == NULL) {
    LOG(LL_ERROR, ("Failed to init AHT10 on I2C (check wiring, SDA=4, SCL=5)"));
  } else {
    LOG(LL_INFO, ("AHT10 initialized successfully"));
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    s_sensors[i].last = NAN;
    s_sensors[i].seq = 0;
  }

  mgos_register_http_endpoint("/sensor", http_sensor_handler, NULL);
  mgos_register_http_endpoint("/back",   http_back_handler,   NULL);

  char cmd_topic[80];
  snprintf(cmd_topic, sizeof(cmd_topic), "%s/cmd", s_base);
  mgos_mqtt_sub(cmd_topic, mqtt_cmd_cb, NULL);

  mgos_set_timer(5000, MGOS_TIMER_REPEAT, telemetry_tick_cb, NULL);
  mgos_set_timer(1000, MGOS_TIMER_REPEAT, conn_state_tick_cb, NULL);

  LOG(LL_INFO, ("Ready: devId=%s", s_dev_id));
  return MGOS_APP_INIT_SUCCESS;
}