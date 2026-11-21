#include "mgos.h"
#include "mgos_app.h"
#include "mgos_config.h"
#include "mgos_gpio.h"
#include "mgos_http_server.h"
#include "mgos_mqtt.h"
#include "mgos_net.h"
#include "mgos_sys_config.h"
#include "mgos_system.h"
#include "mgos_timers.h"
#include "mgos_i2c.h"
#include "mgos_bme280.h"
#include "mongoose.h"

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
// 1. Датчики (BME280 даёт 3 значения: температуру, влажность и давление)
// ============================================================
struct app_sensor {
  const char *id;
  const char *type;
  const char *unit;
  double last;
  uint32_t seq;
};

static struct app_sensor s_sensors[] = {
  { .id = "t1", .type = "bme280.t", .unit = "C" },
  { .id = "h1", .type = "bme280.h", .unit = "%" },
  { .id = "p1", .type = "bme280.p", .unit = "hPa" },
};
#define SENSOR_COUNT ((int) (sizeof(s_sensors) / sizeof(s_sensors[0])))

// У модулей GY-BME280 SDO обычно посажен на GND -> 0x76.
// Если SDO подтянут к 3.3 V — адрес 0x77.
#define BME280_I2C_ADDR 0x76

static struct mgos_bme280 *s_bme = NULL;

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
  snprintf(ap_ssid, sizeof(ap_ssid), "bme280-%s", s_dev_id);
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

// ---------- helpers для формы настройки ----------

static int hexval(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

// Достать поле из urlencoded-тела POST вида "k=v&k=v". Вернёт 1, если поле найдено.
static int form_get(const struct mg_str *body, const char *key, char *out, size_t out_len) {
  size_t klen = strlen(key), i = 0;
  if (out_len == 0) return 0;
  out[0] = '\0';
  while (i + klen + 1 <= body->len) {
    if ((i == 0 || body->p[i - 1] == '&') &&
        memcmp(body->p + i, key, klen) == 0 && body->p[i + klen] == '=') {
      i += klen + 1;
      size_t o = 0;
      while (i < body->len && body->p[i] != '&') {
        char ch = body->p[i];
        if (ch == '+') {
          ch = ' ';
        } else if (ch == '%' && i + 2 < body->len &&
                   hexval(body->p[i + 1]) >= 0 && hexval(body->p[i + 2]) >= 0) {
          ch = (char) (hexval(body->p[i + 1]) * 16 + hexval(body->p[i + 2]));
          i += 2;
        }
        if (o + 1 < out_len) out[o++] = ch;
        i++;
      }
      out[o] = '\0';
      return 1;
    }
    i++;
  }
  return 0;
}

static void json_escape_str(const char *src, char *dst, size_t dst_len) {
  size_t o = 0;
  if (src == NULL) src = "";
  for (size_t i = 0; src[i] != '\0' && o + 2 < dst_len; i++) {
    unsigned char ch = (unsigned char) src[i];
    if (ch == '"' || ch == '\\') { dst[o++] = '\\'; dst[o++] = (char) ch; }
    else if (ch >= 0x20) dst[o++] = (char) ch;
  }
  dst[o] = '\0';
}

// Текущие настройки для подстановки в форму
static void http_config_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "GET")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }

  static char ssid[80], pass[140], server[280], user[80], mpwd[140];
  static char body[1024];
  json_escape_str(mgos_sys_config_get_wifi_sta_ssid(), ssid, sizeof(ssid));
  json_escape_str(mgos_sys_config_get_wifi_sta_pass(), pass, sizeof(pass));
  json_escape_str(mgos_sys_config_get_mqtt_server(), server, sizeof(server));
  json_escape_str(mgos_sys_config_get_mqtt_user(), user, sizeof(user));
  json_escape_str(mgos_sys_config_get_mqtt_pass(), mpwd, sizeof(mpwd));

  struct mgos_net_ip_info ninfo;
  bool sta_connected = (mgos_net_get_ip_info(MGOS_NET_IF_TYPE_WIFI, 0, &ninfo) &&
                        ninfo.ip.sin_addr.s_addr != 0);
  char sta_ip[16] = "";
  if (sta_connected) mgos_net_ip_to_str(&ninfo.ip, sta_ip);

  snprintf(body, sizeof(body),
           "{\"dev\":\"%s\",\"sta_ssid\":\"%s\",\"sta_pass\":\"%s\","
           "\"mqtt_server\":\"%s\",\"mqtt_user\":\"%s\",\"mqtt_pass\":\"%s\","
           "\"sta_on\":%d,\"ap_on\":%d,\"sta_connected\":%d,\"sta_ip\":\"%s\"}",
           s_dev_id, ssid, pass, server, user, mpwd,
           (int) mgos_sys_config_get_wifi_sta_enable(),
           (int) mgos_sys_config_get_wifi_ap_enable(),
           (int) sta_connected, sta_ip);
  app_http_reply(c, 200, body);
}

// Сохранить настройки роутера и MQTT, перейти в режим STA и перезагрузиться
static void http_setup_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "POST")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }
  struct http_message *hm = (struct http_message *) ev_data;

  char sta_ssid[33] = {0}, sta_pass[65] = {0};
  char mqtt_server[129] = {0}, mqtt_user[33] = {0}, mqtt_pass[65] = {0};
  form_get(&hm->body, "sta_ssid", sta_ssid, sizeof(sta_ssid));
  form_get(&hm->body, "sta_pass", sta_pass, sizeof(sta_pass));
  form_get(&hm->body, "mqtt_server", mqtt_server, sizeof(mqtt_server));
  form_get(&hm->body, "mqtt_user", mqtt_user, sizeof(mqtt_user));
  form_get(&hm->body, "mqtt_pass", mqtt_pass, sizeof(mqtt_pass));

  if (sta_ssid[0] == '\0') {
    app_http_reply(c, 400, "{\"ok\":false,\"error\":\"ssid required\"}");
    return;
  }

  mgos_sys_config_set_wifi_sta_ssid(sta_ssid);
  mgos_sys_config_set_wifi_sta_pass(sta_pass);
  mgos_sys_config_set_mqtt_server(mqtt_server);
  mgos_sys_config_set_mqtt_user(mqtt_user);
  mgos_sys_config_set_mqtt_pass(mqtt_pass);
  mgos_sys_config_set_mqtt_enable(mqtt_server[0] != '\0');
  mgos_sys_config_set_wifi_sta_enable(true);
  mgos_sys_config_set_wifi_ap_enable(false);

  if (!mgos_sys_config_save(&mgos_sys_config, false, NULL)) {
    app_http_reply(c, 500, "{\"ok\":false,\"error\":\"save failed\"}");
    return;
  }
  app_http_reply(c, 200, "{\"ok\":true}");
  LOG(LL_INFO, ("Setup saved: sta_ssid='%s' mqtt='%s', rebooting", sta_ssid, mqtt_server));
  mgos_system_restart_after(800);
}

// ============================================================
// 6. Телеметрия (BME280)
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
  if (s_bme == NULL) return;

  struct mgos_bme280_data d;
  if (mgos_bme280_read(s_bme, &d) != 0) {
    LOG(LL_WARN, ("BME280 read failed"));
    return;
  }

  // Либа отдаёт давление в Па — переводим в гектопаскали
  double press_hpa = d.press / 100.0;

  // Обновляем локальное состояние и публикуем
  s_sensors[0].last = d.temp;
  s_sensors[1].last = d.humid;
  s_sensors[2].last = press_hpa;

  if (mgos_mqtt_global_is_connected()) {
    publish_sensor(&s_sensors[0], d.temp);
    publish_sensor(&s_sensors[1], d.humid);
    publish_sensor(&s_sensors[2], press_hpa);
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
// 7b. Страж WiFi: нет IP у STA — поднимаем точку доступа для настройки
// ============================================================
#define WIFI_GUARD_INTERVAL_MS 60000
static bool s_sta_had_ip = false;

static void wifi_guard_cb(void *arg) {
  (void) arg;

  // Чистый режим настройки (AP без STA) — ничего делать не надо
  if (!mgos_sys_config_get_wifi_sta_enable()) return;

  struct mgos_net_ip_info info;
  bool have_ip = (mgos_net_get_ip_info(MGOS_NET_IF_TYPE_WIFI, 0, &info) &&
                  info.ip.sin_addr.s_addr != 0);

  // Логируем только смену состояния, чтобы не спамить каждую минуту
  if (have_ip && !s_sta_had_ip) {
    char ip[16];
    LOG(LL_INFO, ("STA connected, ip=%s", mgos_net_ip_to_str(&info.ip, ip)));
  }
  if (!have_ip && s_sta_had_ip) {
    LOG(LL_WARN, ("STA connection lost"));
  }
  s_sta_had_ip = have_ip;
  if (have_ip) return;

  // Точка доступа уже поднята — настроиться можно, перезагрузка не нужна
  if (mgos_sys_config_get_wifi_ap_enable()) return;

  LOG(LL_WARN, ("STA has no IP, enabling setup AP (wifi '%s')",
                mgos_sys_config_get_wifi_sta_ssid()));
  mgos_sys_config_set_wifi_ap_enable(true);
  if (mgos_sys_config_save(&mgos_sys_config, false, NULL)) {
    mgos_system_restart_after(500);
  }
}

// ============================================================
// 8. init
// ============================================================
enum mgos_app_init_result mgos_app_init(void) {
  make_dev_id();
  LOG(LL_INFO, ("devId=%s base=%s", s_dev_id, s_base));

  config_fix_and_maybe_reboot();

  // Инициализация BME280
  s_bme = mgos_bme280_i2c_create(BME280_I2C_ADDR);
  if (s_bme == NULL) {
    LOG(LL_ERROR, ("Failed to init BME280 on I2C (check wiring, SDA=21, SCL=22, addr 0x%02X)", BME280_I2C_ADDR));
  } else {
    LOG(LL_INFO, ("BME280 initialized successfully (%s)",
                  mgos_bme280_is_bme280(s_bme) ? "BME280" : "BMP280"));
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    s_sensors[i].last = NAN;
    s_sensors[i].seq = 0;
  }

  mgos_register_http_endpoint("/sensor", http_sensor_handler, NULL);
  mgos_register_http_endpoint("/config", http_config_handler, NULL);
  mgos_register_http_endpoint("/setup",  http_setup_handler,  NULL);

  char cmd_topic[80];
  snprintf(cmd_topic, sizeof(cmd_topic), "%s/cmd", s_base);
  mgos_mqtt_sub(cmd_topic, mqtt_cmd_cb, NULL);

  mgos_set_timer(5000, MGOS_TIMER_REPEAT, telemetry_tick_cb, NULL);
  mgos_set_timer(1000, MGOS_TIMER_REPEAT, conn_state_tick_cb, NULL);

  // Если к роутеру не подключились (сразу после настройки или потеряли
  // связь позже) — поднимем точку доступа, чтобы можно было
  // посмотреть/поменять настройки; STA при этом продолжает попытки
  mgos_set_timer(WIFI_GUARD_INTERVAL_MS, MGOS_TIMER_REPEAT, wifi_guard_cb, NULL);

  LOG(LL_INFO, ("Ready: devId=%s", s_dev_id));
  return MGOS_APP_INIT_SUCCESS;
}
