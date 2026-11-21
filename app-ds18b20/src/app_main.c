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
#include "mgos_onewire.h"
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
// 1. Датчики (DS18B20, до MAX_DS штук на одной 1-Wire шине)
// ============================================================
#define MAX_DS 8

struct app_sensor {
  const char *id;
  const char *type;
  const char *unit;
  double last;
  uint32_t seq;
};

static struct app_sensor s_sensors[MAX_DS];
static char s_ids[MAX_DS][4];       // "t1" .. "t8"
static uint8_t s_addrs[MAX_DS][8];  // ROM-адреса в отсортированном порядке
static int s_sensor_count = 0;

// ============================================================
// 2. 1-Wire драйвер DS18B20
// ============================================================
#define DS18B20_FAMILY 0x28
#define DS18B20_CMD_CONVERT   0x44
#define DS18B20_CMD_READ_SCR  0xBE

static struct mgos_onewire *s_ow = NULL;
static bool s_converting = false;

// Найти все DS18B20 на шине. Порядок — по ROM-адресу, поэтому id (t1..tN)
// стабильны между перезагрузками и не зависят от порядка подключения.
static int ds18b20_scan(void) {
  if (s_ow == NULL) return 0;
  uint8_t addr[8];
  int n = 0;

  mgos_onewire_target_setup(s_ow, DS18B20_FAMILY);
  while (n < MAX_DS && mgos_onewire_next(s_ow, addr, 0)) {
    if (addr[0] != DS18B20_FAMILY) continue;  // пропускаем чужие 1-Wire устройства
    if (mgos_onewire_crc8(addr, 7) != addr[7]) {
      LOG(LL_WARN, ("DS18B20: ROM CRC mismatch, пропускаю устройство"));
      continue;
    }
    memcpy(s_addrs[n], addr, 8);
    n++;
  }

  for (int i = 1; i < n; i++) {  // сортировка вставками по 8-байтовому адресу
    uint8_t tmp[8];
    memcpy(tmp, s_addrs[i], 8);
    int j = i - 1;
    while (j >= 0 && memcmp(s_addrs[j], tmp, 8) > 0) {
      memcpy(s_addrs[j + 1], s_addrs[j], 8);
      j--;
    }
    memcpy(s_addrs[j + 1], tmp, 8);
  }
  return n;
}

static void ds18b20_convert_all(void) {
  mgos_onewire_reset(s_ow);
  mgos_onewire_skip(s_ow);                        // обращение ко ВСЕМ устройствам
  mgos_onewire_write(s_ow, DS18B20_CMD_CONVERT);  // одновременный старт конверсии
}

static bool ds18b20_read_addr(const uint8_t *addr, double *out) {
  mgos_onewire_reset(s_ow);
  mgos_onewire_select(s_ow, addr);
  mgos_onewire_write(s_ow, DS18B20_CMD_READ_SCR);

  uint8_t sp[9];
  mgos_onewire_read_bytes(s_ow, sp, 9);
  if (mgos_onewire_crc8(sp, 8) != sp[8]) return false;

  int16_t raw = (int16_t) ((sp[1] << 8) | sp[0]);
  uint8_t cfg = sp[4] & 0x60;  // маска под текущее разрешение
  if (cfg == 0x00) raw &= ~7;
  else if (cfg == 0x20) raw &= ~3;
  else if (cfg == 0x40) raw &= ~1;

  double t = raw * 0.0625;
  if (t < -25 || t > 125) return false;  // фильтр мусора на линии
  *out = t;
  return true;
}

static void ds18b20_log_bus(int n) {
  LOG(LL_INFO, ("DS18B20: found %d sensor(s) on GPIO%d", n, mgos_sys_config_get_app_ds_pin()));
  for (int i = 0; i < n; i++) {
    LOG(LL_INFO, ("  %s = %02X%02X%02X%02X%02X%02X%02X%02X",
                  s_sensors[i].id,
                  s_addrs[i][0], s_addrs[i][1], s_addrs[i][2], s_addrs[i][3],
                  s_addrs[i][4], s_addrs[i][5], s_addrs[i][6], s_addrs[i][7]));
  }
}

// ============================================================
// 3. Идентичность — из MAC, с самовосстановлением
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
// 4. WiFi STA/AP
// ============================================================
static void switch_wifi(bool sta_on, bool ap_on) {
  char ap_ssid[40];
  snprintf(ap_ssid, sizeof(ap_ssid), "ds18b20-%s", s_dev_id);
  mgos_sys_config_set_wifi_sta_enable(sta_on);
  mgos_sys_config_set_wifi_ap_enable(ap_on);
  mgos_sys_config_set_wifi_ap_ssid(ap_ssid);
  bool ok = mgos_sys_config_save(&mgos_sys_config, false, NULL);
  LOG(LL_INFO, ("WiFi: sta=%d ap=%d ssid='%s' saved=%d", sta_on, ap_on, ap_ssid, ok));
  if (ok) mgos_system_restart_after(500);
}

// ============================================================
// 5. MQTT
// ============================================================
static void publish_birth(void) {
  char topic[80];
  snprintf(topic, sizeof(topic), "%s/status", s_base);
  mgos_mqtt_pub(topic, "online", 6, 1, true);

  char meta[512];
  int n = snprintf(meta, sizeof(meta),
                   "{\"fw\":\"1.0.0\",\"dev\":\"%s\",\"mac\":\"%s\",\"sensors\":[",
                   s_dev_id, mgos_sys_ro_vars_get_mac_address());
  for (int i = 0; i < s_sensor_count && n < (int) sizeof(meta) - 64; i++) {
    n += snprintf(meta + n, sizeof(meta) - n,
                  "%s{\"id\":\"%s\",\"type\":\"%s\",\"unit\":\"%s\"}",
                  (i == 0 ? "" : ","), s_sensors[i].id, s_sensors[i].type, s_sensors[i].unit);
  }
  snprintf(meta + n, sizeof(meta) - n, "]}");

  snprintf(topic, sizeof(topic), "%s/meta", s_base);
  mgos_mqtt_pub(topic, meta, strlen(meta), 1, true);
  LOG(LL_INFO, ("Published birth + meta (%d sensors)", s_sensor_count));
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
// 6. HTTP
// ============================================================
static void http_sensor_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "GET")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }

  char body[512];
  int n = snprintf(body, sizeof(body), "{\"device\":\"%s\",\"ts\":%lld,\"sensors\":{", s_dev_id, (long long) time(NULL));
  for (int i = 0; i < s_sensor_count && n < (int) sizeof(body) - 32; i++) {
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
// 7. Телеметрия (DS18B20)
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

// Чтение результата конверсии — отдельным таймером, чтобы не блокировать
// цикл событий ожиданием 750 мс
static void ds_read_all_cb(void *arg) {
  (void) arg;
  s_converting = false;

  for (int i = 0; i < s_sensor_count; i++) {
    double t;
    if (ds18b20_read_addr(s_addrs[i], &t)) {
      s_sensors[i].last = t;
      if (mgos_mqtt_global_is_connected()) {
        publish_sensor(&s_sensors[i], t);
      }
    } else {
      LOG(LL_WARN, ("DS18B20 %s: read failed (провода/подтяжка?)", s_sensors[i].id));
    }
  }
}

static void telemetry_tick_cb(void *arg) {
  (void) arg;
  if (s_ow == NULL) return;
  if (s_converting) return;  // прошлая конверсия ещё не дочитана

  int n = ds18b20_scan();
  if (n == 0) {
    if (s_sensor_count != 0) {
      LOG(LL_WARN, ("DS18B20: all sensors lost (провода/подтяжка?)"));
      s_sensor_count = 0;
    }
    return;
  }
  if (n != s_sensor_count) {
    s_sensor_count = n;
    ds18b20_log_bus(n);
  }

  ds18b20_convert_all();  // старт конверсии на ВСЕХ датчиках разом
  s_converting = true;
  mgos_set_timer(mgos_sys_config_get_app_ds_convert_ms(), false, ds_read_all_cb, NULL);
}

// ============================================================
// 8. Birth после реконнекта
// ============================================================
static bool s_connected_prev = false;
static void conn_state_tick_cb(void *arg) {
  (void) arg;
  bool now = mgos_mqtt_global_is_connected();
  if (now && !s_connected_prev) publish_birth();
  s_connected_prev = now;
}

// ============================================================
// 8b. Страж WiFi: нет IP у STA — поднимаем точку доступа для настройки
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
// 9. init
// ============================================================
enum mgos_app_init_result mgos_app_init(void) {
  make_dev_id();
  LOG(LL_INFO, ("devId=%s base=%s", s_dev_id, s_base));

  config_fix_and_maybe_reboot();

  for (int i = 0; i < MAX_DS; i++) {
    snprintf(s_ids[i], sizeof(s_ids[i]), "t%d", i + 1);
    s_sensors[i].id = s_ids[i];
    s_sensors[i].type = "ds18b20.t";
    s_sensors[i].unit = "C";
    s_sensors[i].last = NAN;
    s_sensors[i].seq = 0;
  }

  // Шина 1-Wire: все DS18B20 на одном GPIO (подтяжка 4.7к к 3.3 V)
  s_ow = mgos_onewire_create(mgos_sys_config_get_app_ds_pin());
  if (s_ow == NULL) {
    LOG(LL_ERROR, ("Failed to init 1-Wire on GPIO%d", mgos_sys_config_get_app_ds_pin()));
  } else {
    s_sensor_count = ds18b20_scan();
    if (s_sensor_count == 0) {
      LOG(LL_WARN, ("DS18B20 not found (check data GPIO%d, 4.7k pull-up to 3.3V)",
                    mgos_sys_config_get_app_ds_pin()));
    } else {
      ds18b20_log_bus(s_sensor_count);
    }
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
