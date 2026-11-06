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
#include "mongoose.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// C-хелперы из http_reply.c
int app_http_method_is(void *ev_data, const char *method);
void app_http_reply(struct mg_connection *c, int code, const char *body);

// ============================================================
// 1. Реле r1..r4 (пины из конфига, активный уровень из конфига)
// ============================================================
#define RELAY_COUNT 4

struct relay {
  const char *id;
  int pin;
  bool on;
};

static struct relay s_relays[RELAY_COUNT] = {
  { .id = "r1" }, { .id = "r2" }, { .id = "r3" }, { .id = "r4" },
};

static int relay_on_level(void) {
  return mgos_sys_config_get_app_relay_active_high() ? 1 : 0;
}

static void relay_apply(struct relay *r) {
  mgos_gpio_write(r->pin, r->on ? relay_on_level() : !relay_on_level());
}

static void relay_save_states(void) {
  int mask = 0;
  for (int i = 0; i < RELAY_COUNT; i++) {
    if (s_relays[i].on) mask |= (1 << i);
  }
  mgos_sys_config_set_app_relay_states(mask);
  mgos_sys_config_save(&mgos_sys_config, false, NULL);
}

static void relay_publish(struct relay *r) {
  char topic[80];
  snprintf(topic, sizeof(topic), "%s/relay/%s/state", s_base, r->id);
  const char *v = r->on ? "on" : "off";
  bool ok = mgos_mqtt_pub(topic, v, strlen(v), 1, true);
  LOG(LL_INFO, ("pub %s %s -> %d", topic, v, ok));
}

static void relay_set(struct relay *r, bool on, const char *src) {
  if (r->on == on) return;
  r->on = on;
  relay_apply(r);
  relay_save_states();
  LOG(LL_INFO, ("relay %s -> %s (%s)", r->id, on ? "ON" : "OFF", src));
  if (mgos_mqtt_global_is_connected()) relay_publish(r);
}

static struct relay *relay_by_id(const char *id) {
  for (int i = 0; i < RELAY_COUNT; i++) {
    if (strcmp(s_relays[i].id, id) == 0) return &s_relays[i];
  }
  return NULL;
}

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
  snprintf(ap_ssid, sizeof(ap_ssid), "relays-%s", s_dev_id);
  mgos_sys_config_set_wifi_sta_enable(sta_on);
  mgos_sys_config_set_wifi_ap_enable(ap_on);
  mgos_sys_config_set_wifi_ap_ssid(ap_ssid);
  bool ok = mgos_sys_config_save(&mgos_sys_config, false, NULL);
  LOG(LL_INFO, ("WiFi: sta=%d ap=%d ssid='%s' saved=%d", sta_on, ap_on, ap_ssid, ok));
  if (ok) mgos_system_restart_after(500);
}

// ============================================================
// 4. MQTT: birth + команды на реле
// ============================================================
static void publish_birth(void) {
  char topic[80];
  snprintf(topic, sizeof(topic), "%s/status", s_base);
  mgos_mqtt_pub(topic, "online", 6, 1, true);

  char meta[512];
  int n = snprintf(meta, sizeof(meta),
                   "{\"fw\":\"1.0.0\",\"dev\":\"%s\",\"mac\":\"%s\",\"active\":\"%s\",\"relays\":[",
                   s_dev_id, mgos_sys_ro_vars_get_mac_address(),
                   mgos_sys_config_get_app_relay_active_high() ? "high" : "low");
  for (int i = 0; i < RELAY_COUNT && n < (int) sizeof(meta) - 64; i++) {
    n += snprintf(meta + n, sizeof(meta) - n,
                  "%s{\"id\":\"%s\",\"gpio\":%d}",
                  (i == 0 ? "" : ","), s_relays[i].id, s_relays[i].pin);
  }
  snprintf(meta + n, sizeof(meta) - n, "]}");
  snprintf(topic, sizeof(topic), "%s/meta", s_base);
  mgos_mqtt_pub(topic, meta, strlen(meta), 1, true);

  // Отдать брокеру актуальные состояния — HA/скрипты увидят их сразу
  for (int i = 0; i < RELAY_COUNT; i++) relay_publish(&s_relays[i]);

  LOG(LL_INFO, ("Published birth + meta + relay states"));
}

static void mqtt_relay_cb(struct mg_connection *c, const char *topic, int topic_len,
                          const char *msg, int msg_len, void *ud) {
  (void) c; (void) ud;
  // Ожидаемый топик: <base>/relay/rN/set
  char t[96] = {0};
  if (topic_len >= (int) sizeof(t)) return;
  memcpy(t, topic, topic_len);

  char prefix[80];
  snprintf(prefix, sizeof(prefix), "%s/relay/", s_base);
  char *p = strstr(t, prefix);
  if (p == NULL) return;
  p += strlen(prefix);

  char id[4] = {0};
  if (p[0] == 'r' && p[1] >= '1' && p[1] <= '4' && p[2] == '/') {
    id[0] = p[0]; id[1] = p[1];
  } else {
    return;
  }
  struct relay *r = relay_by_id(id);
  if (r == NULL) return;

  char cmd[16] = {0};
  int n = msg_len < (int) sizeof(cmd) - 1 ? msg_len : (int) sizeof(cmd) - 1;
  for (int i = 0; i < n; i++) cmd[i] = (char) tolower((unsigned char) msg[i]);

  if (strcmp(cmd, "on") == 0)       relay_set(r, true, "mqtt");
  else if (strcmp(cmd, "off") == 0) relay_set(r, false, "mqtt");
  else if (strcmp(cmd, "toggle") == 0) relay_set(r, !r->on, "mqtt");
  else if (strcmp(cmd, "1") == 0)   relay_set(r, true, "mqtt");
  else if (strcmp(cmd, "0") == 0)   relay_set(r, false, "mqtt");
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
static void http_relays_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "GET")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }

  char body[256];
  int n = snprintf(body, sizeof(body), "{\"device\":\"%s\",\"relays\":{", s_dev_id);
  for (int i = 0; i < RELAY_COUNT && n < (int) sizeof(body) - 24; i++) {
    n += snprintf(body + n, sizeof(body) - n, "%s\"%s\":\"%s\"",
                  (i == 0 ? "" : ","), s_relays[i].id, s_relays[i].on ? "on" : "off");
  }
  snprintf(body + n, sizeof(body) - n, "}}");
  app_http_reply(c, 200, body);
}

static int hexval(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

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

static void http_relay_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "POST")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }
  struct http_message *hm = (struct http_message *) ev_data;

  char id[8] = {0}, state[12] = {0};
  form_get(&hm->body, "id", id, sizeof(id));
  form_get(&hm->body, "state", state, sizeof(state));

  struct relay *r = relay_by_id(id);
  if (r == NULL) {
    app_http_reply(c, 400, "{\"ok\":false,\"error\":\"unknown id\"}");
    return;
  }

  if (strcmp(state, "on") == 0)       relay_set(r, true, "http");
  else if (strcmp(state, "off") == 0) relay_set(r, false, "http");
  else if (strcmp(state, "toggle") == 0) relay_set(r, !r->on, "http");
  else {
    app_http_reply(c, 400, "{\"ok\":false,\"error\":\"state must be on|off|toggle\"}");
    return;
  }

  char body[64];
  snprintf(body, sizeof(body), "{\"ok\":true,\"id\":\"%s\",\"state\":\"%s\"}", r->id, r->on ? "on" : "off");
  app_http_reply(c, 200, body);
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
// 6. Birth после реконнекта
// ============================================================
static bool s_connected_prev = false;
static void conn_state_tick_cb(void *arg) {
  (void) arg;
  bool now = mgos_mqtt_global_is_connected();
  if (now && !s_connected_prev) publish_birth();
  s_connected_prev = now;
}

// ============================================================
// 6b. Страж WiFi: нет IP у STA — поднимаем точку доступа для настройки
// ============================================================
#define WIFI_GUARD_INTERVAL_MS 60000
static bool s_sta_had_ip = false;

static void wifi_guard_cb(void *arg) {
  (void) arg;

  if (!mgos_sys_config_get_wifi_sta_enable()) return;

  struct mgos_net_ip_info info;
  bool have_ip = (mgos_net_get_ip_info(MGOS_NET_IF_TYPE_WIFI, 0, &info) &&
                  info.ip.sin_addr.s_addr != 0);

  if (have_ip && !s_sta_had_ip) {
    char ip[16];
    LOG(LL_INFO, ("STA connected, ip=%s", mgos_net_ip_to_str(&info.ip, ip)));
  }
  if (!have_ip && s_sta_had_ip) {
    LOG(LL_WARN, ("STA connection lost"));
  }
  s_sta_had_ip = have_ip;
  if (have_ip) return;

  if (mgos_sys_config_get_wifi_ap_enable()) return;

  LOG(LL_WARN, ("STA has no IP, enabling setup AP (wifi '%s')",
                mgos_sys_config_get_wifi_sta_ssid()));
  mgos_sys_config_set_wifi_ap_enable(true);
  if (mgos_sys_config_save(&mgos_sys_config, false, NULL)) {
    mgos_system_restart_after(500);
  }
}

// ============================================================
// 7. init
// ============================================================
enum mgos_app_init_result mgos_app_init(void) {
  // Пины в безопасное состояние ДО всего прочего: GPIO15 active-high
  // на LOW = реле выключено и корректный уровень для загрузки.
  s_relays[0].pin = mgos_sys_config_get_app_r1_pin();
  s_relays[1].pin = mgos_sys_config_get_app_r2_pin();
  s_relays[2].pin = mgos_sys_config_get_app_r3_pin();
  s_relays[3].pin = mgos_sys_config_get_app_r4_pin();
  for (int i = 0; i < RELAY_COUNT; i++) {
    mgos_gpio_set_mode(s_relays[i].pin, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_set_pull(s_relays[i].pin, MGOS_GPIO_PULL_NONE);
  }

  make_dev_id();
  LOG(LL_INFO, ("devId=%s base=%s", s_dev_id, s_base));

  config_fix_and_maybe_reboot();

  // Начальное состояние: восстановить сохранённое или всё выключить
  int mask = mgos_sys_config_get_app_relay_states();
  bool restore = mgos_sys_config_get_app_relay_restore();
  for (int i = 0; i < RELAY_COUNT; i++) {
    s_relays[i].on = restore && (mask & (1 << i));
    relay_apply(&s_relays[i]);
    LOG(LL_INFO, ("relay %s: GPIO%d %s", s_relays[i].id, s_relays[i].pin,
                  s_relays[i].on ? "ON" : "OFF"));
  }
  if (!restore) {
    mgos_sys_config_set_app_relay_states(0);
    mgos_sys_config_save(&mgos_sys_config, false, NULL);
  }

  mgos_register_http_endpoint("/relays", http_relays_handler, NULL);
  mgos_register_http_endpoint("/relay",  http_relay_handler,  NULL);
  mgos_register_http_endpoint("/config", http_config_handler, NULL);
  mgos_register_http_endpoint("/setup",  http_setup_handler,  NULL);

  char topic[96];
  snprintf(topic, sizeof(topic), "%s/relay/+/set", s_base);
  mgos_mqtt_sub(topic, mqtt_relay_cb, NULL);
  snprintf(topic, sizeof(topic), "%s/cmd", s_base);
  mgos_mqtt_sub(topic, mqtt_cmd_cb, NULL);

  mgos_set_timer(1000, MGOS_TIMER_REPEAT, conn_state_tick_cb, NULL);
  mgos_set_timer(WIFI_GUARD_INTERVAL_MS, MGOS_TIMER_REPEAT, wifi_guard_cb, NULL);

  LOG(LL_INFO, ("Ready: devId=%s", s_dev_id));
  return MGOS_APP_INIT_SUCCESS;
}
