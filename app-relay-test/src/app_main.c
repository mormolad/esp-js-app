#include "mgos.h"
#include "mgos_app.h"
#include "mgos_config.h"
#include "mgos_gpio.h"
#include "mgos_http_server.h"
#include "mgos_sys_config.h"
#include "mgos_system.h"
#include "mgos_timers.h"
#include "mongoose.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// C-хелперы из http_reply.c
int app_http_method_is(void *ev_data, const char *method);
void app_http_reply(struct mg_connection *c, int code, const char *body);

// ============================================================
// Тестер пинов реле.
//
// Безопасные пины ESP8266: 4, 5, 12, 13, 14, 16 — покой LOW, импульс HIGH.
// Опасные (только по явному включению на странице):
//   GPIO 15 — стрэппинг, при загрузке нужен LOW (как наш покой) — самый
//             безопасный из опасных и частый пин для реле;
//   GPIO 0, 2 — стрэппинг, при загрузке нужен HIGH, поэтому покой HIGH,
//             импульс LOW;
//   GPIO 1, 3 — UART (консоль засорится во время импульса, для прошивки
//             по USB реле с этих пинов лучше отключить).
// НЕ трогаем никогда: 6-11 (подключены к флеш).
// ============================================================
static const int s_safe_pins[] = {4, 5, 12, 13, 14, 16};
#define SAFE_COUNT ((int) (sizeof(s_safe_pins) / sizeof(s_safe_pins[0])))

static const struct { int pin; int idle; } s_risky_pins[] = {
  {15, 0}, {0, 1}, {2, 1}, {1, 0}, {3, 0},
};
#define RISKY_COUNT ((int) (sizeof(s_risky_pins) / sizeof(s_risky_pins[0])))

struct test_pin {
  int pin;
  int idle;    // уровень покоя (безопасный для загрузки)
  int pulse;   // уровень импульса (противоположный)
};

static struct test_pin s_test_pins[SAFE_COUNT + RISKY_COUNT];
static int s_test_count = 0;
static bool s_risky_enabled = false;

#define STEP_HIGH_MS 1500
#define PULSE_MS 800

static bool s_running = false;
static int s_cur_idx = -1;   // индекс текущего пина в импульсе, -1 = покой
static int s_cycle = 0;

static void pin_setup_output(int pin) {
  mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_OUTPUT);
  mgos_gpio_set_pull(pin, MGOS_GPIO_PULL_NONE);
}

static void build_test_pins(void) {
  int n = 0;
  for (int i = 0; i < SAFE_COUNT; i++) {
    s_test_pins[n].pin = s_safe_pins[i];
    s_test_pins[n].idle = 0;
    s_test_pins[n].pulse = 1;
    n++;
  }
  if (s_risky_enabled) {
    for (int i = 0; i < RISKY_COUNT; i++) {
      s_test_pins[n].pin = s_risky_pins[i].pin;
      s_test_pins[n].idle = s_risky_pins[i].idle;
      s_test_pins[n].pulse = !s_risky_pins[i].idle;
      n++;
    }
  }
  s_test_count = n;
}

static bool pin_known(int pin) {
  for (int i = 0; i < s_test_count; i++) {
    if (s_test_pins[i].pin == pin) return true;
  }
  return false;
}

static void set_pins_idle(void) {
  for (int i = 0; i < s_test_count; i++) {
    mgos_gpio_write(s_test_pins[i].pin, s_test_pins[i].idle);
  }
}

// Шаг авто-сканирования: текущий пин в покой, следующий — в импульс
static void scan_step_cb(void *arg) {
  (void) arg;
  if (!s_running) return;

  if (s_cur_idx >= 0) mgos_gpio_write(s_test_pins[s_cur_idx].pin, s_test_pins[s_cur_idx].idle);

  s_cur_idx++;
  if (s_cur_idx >= s_test_count) {
    s_cur_idx = 0;
    s_cycle++;
  }
  int pin = s_test_pins[s_cur_idx].pin;
  mgos_gpio_write(pin, s_test_pins[s_cur_idx].pulse);
  LOG(LL_INFO, ("RELAY TEST: GPIO %d PULSE (cycle %d) — слушаем, какое реле щёлкнуло", pin, s_cycle));

  mgos_set_timer(STEP_HIGH_MS, false, scan_step_cb, NULL);
}

static void scan_start(void) {
  if (s_running) return;
  s_running = true;
  s_cur_idx = -1;
  s_cycle++;
  LOG(LL_INFO, ("RELAY TEST: сканирование запущено (%d пинов)", s_test_count));
  mgos_set_timer(300, false, scan_step_cb, NULL);
}

static void scan_stop(void) {
  s_running = false;
  s_cur_idx = -1;
  set_pins_idle();
  LOG(LL_INFO, ("RELAY TEST: сканирование остановлено, все пины в покое"));
}

static void pulse_idle_cb(void *arg) {
  int pin = (int) (intptr_t) arg;
  for (int i = 0; i < s_test_count; i++) {
    if (s_test_pins[i].pin == pin) {
      mgos_gpio_write(pin, s_test_pins[i].idle);
      return;
    }
  }
}

// Ручной импульс на один пин (работает и когда скан остановлен)
static void pulse_pin(int pin) {
  if (!pin_known(pin)) return;
  int level = -1;
  for (int i = 0; i < s_test_count; i++) {
    if (s_test_pins[i].pin == pin) level = s_test_pins[i].pulse;
  }
  if (level < 0) return;
  mgos_gpio_write(pin, level);
  LOG(LL_INFO, ("RELAY TEST: ручной импульс GPIO %d на %d мс", pin, PULSE_MS));
  mgos_set_timer(PULSE_MS, false, pulse_idle_cb, (void *) (intptr_t) pin);
}

// Включить/выключить опасные пины. При выключении пины остаются выходами
// и держатся на безопасном для загрузки уровне — перезагрузка не сломается.
static void set_risky(bool on) {
  if (s_risky_enabled == on) return;
  scan_stop();
  s_risky_enabled = on;
  build_test_pins();
  for (int i = 0; i < RISKY_COUNT; i++) {
    pin_setup_output(s_risky_pins[i].pin);
    mgos_gpio_write(s_risky_pins[i].pin, s_risky_pins[i].idle);
  }
  LOG(LL_INFO, ("RELAY TEST: опасные пины %s", on ? "ВКЛ (0, 2, 15, 1, 3)" : "ВЫКЛ"));
}

// ============================================================
// HTTP
// ============================================================
static void http_status_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "GET")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }

  int pin = (s_cur_idx >= 0) ? s_test_pins[s_cur_idx].pin : -1;
  char body[160];
  snprintf(body, sizeof(body), "{\"running\":%d,\"pin\":%d,\"cycle\":%d,\"risky\":%d}",
           s_running ? 1 : 0, pin, s_cycle, s_risky_enabled ? 1 : 0);
  app_http_reply(c, 200, body);
}

static void http_start_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "POST")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }
  scan_start();
  app_http_reply(c, 200, "{\"ok\":true}");
}

static void http_stop_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "POST")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }
  scan_stop();
  app_http_reply(c, 200, "{\"ok\":true}");
}

static int decval(char ch) {
  return (ch >= '0' && ch <= '9') ? ch - '0' : -1;
}

// Достать числовое поле из urlencoded-тела POST вида "pin=12&on=1"
static int form_get_int(const struct mg_str *body, const char *key, int *out) {
  size_t klen = strlen(key), i = 0;
  while (i + klen + 1 <= body->len) {
    if ((i == 0 || body->p[i - 1] == '&') &&
        memcmp(body->p + i, key, klen) == 0 && body->p[i + klen] == '=') {
      i += klen + 1;
      int v = 0, digits = 0;
      while (i < body->len && body->p[i] != '&' && digits < 3) {
        int d = decval(body->p[i]);
        if (d < 0) break;
        v = v * 10 + d;
        digits++;
        i++;
      }
      if (digits > 0) { *out = v; return 1; }
      return 0;
    }
    i++;
  }
  return 0;
}

static void http_pulse_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "POST")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }
  struct http_message *hm = (struct http_message *) ev_data;

  int pin = -1;
  if (!form_get_int(&hm->body, "pin", &pin) || !pin_known(pin)) {
    app_http_reply(c, 400, "{\"ok\":false,\"error\":\"unknown pin\"}");
    return;
  }
  pulse_pin(pin);
  char body[64];
  snprintf(body, sizeof(body), "{\"ok\":true,\"pin\":%d}", pin);
  app_http_reply(c, 200, body);
}

static void http_risky_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "POST")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }
  struct http_message *hm = (struct http_message *) ev_data;

  int on = 0;
  if (!form_get_int(&hm->body, "on", &on)) {
    app_http_reply(c, 400, "{\"ok\":false,\"error\":\"on required\"}");
    return;
  }
  set_risky(on != 0);
  char body[48];
  snprintf(body, sizeof(body), "{\"ok\":true,\"risky\":%d}", s_risky_enabled ? 1 : 0);
  app_http_reply(c, 200, body);
}

// ============================================================
// init
// ============================================================
enum mgos_app_init_result mgos_app_init(void) {
  build_test_pins();  // по умолчанию только безопасные пины

  for (int i = 0; i < SAFE_COUNT; i++) {
    pin_setup_output(s_safe_pins[i]);
    mgos_gpio_write(s_safe_pins[i], 0);  // покой — LOW
  }
  LOG(LL_INFO, ("RELAY TEST: безопасные пины %d %d %d %d %d %d на выходе, все LOW",
                s_safe_pins[0], s_safe_pins[1], s_safe_pins[2],
                s_safe_pins[3], s_safe_pins[4], s_safe_pins[5]));

  mgos_register_http_endpoint("/status", http_status_handler, NULL);
  mgos_register_http_endpoint("/start",  http_start_handler,  NULL);
  mgos_register_http_endpoint("/stop",   http_stop_handler,   NULL);
  mgos_register_http_endpoint("/pulse",  http_pulse_handler,  NULL);
  mgos_register_http_endpoint("/risky",  http_risky_handler,  NULL);

  LOG(LL_INFO, ("Ready: подключитесь к WiFi 'relay-test', откройте http://192.168.4.1"));
  return MGOS_APP_INIT_SUCCESS;
}
