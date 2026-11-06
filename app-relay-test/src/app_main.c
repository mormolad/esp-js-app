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
// Безопасные пины ESP8266: 4, 5, 12, 13, 14, 16.
// НЕ трогаем: 0, 2, 15 (стрэппинг загрузки — реле может помешать старту),
// 1, 3 (консоль UART), 6-11 (подключены к флеш).
//
// Логика: все пины в покое LOW, тестируемый — импульс HIGH 1.5 с.
// Для обоих типов модулей реле это один щелчок: active-low реле
// выключено на LOW и включается на HIGH, active-high — наоборот.
// ============================================================
static const int s_pins[] = {4, 5, 12, 13, 14, 16};
#define PIN_COUNT ((int) (sizeof(s_pins) / sizeof(s_pins[0])))

#define STEP_HIGH_MS 1500
#define PULSE_MS 800

static bool s_running = false;
static int s_cur_idx = -1;   // индекс текущего «высокого» пина, -1 = все LOW
static int s_cycle = 0;

static bool pin_allowed(int pin) {
  for (int i = 0; i < PIN_COUNT; i++) {
    if (s_pins[i] == pin) return true;
  }
  return false;
}

static void all_pins_low(void) {
  for (int i = 0; i < PIN_COUNT; i++) mgos_gpio_write(s_pins[i], 0);
}

// Шаг авто-сканирования: текущий пин в LOW, следующий — в HIGH
static void scan_step_cb(void *arg) {
  (void) arg;
  if (!s_running) return;

  if (s_cur_idx >= 0) mgos_gpio_write(s_pins[s_cur_idx], 0);

  s_cur_idx++;
  if (s_cur_idx >= PIN_COUNT) {
    s_cur_idx = 0;
    s_cycle++;
  }
  int pin = s_pins[s_cur_idx];
  mgos_gpio_write(pin, 1);
  LOG(LL_INFO, ("RELAY TEST: GPIO %d HIGH (cycle %d) — слушаем, какое реле щёлкнуло", pin, s_cycle));

  mgos_set_timer(STEP_HIGH_MS, false, scan_step_cb, NULL);
}

static void scan_start(void) {
  if (s_running) return;
  s_running = true;
  s_cur_idx = -1;
  s_cycle++;
  LOG(LL_INFO, ("RELAY TEST: сканирование запущено"));
  mgos_set_timer(300, false, scan_step_cb, NULL);
}

static void scan_stop(void) {
  s_running = false;
  s_cur_idx = -1;
  all_pins_low();
  LOG(LL_INFO, ("RELAY TEST: сканирование остановлено, все пины LOW"));
}

static void pulse_low_cb(void *arg) {
  mgos_gpio_write((int) (intptr_t) arg, 0);
}

// Ручной импульс на один пин (работает и когда скан остановлен)
static void pulse_pin(int pin) {
  if (!pin_allowed(pin)) return;
  mgos_gpio_write(pin, 1);
  LOG(LL_INFO, ("RELAY TEST: ручной импульс GPIO %d на %d мс", pin, PULSE_MS));
  mgos_set_timer(PULSE_MS, false, pulse_low_cb, (void *) (intptr_t) pin);
}

// ============================================================
// HTTP
// ============================================================
static void http_status_handler(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  (void) ud;
  if (ev != MG_EV_HTTP_REQUEST) return;
  if (!app_http_method_is(ev_data, "GET")) { app_http_reply(c, 405, "{\"ok\":false}"); return; }

  int pin = (s_cur_idx >= 0) ? s_pins[s_cur_idx] : -1;
  char body[128];
  snprintf(body, sizeof(body), "{\"running\":%d,\"pin\":%d,\"cycle\":%d}",
           s_running ? 1 : 0, pin, s_cycle);
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

// Достать числовое поле из urlencoded-тела POST вида "pin=12&..."
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
  if (!form_get_int(&hm->body, "pin", &pin) || !pin_allowed(pin)) {
    app_http_reply(c, 400, "{\"ok\":false,\"error\":\"unknown pin\"}");
    return;
  }
  pulse_pin(pin);
  char body[64];
  snprintf(body, sizeof(body), "{\"ok\":true,\"pin\":%d}", pin);
  app_http_reply(c, 200, body);
}

// ============================================================
// init
// ============================================================
enum mgos_app_init_result mgos_app_init(void) {
  for (int i = 0; i < PIN_COUNT; i++) {
    mgos_gpio_set_mode(s_pins[i], MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_set_pull(s_pins[i], MGOS_GPIO_PULL_NONE);
    mgos_gpio_write(s_pins[i], 0);  // покой — LOW
  }
  LOG(LL_INFO, ("RELAY TEST: пины %d %d %d %d %d %d настроены на выход, все LOW",
                s_pins[0], s_pins[1], s_pins[2], s_pins[3], s_pins[4], s_pins[5]));

  mgos_register_http_endpoint("/status", http_status_handler, NULL);
  mgos_register_http_endpoint("/start",  http_start_handler,  NULL);
  mgos_register_http_endpoint("/stop",   http_stop_handler,   NULL);
  mgos_register_http_endpoint("/pulse",  http_pulse_handler,  NULL);

  LOG(LL_INFO, ("Ready: подключитесь к WiFi 'relay-test', откройте http://192.168.4.1"));
  return MGOS_APP_INIT_SUCCESS;
}
