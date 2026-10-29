#include "mgos_http_server.h"

#include <string.h>

#include "mongoose.h"

int app_http_method_is(void *ev_data, const char *method) {
  struct http_message *hm = (struct http_message *) ev_data;
  return mg_vcmp(&hm->method, method) == 0 ? 1 : 0;
}

void app_http_reply(struct mg_connection *c, int code, const char *body) {
  int len = body == NULL ? 0 : (int) strlen(body);
  mg_send_head(c, code, len,
               "Content-Type: application/json\r\nConnection: close");
  if (len > 0) mg_send(c, body, len);
  c->flags |= MG_F_SEND_AND_CLOSE;
}
