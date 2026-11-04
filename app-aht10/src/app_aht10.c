#include "app_aht10.h"

#include "mgos.h"
#include "mgos_i2c.h"

#include <math.h>
#include <stdlib.h>

// ============================================================
// Aosong AHT10/AHT20 по I2C
// Подключение: VDD 3.3 V, GND, SDA -> GPIO4, SCL -> GPIO5,
// подтяжка SDA/SCL к 3.3 V (обычно уже на модуле).
// ============================================================

#define AHT10_CMD_INIT  0xE1   // включить калибровку
#define AHT10_CMD_TRIG  0xAC   // запуск измерения
#define AHT10_CMD_RESET 0xBA   // программный сброс

#define AHT10_STATUS_BUSY       0x80
#define AHT10_STATUS_CALIBRATED 0x08

#define AHT10_POWUP_MS   40
#define AHT10_MEAS_TRIES 50    // таймаут ожидания готовности, 10 мс * 50 = 500 мс

struct app_aht10 {
  struct mgos_i2c *i2c;
  uint8_t addr;
};

static bool aht10_write(struct app_aht10 *dev, const uint8_t *data, size_t len) {
  return mgos_i2c_write(dev->i2c, dev->addr, data, len, true);
}

static bool aht10_read(struct app_aht10 *dev, uint8_t *data, size_t len) {
  return mgos_i2c_read(dev->i2c, dev->addr, data, len, true);
}

struct app_aht10 *app_aht10_create(uint8_t addr) {
  struct mgos_i2c *i2c = mgos_i2c_get_global();
  if (i2c == NULL) {
    LOG(LL_ERROR, ("aht10: I2C bus is not enabled"));
    return NULL;
  }

  struct app_aht10 *dev = (struct app_aht10 *) calloc(1, sizeof(*dev));
  if (dev == NULL) return NULL;
  dev->i2c = i2c;
  dev->addr = addr;

  mgos_msleep(AHT10_POWUP_MS);  // время на стабилизацию после подачи питания

  uint8_t status = 0;
  for (int attempt = 0; attempt < 3; attempt++) {
    uint8_t cmd[3] = {AHT10_CMD_INIT, 0x08, 0x00};
    if (!aht10_write(dev, cmd, sizeof(cmd))) {
      LOG(LL_ERROR, ("aht10: no ACK at 0x%02X (check wiring, SDA=4, SCL=5)", addr));
      free(dev);
      return NULL;
    }
    mgos_msleep(10);
    if (aht10_read(dev, &status, 1) && (status & AHT10_STATUS_CALIBRATED)) {
      LOG(LL_INFO, ("aht10: ready at 0x%02X (status 0x%02X)", addr, status));
      return dev;
    }
  }

  LOG(LL_ERROR, ("aht10: calibration failed (status 0x%02X)", status));
  free(dev);
  return NULL;
}

bool app_aht10_read(struct app_aht10 *dev, double *temp, double *hum) {
  if (temp != NULL) *temp = NAN;
  if (hum != NULL) *hum = NAN;
  if (dev == NULL) return false;

  uint8_t trig[3] = {AHT10_CMD_TRIG, 0x33, 0x00};
  if (!aht10_write(dev, trig, sizeof(trig))) {
    LOG(LL_WARN, ("aht10: trigger failed (no ACK at 0x%02X)", dev->addr));
    return false;
  }

  uint8_t status = AHT10_STATUS_BUSY;
  for (int i = 0; i < AHT10_MEAS_TRIES && (status & AHT10_STATUS_BUSY); i++) {
    mgos_msleep(10);
    if (!aht10_read(dev, &status, 1)) return false;
  }
  if (status & AHT10_STATUS_BUSY) {
    LOG(LL_WARN, ("aht10: measurement timeout"));
    return false;
  }

  uint8_t data[6];
  if (!aht10_read(dev, data, sizeof(data))) return false;

  uint32_t raw_h = ((uint32_t) data[1] << 12) | ((uint32_t) data[2] << 4) | ((uint32_t) data[3] >> 4);
  uint32_t raw_t = (((uint32_t) data[3] & 0x0F) << 16) | ((uint32_t) data[4] << 8) | (uint32_t) data[5];

  if (hum != NULL) *hum = (double) raw_h * 100.0 / 1048576.0;           // 2^20
  if (temp != NULL) *temp = (double) raw_t * 200.0 / 1048576.0 - 50.0;
  return true;
}

void app_aht10_close(struct app_aht10 *dev) {
  free(dev);
}
