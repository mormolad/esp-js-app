#pragma once
#include <stdbool.h>
#include <stdint.h>

struct app_aht10;

// Создать драйвер на глобальной I2C-шине (addr обычно 0x38).
// Выполняет инициализацию/калибровку. NULL — датчик не ответил.
struct app_aht10 *app_aht10_create(uint8_t addr);

// Измерить: *temp — °C, *hum — %RH. При ошибке вернёт false, значения = NAN.
bool app_aht10_read(struct app_aht10 *dev, double *temp, double *hum);

void app_aht10_close(struct app_aht10 *dev);
