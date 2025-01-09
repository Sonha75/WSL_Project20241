#ifndef DHT_H
#define DHT_H

#include <stdint.h>
#include "driver/gpio.h"

typedef struct {
    float temperature;
    float humidity;
} dht_reading_t;

esp_err_t dht_read(gpio_num_t gpio_pin, dht_reading_t *reading);

#endif
