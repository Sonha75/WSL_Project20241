#include "dht.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_rom_sys.h"

static const char *TAG = "DHT";

#define DHT_DATA_BIT_COUNT 40
#define DHT_RESPONSE_TIMEOUT_US 200

static esp_err_t dht_send_start_signal(gpio_num_t gpio_pin) {
  gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(gpio_pin, 0);
  esp_rom_delay_us(18000); // 18ms
  gpio_set_level(gpio_pin, 1);
  esp_rom_delay_us(20); // 20-40us
  gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
  return ESP_OK;
}

static esp_err_t dht_wait_for_response(gpio_num_t gpio_pin) {
  int timeout = DHT_RESPONSE_TIMEOUT_US;
  while (gpio_get_level(gpio_pin) == 0) { // Low
    if (timeout-- <= 0) {
      ESP_LOGE(TAG, "DHT response timeout - low signal");
      return ESP_FAIL;
    }
    esp_rom_delay_us(1);
  }

  timeout = DHT_RESPONSE_TIMEOUT_US;
  while (gpio_get_level(gpio_pin) == 1) { // High
    if (timeout-- <= 0) {
      ESP_LOGE(TAG, "DHT response timeout - high signal");
      return ESP_FAIL;
    }
    esp_rom_delay_us(1);
  }

  return ESP_OK;
}

static esp_err_t dht_read_data(gpio_num_t gpio_pin, uint8_t *data) {
  for (int i = 0; i < DHT_DATA_BIT_COUNT; i++) {
    int timeout = DHT_RESPONSE_TIMEOUT_US;
    while (gpio_get_level(gpio_pin) == 0) {
      if (timeout-- <= 0) {
         ESP_LOGE(TAG, "DHT data bit read timeout - low signal");
        return ESP_FAIL;
      }
      esp_rom_delay_us(1);
    }

    esp_rom_delay_us(30);

    if (gpio_get_level(gpio_pin) == 1) {
      data[i / 8] |= (1 << (7 - (i % 8)));
    }

    timeout = DHT_RESPONSE_TIMEOUT_US;
    while (gpio_get_level(gpio_pin) == 1) {
        if(timeout-- <= 0) {
             ESP_LOGE(TAG, "DHT data bit read timeout - high signal");
            return ESP_FAIL;
        }
        esp_rom_delay_us(1);
    }
  }
  return ESP_OK;
}

esp_err_t dht_read(gpio_num_t gpio_pin, dht_reading_t *reading) {
    uint8_t data[5] = {0};

    if (dht_send_start_signal(gpio_pin) != ESP_OK) {
      return ESP_FAIL;
    }

    if (dht_wait_for_response(gpio_pin) != ESP_OK) {
      return ESP_FAIL;
    }

    if (dht_read_data(gpio_pin, data) != ESP_OK) {
        return ESP_FAIL;
    }


  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    ESP_LOGE(TAG, "DHT checksum error");
    return ESP_FAIL;
  }

  reading->humidity = (float)data[0];
  reading->temperature = (float)data[2];

  return ESP_OK;
}
