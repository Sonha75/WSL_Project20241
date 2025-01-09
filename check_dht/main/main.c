#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"
#include "driver/gpio.h"

static const char *TAG = "main";
#define DHT11_PIN GPIO_NUM_4

void app_main(void) {
  esp_err_t err;
  dht_reading_t reading;

  while (1) {
    err = dht_read(DHT11_PIN, &reading);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Humidity: %.1f%%, Temperature: %.1f°C", reading.humidity, reading.temperature);
    } else {
      ESP_LOGE(TAG, "Failed to read DHT11 sensor");
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2 seconds
  }
}
