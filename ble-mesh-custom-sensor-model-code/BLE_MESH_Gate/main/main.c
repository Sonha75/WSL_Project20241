
#include <sdkconfig.h>
#include "nvs_flash.h"

#include "esp_log.h"
#include "mesh_device_app.h"
#include "board.h"
#include "mqtt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_config.h"
#define TAG "MAIN"


static uint8_t ticks = 0;

void app_main(void) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

  

     board_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }


   //xTaskCreate(environment_task, "air_sensor_main_task", 1024 * 2, (void *)0, 15, NULL);

     wifi_init_station();
     mqtt_client_sta();
   
}
