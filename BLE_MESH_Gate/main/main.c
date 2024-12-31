
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



// QueueHandle_t ble_mesh_received_data_queue = NULL;
static uint8_t ticks = 0;
// model_sensor_data_t device_sensor_data;

// static void environment_task(void *arg) 
// {
   

    
   
//     while(1) {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);

       
//         if ((ticks++ >= 5) ) {
//         // if (ticks++ >= 5) {
//             // ESP_LOGI(TAG, "TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);
//             // device_sensor_data.eCO2 = eco2_baseline;
//             // device_sensor_data.tVOC = tvoc_baseline;
//             // strcpy(device_sensor_data.device_name, "GW");

//             // ble_mesh_custom_sensor_client_model_message_set(device_sensor_data);
//             //ble_mesh_custom_sensor_client_model_message_get();
//             model_sensor_data_t _received_data;
        
//         // if (xQueueReceive(ble_mesh_received_data_queue, &_received_data, 1000 / portTICK_PERIOD_MS) == pdPASS) {
//         //     ESP_LOGI(TAG, "Recebido dados de %s", _received_data.device_name);
//         //     ESP_LOGI(TAG, "    Temperatura: %f", _received_data.temperature);
//         //     ESP_LOGI(TAG, "    Pressao:     %f", _received_data.pressure);
//         //     ESP_LOGI(TAG, "    Umidade:     %f", _received_data.humidity);
//         //     ESP_LOGI(TAG, "    TVOC:        %d", _received_data.tVOC);
//         //     ESP_LOGI(TAG, "    eCO2:        %d", _received_data.eCO2);
//         //     ESP_LOGI(TAG, "    Ruido:       %d", _received_data.noise_level);
//         //     ESP_LOGI(TAG, "    Vermelho:    %f", _received_data.red);
//         //     ESP_LOGI(TAG, "    Laranja:     %f", _received_data.orange);
//         //     ESP_LOGI(TAG, "    Amarelo:     %f", _received_data.yellow);
//         //     ESP_LOGI(TAG, "    Verde:       %f", _received_data.green);
//         //     ESP_LOGI(TAG, "    Azul:        %f", _received_data.blue);
//         //     ESP_LOGI(TAG, "    Violeta:     %f", _received_data.violet);
//         // }
//             ticks = 0;
        
//     }
// }
// }
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
