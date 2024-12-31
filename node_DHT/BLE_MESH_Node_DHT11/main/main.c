#include <sdkconfig.h>
#include "nvs_flash.h"
#include <stdio.h>
#include "esp_log.h"
#include "mesh_server.h"
#include "mesh_device_app.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht11.h"
#include "freertos/semphr.h"



#define DHT_IO 26
static const char* TAG = "MESH-SERVER-MAIN";
TaskHandle_t dht_task_handle = NULL;
SemaphoreHandle_t dataReadySemaphore;


static uint8_t ticks = 0;
struct dht11_reading dht11_lastdata,dht11_currentdata;


model_sensor_data_t _server_model_state= {
    
    .device_name = "dht11",
    .index=3,
    .temperature=3,
    
   
    .humidity=3,
    
    .light=3,
    .smoke=3,
    .temp_ADC=3
};


static void environment_sensor(void *arg) {
    while(1) {
//vTaskDelay(2000);

//read DHT11
    dht11_currentdata = DHT11_read();
     while(dht11_currentdata.status != DHT11_OK){
            dht11_currentdata = DHT11_read();
        }
    dht11_lastdata=dht11_currentdata;

            xSemaphoreGive(dataReadySemaphore);
    

     vTaskDelay(2000 / portTICK_PERIOD_MS);
    

       if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE) {
        
       
              
       
           _server_model_state.temperature=dht11_lastdata.temperature;
           _server_model_state.humidity=dht11_lastdata.humidity;
           






           ESP_LOGE(TAG,"%f", _server_model_state.temperature);
        ESP_LOGE(TAG,"%f", _server_model_state.humidity);
           
            //strcpy(_server_model_state.device_name, "Node 3");

            
            server_send_to_unicast(_server_model_state);
           // dht11_currentdata.status == DHT11_OK;
           vTaskDelay(8000 / portTICK_PERIOD_MS);
       }
        
   }
}
void app_main(void) {
    dataReadySemaphore = xSemaphoreCreateBinary();
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

     board_init();
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    DHT11_init(DHT_IO);

    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err 0x%06x)", err);
    }
   
    
    
   
    xTaskCreate(environment_sensor, "environment_sensor", 1024 * 2, (void *)0, 1, NULL);
    
}