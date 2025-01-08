#include <sdkconfig.h>
#include "nvs_flash.h"
#include <stdio.h>
#include "esp_log.h"
#include "mesh_server.h"
#include "mesh_device_app.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos/semphr.h"


#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/adc.h"


static const char* TAG = "MESH-SERVER-MAIN";
TaskHandle_t dht_task_handle = NULL;
SemaphoreHandle_t dataReadySemaphore;
uint16_t getRawLm35Avr(adc1_channel_t lm35Channel);
static float LM35_Temp = 0;
float getTemp(adc1_channel_t lm35Channel);


static uint8_t ticks = 0;



model_sensor_data_t _server_model_state= {
    
    .device_name = "LM35",
    .index=2,
    .temperature=2,
    
   
    .humidity=2,
    
    .light=2,
    .smoke=2,
    .temp_ADC=2
};




static void environment_sensor(void *arg) {
    while(1) {
//vTaskDelay(2000);

    LM35_Temp = getTemp(ADC1_CHANNEL_7);
            xSemaphoreGive(dataReadySemaphore);
    

     vTaskDelay(2000 / portTICK_PERIOD_MS);
    

       if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE) {
        
       vTaskDelay(8000 / portTICK_PERIOD_MS);
              
       
           _server_model_state.temp_ADC=LM35_Temp;






           ESP_LOGE(TAG,"%f", _server_model_state.temp_ADC);
            
           
            //strcpy(_server_model_state.device_name, "Node 3");

            
            server_send_to_unicast(_server_model_state);
           // dht11_currentdata.status == DHT11_OK;
       }
        
   }
}


uint16_t getRawLm35Avr(adc1_channel_t lm35Channel)
{
    uint32_t Sum = 0;
    uint16_t Aver = 0;
    for(int i=0; i<1000; i++)
    {
        Sum += adc1_get_raw(lm35Channel);
    }

    Aver = Sum / 1000;

    return Aver;
}

float getTemp(adc1_channel_t lm35Channel)
{
    uint16_t RawLm35 = getRawLm35Avr(lm35Channel);
    return (RawLm35 * 0.0806 + 11.5);                  // Cái này chỉnh bừa =))
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
    
    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err 0x%06x)", err);
    }
   

   adc1_config_width(ADC_BITWIDTH_12);
   adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    
    
   
    xTaskCreate(environment_sensor, "environment_sensor", 1024 * 2, (void *)0, 1, NULL);
    
}