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

// #include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/adc.h"




// #define I2C_MASTER_SCL_IO   22               /*!< gpio number for I2C master clock */
// #define I2C_MASTER_SDA_IO   21               /*!< gpio number for I2C master data  */
// #define I2C_MASTER_NUM      I2C_NUM_0 /*!< I2C port number for master dev */

static const char* TAG = "MESH-SERVER-MAIN";
TaskHandle_t dht_task_handle = NULL;
SemaphoreHandle_t dataReadySemaphore;


static uint8_t ticks = 0;

static int adc_Value_MQ2 = 0;


void init_MQ2(void);

model_sensor_data_t _server_model_state= {
    
    .device_name = "MQ2",
    .index=4,
    .temperature=4,
    
   
    .humidity=4,
    
    .light=4,
    .smoke=4,
    .temp_ADC=4
};

void init_MQ2(void){
    

   adc1_config_width(ADC_BITWIDTH_12);
   adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
}



static void environment_sensor(void *arg) {
    while(1) {
//vTaskDelay(2000);


//read MQ2
    adc_Value_MQ2 = adc1_get_raw(ADC1_CHANNEL_5);

            xSemaphoreGive(dataReadySemaphore);
    

     vTaskDelay(2000 / portTICK_PERIOD_MS);
    

       if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE) {
        
       vTaskDelay(8000 / portTICK_PERIOD_MS);
              
       
        
        
           _server_model_state.smoke=adc_Value_MQ2;






        ESP_LOGE(TAG,"%lf", _server_model_state.smoke);
            
           
            //strcpy(_server_model_state.device_name, "Node 3");

            
            server_send_to_unicast(_server_model_state);
           // dht11_currentdata.status == DHT11_OK;
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
    //DHT11_init(DHT_IO);
    
    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err 0x%06x)", err);
    }
   
    
    
   
    xTaskCreate(environment_sensor, "environment_sensor", 1024 * 2, (void *)0, 1, NULL);
    
}