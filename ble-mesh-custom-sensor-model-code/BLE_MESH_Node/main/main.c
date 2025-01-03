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
#include "bh1750.h"
#include "OLEDDisplay.h"
#include "OLEDDisplayFonts.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/adc.h"




#define I2C_MASTER_SCL_IO   22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   21               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM      I2C_NUM_0 /*!< I2C port number for master dev */
#define DHT_IO 26
static const char* TAG = "MESH-SERVER-MAIN";
TaskHandle_t dht_task_handle = NULL;
SemaphoreHandle_t dataReadySemaphore;

static uint16_t level_BH1750;
static ESP32_BH1750 *bh1750_handle; 
static uint8_t ticks = 0;
struct dht11_reading dht11_lastdata,dht11_currentdata;
static int adc_Value_MQ2 = 0;

static void oled_task(void *arg);
void init_BH1750(void);
void init_MQ2(void);

model_sensor_data_t _server_model_state= {
    
    .device_name = "esp_3",
    .index=3,
    .temperature=0,
    
   
    .humidity=0,
    
    .light=0,
    .smoke=0,
    .temp_ADC=0
};

void init_MQ2(void){
    

   adc1_config_width(ADC_BITWIDTH_12);
   adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
}
static void oled_task(void *arg)
{
	OLEDDisplay_t *oled = OLEDDisplay_init(I2C_MASTER_NUM,0x78,I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO);

    char str[30];
	
    OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_CENTER);
    OLEDDisplay_setFont(oled,ArialMT_Plain_16);
    OLEDDisplay_drawString(oled,64, 0, "illuminance");
	OLEDDisplay_display(oled);
	vTaskDelay(500 / portTICK_PERIOD_MS);


		while(1) {
            OLEDDisplay_clear(oled);
            OLEDDisplay_setFont(oled,ArialMT_Plain_16);
            OLEDDisplay_drawString(oled,64, 0, "illuminance");
            OLEDDisplay_setFont(oled,ArialMT_Plain_24);
            sprintf(str, "%d", level_BH1750);
			OLEDDisplay_drawString(oled, 64, 30, str);
			OLEDDisplay_display(oled);
			vTaskDelay(250 / portTICK_PERIOD_MS);
		}

}
void init_BH1750(void){
    int i2c_master_port = I2C_NUM_0;
ESP32_BH1750 *bh1750_handle = (ESP32_BH1750*)calloc(1, sizeof(ESP32_BH1750));
    
    

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 700000,
        .clk_flags = 0
    };

    i2c_param_config(i2c_master_port, &conf);

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "Install OK.........");
    BH1750_init(bh1750_handle, 0x23, I2C_NUM_0);
    ESP_LOGI(TAG, "INIT OK.........");
    BH1750_power_on(bh1750_handle);
    BH1750_set(bh1750_handle, BH1750_MODE_ONE_TIME, BH1750_RES_HIGH, 69);

    xTaskCreate(oled_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}


static void environment_sensor(void *arg) {
    while(1) {
//vTaskDelay(2000);

//read DHT11
    dht11_currentdata = DHT11_read();
     while(dht11_currentdata.status != DHT11_OK){
            dht11_currentdata = DHT11_read();
        }
    dht11_lastdata=dht11_currentdata;


//read BH1750
    BH1750_measure_and_read(bh1750_handle, &level_BH1750);

//read MQ2
    adc_Value_MQ2 = adc1_get_raw(ADC1_CHANNEL_5);

            xSemaphoreGive(dataReadySemaphore);
    

     vTaskDelay(2000 / portTICK_PERIOD_MS);
    

       if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE) {
        
       vTaskDelay(3000 / portTICK_PERIOD_MS);
              
       
           _server_model_state.temperature=dht11_lastdata.temperature;
           _server_model_state.humidity=dht11_lastdata.humidity;
           _server_model_state.light=level_BH1750;
           _server_model_state.smoke=adc_Value_MQ2;






           ESP_LOGE(TAG,"%f", _server_model_state.temperature);
            
           
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
    DHT11_init(DHT_IO);
    init_BH1750();
    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err 0x%06x)", err);
    }
   
    
    
   
    xTaskCreate(environment_sensor, "environment_sensor", 1024 * 2, (void *)0, 1, NULL);
    
}