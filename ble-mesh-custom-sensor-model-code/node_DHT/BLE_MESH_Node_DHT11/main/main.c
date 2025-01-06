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
#include "string.h"

#define DHT_IO 4
#define MSG_ID_MAX 100
static const char *TAG = "MESH-SERVER-MAIN";

TaskHandle_t dht_task_handle = NULL;

static uint8_t ticks = 0;
struct dht11_reading dht11_lastdata, dht11_currentdata;
union_t dht11_message;
static uint32_t msg_counter = 0;

static uint32_t calculate_checksum(union_t *msg);

model_sensor_data_t _server_model_state = {
    .msg_id = 0,
    .device_name = "dht1",
    .temperature = 0.0f,
    .humidity = 0.0f,
    .index = 1,
    .status = false,
    .csum = 0};

static void environment_sensor(void *arg)
{
    while (1)
    {

        // read DHT11
        dht11_lastdata.temperature = 0;
        dht11_lastdata.humidity = 0;
        dht11_currentdata = DHT11_read();
        while (dht11_currentdata.status != DHT11_OK)
        {
            dht11_currentdata = DHT11_read();
        }
        dht11_lastdata = dht11_currentdata;
        msg_counter = msg_counter % MSG_ID_MAX;
        _server_model_state.msg_id = ++msg_counter;
        _server_model_state.temperature = dht11_lastdata.temperature;
        _server_model_state.humidity = dht11_lastdata.humidity;
        _server_model_state.status = true;
        memcpy(&dht11_message.message, &_server_model_state, sizeof(model_sensor_data_t));

        dht11_message.message.csum = calculate_checksum(&dht11_message);
        
        // memcpy(&_server_model_state, &dht11_message.message, sizeof(model_sensor_data_t));

        ESP_LOGI(TAG, "Sending message ID: %lu, Temp: %.1f, Humidity: %.1f, Checksum: %u",
                 _server_model_state.msg_id,
                 _server_model_state.temperature,
                 _server_model_state.humidity,
                 dht11_message.message.csum
                 );

        server_send_to_unicast(dht11_message.message);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
void app_main(void)
{

    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    board_init();
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    DHT11_init(DHT_IO);

    err = ble_mesh_device_init();
    if (err)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err 0x%06x)", err);
    }

    xTaskCreate(environment_sensor, "environment_sensor", 1024 * 2, (void *)0, 1, NULL);
}

static uint32_t calculate_checksum(union_t *msg)
{
    uint32_t sum = 0;
    for (int i = 0; i < sizeof(union_t) - 4; i++)
    {
        sum += msg->data[i];
         ESP_LOGE(TAG, "data %02X ",  msg->data[i]);
    }
    return sum;
}