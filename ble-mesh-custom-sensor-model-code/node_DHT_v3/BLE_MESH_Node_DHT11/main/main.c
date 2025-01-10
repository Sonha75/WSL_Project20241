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

// LED state variables
static uint8_t led1_state = LED_OFF;
static uint8_t led2_state = LED_OFF;
static uint8_t led3_state = LED_OFF;

// Thresholds for LED1 (Temperature)
#define TEMP_LOWER_BOUND 20.0f
#define TEMP_UPPER_BOUND 35.0f

// Thresholds for LED2 (Humidity)
#define HUMIDITY_LOWER_BOUND 50.0f
#define HUMIDITY_UPPER_BOUND 80.0f

static uint32_t calculate_checksum(union_t *msg);

model_sensor_data_t _server_model_state = {
    .msg_id = 0,
    .device_name = "dht3",
    .temperature = 0.0f,
    .humidity = 0.0f,
    .index = 3,
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
         if(dht11_currentdata.status != DHT11_OK) {
            // LED3 blink when no data receive from DHT11
            led3_state = !led3_state;
            board_led_operation(LED_3, led3_state);
            ESP_LOGE(TAG, "DHT11 read error");
            _server_model_state.status = 0;
         } else {
            dht11_lastdata = dht11_currentdata;
            msg_counter = msg_counter % MSG_ID_MAX;
            _server_model_state.msg_id = ++msg_counter;
            _server_model_state.temperature = dht11_lastdata.temperature;
            _server_model_state.humidity = dht11_lastdata.humidity;
            _server_model_state.status = true;
            memcpy(&dht11_message.message, &_server_model_state, sizeof(model_sensor_data_t));

            dht11_message.message.csum = calculate_checksum(&dht11_message);

            // LED1 Control (Temperature)
            if(_server_model_state.temperature < TEMP_LOWER_BOUND) {
                led1_state = LED_ON; // ON if below lower bound
            } else if(_server_model_state.temperature > TEMP_UPPER_BOUND) {
                led1_state = LED_ON;  // ON if above upper bound
            } else {
                led1_state = LED_OFF; // OFF if within the range
            }
            board_led_operation(LED_1, led1_state);

            // LED2 Control (Humidity)
             if(_server_model_state.humidity < HUMIDITY_LOWER_BOUND) {
               led2_state = LED_ON; // ON if below lower bound
            } else if(_server_model_state.humidity > HUMIDITY_UPPER_BOUND) {
                led2_state = LED_ON; // ON if above upper bound
            } else {
                 led2_state = LED_OFF; // OFF if within the range
            }
            board_led_operation(LED_2, led2_state);


            ESP_LOGI(TAG, "Sending message ID: %lu, Temp: %.1f, Humidity: %.1f, Checksum: %u",
                    _server_model_state.msg_id,
                    _server_model_state.temperature,
                    _server_model_state.humidity,
                    dht11_message.message.csum
                );

            server_send_to_unicast(dht11_message.message);
             // reset error LED
            board_led_operation(LED_3, LED_OFF);
         }
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Blink LED and read DHT every 1 second
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
        //ESP_LOGE(TAG, "data %02X ",  msg->data[i]);
    }
    return sum;
}
