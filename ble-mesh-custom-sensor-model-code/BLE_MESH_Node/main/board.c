

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"

#include "iot_button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mesh_server.h"
#define TAG "BOARD"

#define BUTTON_IO_NUM           0
#define BUTTON_ACTIVE_LEVEL     0


// extern model_sensor_data_t _server_model_state;
// thon tin node gui di
 extern model_sensor_data_t _server_model_state;
// struct _led_state led_state= {

//     { LED_OFF, LED_OFF, LED_G, "green" }

// };

void board_led_operation(uint8_t pin, uint8_t onoff)
{


    //     if (onoff == led_state.previous) {
    //         ESP_LOGW(TAG, "led %s is already %s",
    //                  led_state.name, (onoff ? "on" : "off"));
    //         return;
    //     }
    //     gpio_set_level(pin, onoff);
    //     led_state.previous = onoff;
    //     return;


    // ESP_LOGE(TAG, "LED is not found!");
     gpio_set_level(pin, onoff);
     vTaskDelay(1000 / portTICK_PERIOD_MS);
     onoff=0;
     gpio_set_level(pin, onoff);

}

// static void board_led_init(void)
// {

//         gpio_reset_pin(led_state.pin);
//         gpio_set_direction(led_state.pin, GPIO_MODE_OUTPUT);
//         gpio_set_level(led_state.pin, LED_OFF);
//         led_state.previous = LED_OFF;

// }

// extern void example_ble_mesh_send_status(void);

static void button_tap_cb(void* arg)
{
    ESP_LOGI(TAG, "tap cb (%s)", (char *)arg);

    // example_ble_mesh_send_status();
    set_information();
     server_send_to_unicast(_server_model_state);
}

static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
    }
}

void board_init(void)
{
    //board_led_init();
    board_button_init();
}
