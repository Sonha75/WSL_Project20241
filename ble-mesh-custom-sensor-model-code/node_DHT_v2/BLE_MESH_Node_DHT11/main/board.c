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

// extern model_sensor_data_t _server_model_state; // Đã có trong main.c
extern model_sensor_data_t _server_model_state;


void board_led_operation(uint8_t pin, uint8_t onoff) {
    gpio_set_level(pin, onoff);
}


void board_led_init(void) {
    gpio_config_t io_conf;

    // Setup LEDs as output, pull down to prevent them being HIGH on startup
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_1) | (1ULL << LED_2) | (1ULL << LED_3);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    // Ensure LEDs start with state off
    board_led_operation(LED_1, LED_OFF);
    board_led_operation(LED_2, LED_OFF);
    board_led_operation(LED_3, LED_OFF);

}


static void button_tap_cb(void* arg)
{
    ESP_LOGI(TAG, "tap cb (%s)", (char *)arg);
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
    board_led_init(); // Initialize LEDs
    board_button_init();
}
