#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/gpio.h"

#if defined(CONFIG_BLE_MESH_ESP_WROOM_32)
#define LED_R GPIO_NUM_25
#define LED_G GPIO_NUM_2
#define LED_B GPIO_NUM_27
#elif defined(CONFIG_BLE_MESH_ESP_WROVER)
#define LED_R GPIO_NUM_0
#define LED_G GPIO_NUM_2
#define LED_B GPIO_NUM_4
#endif

// New LED definitions
#define LED_1 GPIO_NUM_5
#define LED_2 GPIO_NUM_18
#define LED_3 GPIO_NUM_19

#define LED_ON  1
#define LED_OFF 0

struct _led_state {
    uint8_t current;
    uint8_t previous;
    uint8_t pin;
    char *name;
};

void board_led_operation(uint8_t pin, uint8_t onoff);
void board_led_init(void); // Add this
void board_init(void);

#endif
