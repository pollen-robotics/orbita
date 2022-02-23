#include "utils.h"
#include "gpio.h"

static uint8_t status_led_state = 1;


void toggle_status_led()
{
    status_led(1 - status_led_state);
}

void status_led(uint8_t state)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (state == 0));
    status_led_state = state;
}