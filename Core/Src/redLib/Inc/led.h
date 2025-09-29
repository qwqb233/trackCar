//
// Created by qwqb233 on 2025/2/25.
//

#ifndef CARTRACE_1_LED_H
#define CARTRACE_1_LED_H

#include "main.h"

typedef struct ledClass
{
    GPIO_TypeDef* led_port;
    uint16_t led_pin;
    uint8_t led_state;

    void (*led_on)(struct ledClass* led);
    void (*led_off)(struct ledClass* led);
    void (*led_toggle)(struct ledClass* led);
}LedClass;

void led_on(LedClass* self)
{
    Set_Pin(self->led_port, self->led_pin);
    self->led_state = 1;
}
void led_off(LedClass* self)
{
    Reset_Pin(self->led_port, self->led_pin);
    self->led_state = 0;
}
void led_toggle(LedClass* self)
{
    if(self->led_state)
    {
        led_off(self);
    }
    else
    {
        led_on(self);
    }
}

void led_init(LedClass* LED, GPIO_TypeDef* led_port, uint16_t led_pin)
{
    LED->led_port = led_port;
    LED->led_pin = led_pin;
    LED->led_state = 0;
    LED->led_on = led_on;
    LED->led_off = led_off;
    LED->led_toggle = led_toggle;
}


#endif //CARTRACE_1_LED_H
