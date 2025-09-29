//
// Created by qwqb233 on 2025/2/25.
//
#include "main.h"
#include "trailing.h"

uint8_t get_xPin_state(struct trailingClass * self ,TrailingPin X_pin)
{
    switch (X_pin)
    {
        case TrailingPin_X1:
            return self->x1_state;
        case TrailingPin_X2:
            return self->x2_state;
        case TrailingPin_X3:
            return self->x3_state;
        case TrailingPin_X4:
            return self->x4_state;
        default:
            return 0;
    }
}

void Trailing_scan(struct trailingClass * self)
{
    if (self->x1_port!= NULL)
    {
		self->x1_last_state = self->x1_state;
        self->x1_state = Read_Pin(self->x1_port, self->x1_pin);
    }
    if (self->x2_port!= NULL)
    {
		self->x2_last_state = self->x2_state;
        self->x2_state = Read_Pin(self->x2_port, self->x2_pin);
    }
    if (self->x3_port!= NULL)
    {
		self->x3_last_state = self->x3_state;
        self->x3_state = Read_Pin(self->x3_port, self->x3_pin);
    }
    if (self->x4_port!= NULL)
    {
		self->x4_last_state = self->x4_state;
        self->x4_state = Read_Pin(self->x4_port, self->x4_pin);
    }

}

void trailing_init(TrailingClass *trailing, GPIO_TypeDef* x1_port, uint16_t x1_pin,GPIO_TypeDef* x2_port, uint16_t x2_pin,GPIO_TypeDef* x3_port, uint16_t x3_pin,GPIO_TypeDef* x4_port, uint16_t x4_pin)
{
    trailing->x1_port = x1_port;
    trailing->x2_port = x2_port;
    trailing->x3_port = x3_port;
    trailing->x4_port = x4_port;
    trailing->x1_pin = x1_pin;
    trailing->x2_pin = x2_pin;
    trailing->x3_pin = x3_pin;
    trailing->x4_pin = x4_pin;
    trailing->x1_state = 0;
    trailing->x2_state = 0;
    trailing->x3_state = 0;
    trailing->x4_state = 0;
    trailing->x1_last_state = 0;
    trailing->x2_last_state = 0;
    trailing->x3_last_state = 0;
    trailing->x4_last_state = 0;
    trailing->get_xPin_state = get_xPin_state;
    trailing->Trailing_scan = Trailing_scan;
}
