//
// Created by qwqb233 on 2025/2/25.
//

#ifndef CARTRACE_1_RGB_H
#define CARTRACE_1_RGB_H

#include "main.h"

#define RED_LIGHT 3
#define GREEN_LIGHT 5
#define BLUE_LIGHT 6
#define YELLOW_LIGHT 1
#define CYAN_LIGHT 4
#define MAGENTA_LIGHT 2
#define WHITE_LIGHT 0

typedef struct rgbClass
{
    GPIO_TypeDef * R_port;
    GPIO_TypeDef * G_port;
    GPIO_TypeDef * B_port;
    uint16_t R_pin;
    uint16_t G_pin;
    uint16_t B_pin;
    uint8_t rgb_light;
    //设置RGB灯颜色
    void (*setRGB)(struct rgbClass *self);
}RGBClass;

void setRGB(RGBClass *self)
{
    uint8_t light = self->rgb_light;
	Write_Pin(self->R_port,self->R_pin,(light >> 2)&0x01);
	Write_Pin(self->G_port,self->G_pin,(light >> 1)&0x01);
	Write_Pin(self->B_port,self->B_pin,(light >> 0)&0x01);
}

void RGB_init(RGBClass *RGB, GPIO_TypeDef * R_port, uint16_t R_pin, GPIO_TypeDef * G_port, uint16_t G_pin, GPIO_TypeDef * B_port, uint16_t B_pin)
{
    RGB->R_port = R_port;
    RGB->G_port = G_port;
    RGB->B_port = B_port;
    RGB->R_pin = R_pin;
    RGB->G_pin = G_pin;
    RGB->B_pin = B_pin;
    RGB->rgb_light = 0;
    RGB->setRGB = setRGB;
}

#endif //CARTRACE_1_RGB_H
