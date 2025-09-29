//
// Created by qwqb233 on 2025/2/25.
//

#ifndef CARTRACE_1_TRAILING_H
#define CARTRACE_1_TRAILING_H


typedef enum TrailingPin
{
    TrailingPin_X1 = 0,
    TrailingPin_X2,
    TrailingPin_X3,
    TrailingPin_X4,
}TrailingPin;

typedef struct trailingClass
{
    GPIO_TypeDef* x1_port;
    GPIO_TypeDef* x2_port;
    GPIO_TypeDef* x3_port;
    GPIO_TypeDef* x4_port;
    uint16_t x1_pin;
    uint16_t x2_pin;
    uint16_t x3_pin;
    uint16_t x4_pin;

    uint8_t x1_state;
    uint8_t x2_state;
    uint8_t x3_state;
    uint8_t x4_state;

    uint8_t x1_last_state;
    uint8_t x2_last_state;
    uint8_t x3_last_state;
    uint8_t x4_last_state;

    //获取指定寻迹模块引脚的状态
    uint8_t (*get_xPin_state)( struct trailingClass * self,TrailingPin X_pin);
    //扫描寻迹模块引脚的状态
    void (*Trailing_scan)(struct trailingClass * self);
}TrailingClass;

void trailing_init(TrailingClass *trailing, GPIO_TypeDef* x1_port, uint16_t x1_pin,GPIO_TypeDef* x2_port, uint16_t x2_pin,GPIO_TypeDef* x3_port, uint16_t x3_pin,GPIO_TypeDef* x4_port, uint16_t x4_pin);
#endif //CARTRACE_1_TRAILING_H
