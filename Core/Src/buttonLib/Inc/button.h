//
// Created by qwqb233 on 2025/2/24.
//

#ifndef CARTRACE_1_BUTTON_H
#define CARTRACE_1_BUTTON_H
#include "main.h"

typedef enum buttonState {
    Button_Pressed,
    Button_Released,
    Button_Holding
}ButtonState;

typedef struct buttonClass
{
    GPIO_TypeDef* button_port;
    uint16_t button_pin;
    ButtonState button_state;
    ButtonState last_button_state;
    uint8_t last_button_level;
    uint8_t button_level;
    uint32_t button_hold_time;
    uint32_t button_hold_time_set;

    //回调函数,需要用户自己实现并赋值
    void (*button_callback)(struct buttonClass * button);
    //长按回调函数,需要用户自己实现并赋值
    void (*button_hold_callback)(struct buttonClass * button);

    /*
    * @brief 按键扫描函数，扫描按键状态并调用回调函数
    * @param button 按键对象指针
    */
    void (*button_scan)(struct buttonClass * button);

    uint16_t (*get_button_level)(struct buttonClass * button);
    void (*set_button_level)(struct buttonClass * button, uint16_t level);
    void (*set_button_hold_time)(struct buttonClass * button, uint32_t time);
    ButtonState (*get_button_state)(struct buttonClass * button);
    ButtonState (*get_last_button_state)(struct buttonClass * button);
    void (*set_button_state)(struct buttonClass * button, ButtonState state);
    void (*set_button_callback)(struct buttonClass * button, void (*callback)(struct buttonClass * button));
    void (*set_button_hold_callback)(struct buttonClass * button, void (*callback)(struct buttonClass * button));

}ButtonClass;

void button_init(ButtonClass *button, GPIO_TypeDef *button_port, uint16_t button_pin,uint16_t button_hold_time_set);

#endif //CARTRACE_1_BUTTON_H
