//
// Created by qwqb233 on 2025/2/24.
//
#include "main.h"
#include "button.h"

/*
 * @brief 按键扫描函数，扫描按键状态并调用回调函数
 * @param button 按键对象指针
 */
void button_scan(ButtonClass *button)
{
    button->button_level = Read_Pin(button->button_port, button->button_pin);
    if (button->button_level != button->last_button_level)
    {
        button->last_button_level = button->button_level;
        if (button->button_level == GPIO_PIN_RESET)
        {
            button->button_state = Button_Released;
        }
        else
        {
            button->button_state = Button_Pressed;
			if(button->last_button_state == Button_Pressed || button->last_button_state == Button_Holdin)
			{
				button->button_state = Button_Holdin;
			}
        }
        if (button->button_state != button->last_button_state)
        {
            button->last_button_state = button->button_state;
            if (button->button_state == Button_Pressed) {
                button->button_hold_time = 0;
                if (button->button_callback != NULL) {
                    button->button_callback(button);
                }
            }
        }
    }
    else {
        if (button->button_state == Button_Pressed || button->button_state == Button_Holdin) {
            button->button_hold_time++;
			button->button_state = Button_Holdin;
            if (button->button_hold_time >= button->button_hold_time_set) {
                button->button_state = Button_Holding;
                if (button->button_hold_callback != NULL) {
                    button->button_hold_callback(button);
                }
                button->button_hold_time = 0;
            }
        }
    }
}

/*
 * @brief 获取按键电平函数
 * @param button 按键对象指针
 * @return 按键电平
 */
uint16_t get_button_level(ButtonClass *button)
{
    return button->button_level;
}

/*
 * @brief 设置按键电平函数
 * @param button 按键对象指针
 * @param level 按键电平
 */
void set_button_level(ButtonClass *button, uint16_t level)
{
    button->button_level = level;
}

/*
 * @brief 设置按键持续时间函数
 * @param button 按键对象指针
 * @param time 按键持续时间
 */
void set_button_hold_time(ButtonClass *button, uint32_t time)
{
    button->button_hold_time_set = time;
}

/*
 * @brief 获取按键状态函数
 * @param button 按键对象指针
 * @return 按键状态
 */
ButtonState get_button_state(ButtonClass *button)
{
    return button->button_state;
}

/*
 * @brief 获取上一次按键状态函数
 * @param button 按键对象指针
 * @return 上一次按键状态
 */
ButtonState get_last_button_state(ButtonClass *button)
{
    return button->last_button_state;
}

/*
 * @brief 设置按键状态函数
 * @param button 按键对象指针
 * @param state 按键状态
 */
void set_button_state(ButtonClass *button, ButtonState state)
{
    button->button_state = state;
}

/*
 * @brief 设置按键回调函数
 * @param button 按键对象指针
 * @param callback 按键回调函数
 */
void set_button_callback(ButtonClass *button, void (*callback)(struct buttonClass * button))
{
    button->button_callback = callback;
}

/*
 * @brief 设置按键持续时间回调函数
 * @param button 按键对象指针
 * @param callback 按键持续时间回调函数
 */
void set_button_hold_callback(ButtonClass *button, void (*callback)(struct buttonClass * button))
{
    button->button_hold_callback = callback;
}

void button_init(ButtonClass *button, GPIO_TypeDef *button_port, uint16_t button_pin,uint16_t button_hold_time_set)
{
    button->button_port = button_port;
    button->button_pin = button_pin;
    button->button_state = Button_Released;
    button->last_button_state = Button_Released;
    button->last_button_level = GPIO_PIN_RESET;
    button->button_level = GPIO_PIN_RESET;
    button->button_hold_time = 1000;
    button->button_hold_time_set = button_hold_time_set;
    button->button_callback = NULL;
    button->button_hold_callback = NULL;
    button->button_scan = button_scan;
    button->get_button_level = get_button_level;
    button->set_button_level = set_button_level;
    button->set_button_hold_time = set_button_hold_time;
    button->get_button_state = get_button_state;
    button->get_last_button_state = get_last_button_state;
    button->set_button_state = set_button_state;
    button->set_button_callback = set_button_callback;
    button->set_button_hold_callback = set_button_hold_callback;
}

