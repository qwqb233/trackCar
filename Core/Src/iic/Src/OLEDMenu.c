//
// Created by qwqb233 on 2025/2/26.
//
#include "main.h"
#include "OLED.h"
#include "OLEDMenu.h"
#include <stdio.h>


void OLED_ShowWelcomeMenu(I2C_HandleTypeDef *I2Cx) {
    static uint8_t welcome_state = 0;
    if(welcome_state == 0) {
        OLED_Clear(I2Cx);
        welcome_state = 1;
    }
    OLED_ShowString(I2Cx,1, 1, "Smart Car");
    OLED_ShowString(I2Cx,2, 1, "04 46 49 31");
    OLED_ShowString(I2Cx,3, 1, "2025/2/26");
    OLED_ShowString(I2Cx,4, 1, "Block: 1");
}

void OLED_ShowRunMenu(I2C_HandleTypeDef *I2Cx,float speed_x,float speed_y, float speed_w, uint32_t distance, uint16_t time,uint8_t me)
{
    static uint8_t run_state = 0;
    if(run_state == 0) {
        OLED_Clear(I2Cx);
        run_state = 1;
    }
    char  speed_str[100];
    char  distance_str[100];
    char  time_str[100];
    sprintf(speed_str, "%.1f %.1f %.1f", speed_x, speed_y, speed_w);
    sprintf(distance_str, "dist:%d m", distance);
    sprintf(time_str, "time:%d:%02d:%02d", time/3600, (time%3600)/60, time%60);
    OLED_ShowString(I2Cx,1, 1, speed_str);
    OLED_ShowString(I2Cx,2, 1, distance_str);
    OLED_ShowString(I2Cx,3, 1, time_str);
	OLED_ShowNum(I2Cx,4,1,me,3);
}

