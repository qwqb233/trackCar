//
// Created by qwqb233 on 2025/2/26.
//

#ifndef F4TEST_OLEDMENU_H
#define F4TEST_OLEDMENU_H
void OLED_ShowWelcomeMenu(I2C_HandleTypeDef *I2Cx);
void OLED_ShowRunMenu(I2C_HandleTypeDef *I2Cx,float speed_x,float speed_y, float speed_w, uint32_t distance, uint16_t time, uint8_t me);
#endif //F4TEST_OLEDMENU_H
