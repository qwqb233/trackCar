#ifndef __OLED_H
#define __OLED_H

/*void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);*/


void OLED_WriteCommand(I2C_HandleTypeDef *,uint8_t);
void OLED_WriteData(I2C_HandleTypeDef *,uint8_t);
void OLED_SetCursor(I2C_HandleTypeDef *,uint8_t, uint8_t);
void OLED_Clear(I2C_HandleTypeDef *);
void OLED_ShowChar(I2C_HandleTypeDef *,uint8_t, uint8_t, char);
void OLED_ShowString(I2C_HandleTypeDef *,uint8_t, uint8_t, char *);
void OLED_ShowNum(I2C_HandleTypeDef *,uint8_t , uint8_t , uint32_t , uint8_t );
void OLED_ShowSignedNum(I2C_HandleTypeDef *,uint8_t , uint8_t , int32_t , uint8_t );
void OLED_ShowHexNum(I2C_HandleTypeDef *,uint8_t , uint8_t , uint32_t , uint8_t );
void OLED_ShowBinNum(I2C_HandleTypeDef *,uint8_t , uint8_t , uint32_t , uint8_t );

void OLED_Init(I2C_HandleTypeDef *);


#endif
