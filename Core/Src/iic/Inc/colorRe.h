//
// Created by qwqb233 on 2025/2/25.
//

#ifndef CARTRACE_1_COLORRE_H
#define CARTRACE_1_COLORRE_H
#define u32 uint32_t
#define u16 uint16_t
#define u8 uint8_t

#define max3v(v1, v2, v3)   ((v1)<(v2)? ((v2)<(v3)?(v3):(v2)):((v1)<(v3)?(v3):(v1)))
#define min3v(v1, v2, v3)   ((v1)>(v2)? ((v2)>(v3)?(v3):(v2)):((v1)>(v3)?(v3):(v1)))

typedef struct{
    unsigned short  c;      //[0-65536]
    unsigned short  r;
    unsigned short  g;
    unsigned short  b;
}COLOR_RGBC;//RGBC

typedef struct{
    unsigned short h;       //[0,360]
    unsigned char  s;       //[0,100]
    unsigned char  l;       //[0,100]
}COLOR_HSL;//HSL

void TCS34725_Write(I2C_HandleTypeDef *I2Cx,u8 subAddr, u8* dataBuffer, u8 bytesNumber);
void TCS34725_Read(I2C_HandleTypeDef *I2Cx,u8 subAddr, u8* dataBuffer, u8 bytesNumber);
void TCS34725_SetIntegrationTime(I2C_HandleTypeDef *I2Cx,u8 time);
void TCS34725_SetGain(I2C_HandleTypeDef *I2Cx,u8 gain);
void TCS34725_Enable(I2C_HandleTypeDef *I2Cx);
void TCS34725_Disable(I2C_HandleTypeDef *I2Cx);
u8 TCS34725_Init(I2C_HandleTypeDef *I2Cx);
u16 TCS34725_GetChannelData(I2C_HandleTypeDef *I2Cx,u8 reg);
u8 TCS34725_GetRawData(I2C_HandleTypeDef *I2Cx,COLOR_RGBC *rgbc);
void RGBtoHSL(COLOR_RGBC *Rgb, COLOR_HSL *Hsl);
void TCS34725_Test(I2C_HandleTypeDef *I2Cx);


#endif //CARTRACE_1_COLORRE_H
