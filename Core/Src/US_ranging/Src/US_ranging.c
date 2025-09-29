//
// Created by qwqb233 on 2025/2/27.
// note: 引脚设定：外部中断模式为上升沿和下降沿触发
#include "main.h"
#include "project_exit.h"
#include "US_ranging.h"


/*
 * @brief  获取返回脉冲宽度并计算距离（需要先发送trigger信号）
 * @param  None
 * @retval 距离（单位：cm）
 * @note   该函数需要先发送trigger信号,函数中ranging_time来自project_exit.h,更改该文件中外部中断函数可以修改echo引脚
 *         如果一直阻塞，检查外部中断函数是否正确
 */
//uint16_t __get_distance_cm(void)
//{	
//	uint32_t time = get_time();
//    while(ranging_start_time == 0 || ranging_end_time == 0)
//	{
//		if(get_time() > time+1500)
//			break;
//	}
//    uint32_t time_diff = ranging_end_time - ranging_start_time;
//    uint16_t distance_cm = (uint16_t)(time_diff * 5.8);
//    ranging_start_time = 0;
//    ranging_end_time = 0;
//	if(ranging_start_time > ranging_end_time)
//		return 0;
//    return distance_cm;
//}

///*
// * @brief  获取超声波传感器距离
// * @param  trigger_port 触发引脚端口
// * @param  trigger_pin  触发引脚编号
// * @retval 距离（单位：cm）
// */
//uint16_t get_distance_cm(GPIO_TypeDef* trigger_port, uint16_t trigger_pin)
//{
//    Set_Pin(trigger_port, trigger_pin);
//    int i = 0;
//    while(i < 1000) i++;
//    Reset_Pin(trigger_port, trigger_pin);
//    return __get_distance_cm();
//}

void delay_us(uint32_t us)
{
	__IO uint32_t Delay = us * (SystemCoreClock / 8U / 1000000U);
	
	do
	{
		__NOP();
	}while(Delay--);
	
}


uint16_t get_distance_cm_nT(GPIO_TypeDef * trigger_port, uint16_t trigger_pin)
{
	Set_Pin(trigger_port,trigger_pin);
	delay_us(20);
	Reset_Pin(trigger_port,trigger_pin);
	uint32_t timeOut = 0;
	while(Read_Pin(GPIOC,ec_Pin) == 0)
	{
		timeOut++;
		delay_us(1);
		if(timeOut >= 1000)return 100;
	}
	uint32_t start_time = 0,end_time = 0;
	while(Read_Pin(GPIOC,ec_Pin) == 1)
	{
		end_time++;
		delay_us(1);
		if(end_time >= 1000)return 100;
		
	}
	uint32_t dist = (end_time - start_time) * 0.0343/2;
	return dist;
	
}
