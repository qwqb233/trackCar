//
// Created by qwqb233 on 2025/2/27.
//
#include "main.h"
#include "project_exit.h"

uint32_t ranging_start_time = 0;
uint32_t ranging_end_time = 0;

//uint32_t time__ = 0;

//void set_time(void)
//{
//	time__ ++;
//}

//uint32_t get_time(void)
//{
//    return time__;
//}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    /* Prevent unused argument(s) compilation warning */
//    //UNUSED(GPIO_Pin);
//    /* NOTE: This function Should not be modified, when the callback is needed,
//             the HAL_GPIO_EXTI_Callback could be implemented in the user file
//     */
//    if(GPIO_Pin == ec_Pin)
//    {
//		Timer_Start();
//        uint32_t time = get_time();
//        static uint8_t state = 0;
//        if(ranging_start_time == 0 || ranging_end_time == 0) {
//            if (state == 0) {
//                state = 1;
//                ranging_start_time = time;
//            } else {
//                state = 0;
//                ranging_end_time = time;
//				Timer_Stop();
//            }
//        }
//    }
//}
