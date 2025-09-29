//
// Created by qwqb233 on 2025/2/27.
//

#ifndef CARTRACE_1_US_RANGING_H
#define CARTRACE_1_US_RANGING_H

uint16_t get_distance_cm(GPIO_TypeDef* trigger_port, uint16_t trigger_pin);
uint16_t get_distance_cm_nT(GPIO_TypeDef * trigger_port, uint16_t trigger_pin);
#endif //CARTRACE_1_US_RANGING_H
