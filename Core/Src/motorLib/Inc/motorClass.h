//
// Created by qwqb233 on 2025/2/24.
//

#ifndef CARTRACE_1_MOTORCLASS_H
#define CARTRACE_1_MOTORCLASS_H

#define ccr_count(duty,arr) duty*arr/100

typedef struct motorClass
{
    //PID相关参数
    float kP;
    float kI;
    float kD;
    //控制电机的两个定时器
    TIM_HandleTypeDef * motor_tim_1;
    uint16_t motor_tim_ch_1;
    TIM_HandleTypeDef * motor_tim_2;
    uint16_t  motor_tim_ch_2;
    //定时器的自动重装器的数值，用于计算占空比
    uint16_t arr;
    //编码器相关参数
    //编码器的定时器
    TIM_HandleTypeDef * motor_encoder_tim;
    //电机的速度、编码器的数值、距离、由运动学关系计算的速度、设置占空比、当前占空比
    float motor_V;
    short motor_encoder;
    float motor_distance;
    float motor_speed;
	float motor_speed_target;
    int motor_set_duty;
    int motor_now_duty;

    float (*get_motor_V)(struct motorClass * motor);
    float (*get_motor_encoder)(struct motorClass * motor);
    float (*get_motor_distance)(struct motorClass * motor);
    void (*set_motor_V)(struct motorClass * motor,float V);
    void (*set_motor_encoder)(struct motorClass * motor,float encoder);
    void (*set_motor_distance)(struct motorClass * motor,float distance);
    void (*motor_pwm_control)(struct motorClass * motor,int duty);
    void (*motor_get_encoder)(struct motorClass * motor);
    void (*motor_go)(struct motorClass * motor[4],float speed);
    void (*motor_control_AllInOne)(struct motorClass * motor[4],float target_speed[4]);
    int16_t (*motor_PID_control)(struct motorClass * motor,float target_speed);
}MotorClass;



typedef struct carClass
{
    float Vx;
    float Vy;
    float Vw;
}CarClass;


void motor_go(MotorClass * motor[4],float speed);
void motor_control_AllInOne(MotorClass * motor[4],float target_speed[4]);
void __motor_init(MotorClass * motor);//,TIM_HandleTypeDef * motor_tim_1,uint16_t motor_tim_ch_1,TIM_HandleTypeDef * motor_tim_2,uint16_t motor_tim_ch_2,uint16_t arr);
void motor_init(MotorClass * motorList[4]);
int16_t motor_PID_control(MotorClass * motor,float target_speed);


void car_speed_to_motor_speed(CarClass * car,MotorClass * motor[4]);
CarClass *  motor_speed_to_car_speed(CarClass * car,MotorClass * motor[4]);
#endif //CARTRACE_1_MOTORCLASS_H
