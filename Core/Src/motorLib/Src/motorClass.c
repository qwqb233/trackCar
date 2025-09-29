//
// Created by qwqb233 on 2025/2/24.
// 使用方法：
// 1. 初始化电机对象：MotorClass motor;
// 2. 调用motor_init()初始化电机对象
// 3. 手动设置电机的相关硬件参数：motor.motor_tim_1 = &htim1; motor.motor_tim_ch_1 = TIM_CHANNEL_1; motor.motor_tim_2 = &htim2; motor.motor_tim_ch_2 = TIM_CHANNEL_2; motor.arr = 10000; motor.motor_encoder_tim = &htim3；
// note:PID控制器使用步骤：
// 1. 初始化PID控制器参数：Kp,Ki,Kd
// 2. 设定目标值：target_speed
// 3. 调用get_motor_encoder()将电机转速放入类中motor_encoder
// 4. 调用motor_PID_control()计算PID输出值
// 5. 调用motor_pwm_control()将PID输出值作为PWM输出值
// 6. 调用motor_get_encoder()更新电机转速
// 7. 重复步骤4-6，直到目标速度达到
// 8. 调用motor_go()将所有电机转速设定为目标速度
// 9. 调用motor_control_AllInOne()可以省略如上步骤通过target_speed数组控制所有电机转速
// PS: motor_init()、motor_control_AllInOne()、motor_go()、motor_speed_to_car_speed()、car_speed_to_motor_speed()与motorClass相关参数都是一个4个元素的数组，分别对应四个电机。

#include "main.h"
#include "motorClass.h"
#include <stdlib.h>

//TODO：测量小车的尺寸
float W = 17.0f;
float H = 16.0f;

void motor_set_V(MotorClass * motor,float V)
{
    motor->motor_V = V;
}
void motor_set_encoder(MotorClass * motor,float encoder)
{
    motor->motor_encoder = encoder;
}
void motor_set_distance(MotorClass * motor,float distance)
{
    motor->motor_distance = distance;
}

float motor_get_V(MotorClass * motor)
{
    return motor->motor_V;
}
float motor_get_encoder(MotorClass * motor)
{
    return motor->motor_encoder;
}
float motor_get_distance(MotorClass * motor)
{
    return motor->motor_distance;
}


/*
 * @brief 使用PID算法控制电机转速
 * @param motor 电机对象指针
 * @param target_speed 目标速度
 * @return 电机pwm输出值
 */
int16_t motor_PID_control(MotorClass * motor,float target_speed)
{
    static int16_t motor_pwm_out = 0;
    static float err,err_last,err_integral = 0;
    float speed = motor->motor_speed;
	int now_duty = motor->motor_now_duty;
	if(target_speed < 0){target_speed = -target_speed;speed = -speed;}
    err = target_speed - speed;
    err_integral += err*0.01;
    motor_pwm_out = (int16_t)(motor->kP * err + motor->kI * err_integral + motor->kD * (err - err_last));
    err_last = err;
    int16_t result = motor_pwm_out + now_duty;
    return result;
}

/*
 * @brief 直接控制电机pwm输出
 * @param motor 电机对象指针
 * @param duty pwm占空比
 * @return 无
 */
void motor_pwm_control(MotorClass * motor,int duty)
{
    if(duty >= 0)
    {
        set_pwm_compare(motor->motor_tim_1,motor->motor_tim_ch_1,ccr_count(duty,motor->arr));
        set_pwm_compare(motor->motor_tim_2,motor->motor_tim_ch_2,0);
    }
    else
    {
		duty = -duty;
        set_pwm_compare(motor->motor_tim_1,motor->motor_tim_ch_1,0);
        set_pwm_compare(motor->motor_tim_2,motor->motor_tim_ch_2,ccr_count(duty,motor->arr));
    }
    if(duty < 0)motor->motor_now_duty = -duty;
	else motor->motor_now_duty = duty;
}



/*
 * @brief 所有电机同速转动
 * @param motor 电机对象指针数组
 * @param speed 转速
 * @return 无
 */
void motor_go(MotorClass * motor[4],float speed)
{
    float speed_list[] = {speed,speed,speed,speed};
    motor_control_AllInOne(motor,speed_list);
}

/*
 * @brief 由电机转速计算小车速度
 * @param motor 电机对象指针数组
 * @return 小车速度对象指针
 */
CarClass *  motor_speed_to_car_speed(CarClass * car,MotorClass * motor[4])
{
    float Va = -motor[1]->motor_speed;
    float Vb = -motor[2]->motor_speed;
    float Vc = motor[0]->motor_speed;
    float Vd = motor[3]->motor_speed;
    car->Vy = (Va + Vb + Vc + Vd) / 4;
    car->Vx = (-Va + Vb + Vc - Vd) / 4;
    car->Vw = (-Va + Vb - Vc + Vd) / (2*H + 2*W);
    return car;
}

/*
 * @brief 由小车速度计算电机转速
 * @param car 小车对象指针
 * @param motor 电机对象指针数组
 * @return 无
 * @note 由于没有加速度计，通过设定小车速度来控制电机转速
 */
void car_speed_to_motor_speed(CarClass * car,MotorClass * motor[4])
{
    float Vx = car->Vx;
    float Vy = car->Vy;
    float Vw = car->Vw;
    motor[0]->motor_speed_target = -(Vx + Vy -((W+H)/2)*Vw);
    motor[1]->motor_speed_target = -(Vx - Vy -((W+H)/2)*Vw);
    motor[2]->motor_speed_target = (Vx + Vy +((W+H)/2)*Vw);
    motor[3]->motor_speed_target = (Vx - Vy +((W+H)/2)*Vw);
}


void get_motor_encoder(MotorClass * motor)
{
    uint8_t oneRound = 13;
    motor->motor_encoder = get_encoder(motor->motor_encoder_tim);
    clear_encoder(motor->motor_encoder_tim);
    motor->motor_speed = (float )((float )((float)motor->motor_encoder*16600)/48/(float)oneRound/4/10);

    //clear_encoder(motor->motor_encoder_tim);
}

/*
 * @brief 使用PID控制所有电机
 * @param motor 电机对象指针数组
 * @param target_speed 目标速度数组
 * @return 无
 */
void motor_control_AllInOne(MotorClass * motor[4],float target_speed[4])
{
    for(int i = 0;i < 4;i++)
    {
		get_motor_encoder(motor[i]);
		int duty = (int16_t)motor[i]->motor_PID_control(motor[i],target_speed[i]);
		if(target_speed[i] < 0)duty = -duty;
   
		motor[i]->motor_pwm_control(motor[i],duty);
    }
}

void __motor_init(MotorClass * motor)//,TIM_HandleTypeDef * motor_tim_1,uint16_t motor_tim_ch_1,TIM_HandleTypeDef * motor_tim_2,uint16_t motor_tim_ch_2,uint16_t arr)
{
    motor->kP = 0.0f;
    motor->kI = 0.0f;
    motor->kD = 0.0f;
    motor->motor_V = 0.0f;
    motor->motor_encoder = 0.0f;
    motor->motor_distance = 0.0f;
	motor->motor_speed_target = 0.0f;

/*    motor->motor_tim_1 = motor_tim_1;
    motor->motor_tim_ch_1 = motor_tim_ch_1;
    motor->motor_tim_2 = motor_tim_2;
    motor->motor_tim_ch_2 = motor_tim_ch_2;
    motor->arr = arr;*/

    motor->get_motor_V = motor_get_V;
    motor->get_motor_encoder = motor_get_encoder;
    motor->get_motor_distance = motor_get_distance;
    motor->set_motor_V = motor_set_V;
    motor->set_motor_encoder = motor_set_encoder;
    motor->set_motor_distance = motor_set_distance;
    motor->motor_pwm_control = motor_pwm_control;
    motor->motor_get_encoder = get_motor_encoder;
    motor->motor_PID_control = motor_PID_control;
}

//初始化电机
void motor_init(MotorClass * motorList[4])
{
    for(int i = 0;i < 4;i++)
    {
        __motor_init(motorList[i]);
    }
}
