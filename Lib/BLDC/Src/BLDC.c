#include "BLDC.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "AS5600.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>
#include <math.h>

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))//限制函数，限制amt在low和high之间
#define BLDC_DIRECT -1 //电机旋转方向，1为逆时针正方向，-1为顺时针，根据实际情况修改
#define POLE_PAIRS                7                       //极对数，根据实际电机调整
#define _3PI_2                    4.71238898038f         //3pi/2，电角度270度，帮助电机启动
#define PWM_TIM                   &htim1                    //PWM定时器，根据实际情况修改
#define TIMESTAMP_TIM             &htim4                //时间戳定时器，根据实际情况修改
float voltage_limit =             10.0;           //BLDC电压限制
float voltage_power_supply =      12.0;    //电源电压
float shaft_angle =               0;                //轴角度
uint32_t open_loop_timestamp =    0;     //开环时间戳
float zero_electrical_angle =     0;      //零电角度
float Ualpha = 0, Ubeta =         0;          //两相电压
float ua = 0, ub = 0, uc =        0;         //三相电压
float da_a = 0, db_b = 0, dc_c =  0;   //三相占空比


/**
  * 函    数：电机初始化函数,定时器1用于PWM输出，定时器4用于开环速度生成器的时间基准(单位：ms)
  * 功    能：此函数内部只启动PWM输出，定时器需要cubemx，此处使用二分频，
                自动重装载值为1800-1，中心对齐模式3，PWM频率约为20kHz，分辨率为1/1800
  *             用户需要根据实际情况调整定时器参数以满足电机控制需求
    *           其他功能如传感器校准、零电角度测量等需要用户根据实际情况添加（以后再写）
  * 输    入：无
  * 参    数：无
  * 返 回 值：无
  */
void BLDC_Init(void)
{
    // 启动PWM
    HAL_TIM_PWM_Start(PWM_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(PWM_TIM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(PWM_TIM, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(TIMESTAMP_TIM); // 启动时间戳定时器
    char buffer[30] = {"完成电机初始化"};
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer,
                         sizeof(buffer), HAL_MAX_DELAY);

    HAL_Delay(1000); // 等待电机稳定
    set_phase_voltage(3, 0, _3PI_2); // 以电角度270度（即物理角度270/极对数）施加电压，帮助电机启动
    HAL_Delay(3000);
    zero_electrical_angle = _electricalangle(); // 读取当前电角度作为零电角度
    set_phase_voltage(0, 0, _3PI_2); // 施加0电压，停止电机
}

/**
  * 函    数：计算电角度、设置三相电压和设置相电压的函数
  * 功    能：计算电角度
  * 输    入：物理角度与极对数
  * 返 回 值：电角度
  */

float electrical_angle(float shaft_Angle, uint8_t pole_pairs)// 计算电角度
{
    return (shaft_Angle * pole_pairs);
}


float _electricalangle(void)
{
    return angle_nomalize((float)(POLE_PAIRS) * GetAngle_Without_Track() - zero_electrical_angle);;
}
/**
  * 函    数：电角度归一化函数
  * 功    能：归一化电角度
  * 输    入：电角度
  * 返 回 值：归一化后的电角度（0-360度）
  */

float angle_nomalize(float angle)// 角度归一化到0-2pi范围内
{
    while (angle > 2.0 * M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < 0.0)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/**
  * 函    数：计算三项占空比和设置三相电压的函数
  * 功    能：计算三项占空比并设置三相电压
  * 输    入：三相电压
  * 返 回 值：无
  */

void set_PWM(float Ua, float Ub, float Uc) // 设置三相电压
{
    // 电压限制
    Ua = _constrain(Ua, 0, voltage_limit);
    Ub = _constrain(Ub, 0, voltage_limit);
    Uc = _constrain(Uc, 0, voltage_limit);

    // 电压转占空比,限制占空比在0-1之间
    da_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    db_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    // 占空比转PWM值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(da_a * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1)));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(db_b * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1)));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1)));
}

/**
  * 函    数：根据输入电压，电角度进行克拉克逆变换和帕克逆变换，并设置PWM输出的函数
  * 功    能：根据输入电压进行克拉克逆变换和帕克逆变换，并设置PWM输出
  * 输    入：Uq：q轴电压，Ud：d轴电压，Angle：电角度（rad）
  * 参    数：无
  * 返 回 值：无
  */

void set_phase_voltage(float Uq, float Ud, float Angle)
{
    Angle = angle_nomalize(Angle + zero_electrical_angle); // 计算电角度并归一化
    
    //帕克逆变换，将d轴和q轴电压转换为两相静止坐标系下的电压，注意这里的正负号和坐标系定义有关，可能需要根据实际情况调整
    Ualpha = -Uq * sinf(Angle) + Ud * cosf(Angle);
    Ubeta = Uq * cosf(Angle) + Ud * sinf(Angle);

    //克拉克逆变换，并加上电源电压的一半以实现正负电压的输出，保证占空比在0-1之间
    ua = Ualpha + voltage_power_supply / 2.0f;
    ub = (sqrt(3) * Ubeta - Ualpha) / 2.0f + voltage_power_supply / 2.0f;
    uc = (-sqrt(3) * Ubeta - Ualpha) / 2.0f + voltage_power_supply / 2.0f;

    // 设置PWM
    set_PWM(ua, ub, uc);//随便写测试git的注释
}

 /**
   * 函    数：开环速度生成器
   * 功    能：请注意开环调控不要间隔太久，可能会导致tim4溢出超过两次，大概会出问题？
   * 输    入：角速度(单位：rad / s)
   * 参    数：极对数
   * 返 回 值：当前电压，主要用于调试
   */
 float velocity_open_loop(float target_velocity)
 {
     uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim4); // 获取当前时间（单位：ms）
     //定时器记一次数0.5ms，所以除以2000.0f得到秒，大概32秒会溢出一次，接下来处理溢出
     //最大分频只能65536，为了取整用36000-1，所以定时器频率为72MHz/36000=2000Hz，即每0.5ms计数一次
     if (current_time < open_loop_timestamp) // 处理定时器溢出
     {
         current_time += 65536; // 定时器溢出后加上最大值
         //如果溢出超过64秒，大概会出问题?但一般不会出现这么长时间的开环运行，所以暂不处理
     }
     float dt = (current_time - open_loop_timestamp) / 2000.0f; // 计算时间差（单位：s）
     shaft_angle = angle_nomalize(shaft_angle + target_velocity * dt); // 更新轴角度，target_velocity单位为rad/s
     float Uq = voltage_limit; // 开环时直接使用最大电压
     float Ud = 0; // d轴电压为0
     set_phase_voltage(Uq, Ud, electrical_angle(shaft_angle, POLE_PAIRS)); // 设置相电压
     open_loop_timestamp = current_time; // 更新时间戳
     // 返回当前电压，主要用于调试
     return Uq;
}


 /**
   * 函    数：根据输入目标角度和当前角度进行位置控制的函数，使用简单的P控制器
   * 功    能：根据输入目标角度和当前角度进行位置控制，
   * 输    入：目标角度（单位：rad）
   * 参    数：无
   * 返 回 值：无
   */
float Kd_Last_Angle1 = 0.0f;
float Error0 = 0.0f;
float Error1 = 0.0f;
float DifOut = 0.0f;
float a = 0.5f;
void Set_TarAngle(float Target_Angle)
{
  
  float Sensor_Angle = GetAngle();
  float kp = 0.133;
  float kd = 0.05;
  Error1 = Error0;
  Error0 = Target_Angle - Sensor_Angle;
  DifOut = kd * a * (Error0 - Error1) + (1 - a) * DifOut; 
  float Uq1 = BLDC_DIRECT * _constrain(kp * Error0 * 180 / M_PI + DifOut * 180 / M_PI, -6, 6);
  set_phase_voltage(Uq1,0, _electricalangle());
  Kd_Last_Angle1 = Sensor_Angle;
}
  


