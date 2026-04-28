#include "AS5600.h"
#include <stdint.h>
#define AS5600_ADDRESS (0x36 << 1) // AS5600 I2C地址
#define AS5600_RAW_ANGLE_H    0x0C // AS5600原始角度寄存器高字节地址
#define AS5600_RAW_ANGLE_L    0x0D // AS5600原始角度寄存器低字节地址
#define AS5600_IIC      &hi2c2 // I2C句柄，根据实际情况修改
#define _2PI 6.28318530718f
#define AS5600_DIRECT -1 // 1为顺时针增大，-1为逆时针增大，根据实际情况修改


void AS5600_WriteReg(uint8_t RegAddress, uint8_t Data) // 写寄存器函数
{
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 发送数据
}

uint8_t AS5600_ReadReg(uint8_t RegAddress) // 读寄存器函数
{
    uint8_t Data;
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_I2C_Master_Receive(AS5600_IIC, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 接收数据
    return Data;
}

 /**
   * 函    数：获取AS5600原始角度值的函数
   * 功    能：获取AS5600原始角度值
   * 输    入：无
   * 参    数：无
   * 返 回 值：原始角度值
   */
float AS5600_GetRawAngle(void)
{
    uint8_t reg = AS5600_RAW_ANGLE_H;
    uint8_t data[2];
    
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(AS5600_IIC, AS5600_ADDRESS, data, 2, HAL_MAX_DELAY);
    
    uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x0FFF; // 12位原始角度值
    return (float)raw * _2PI / 4096.0f;
}

float full_rotations = 0.0f;//圈数
float Last_Angle = 0.0f;
//磁编码器弧度制角度累计计算:(0-∞)



 /**
   * 函    数：获取累计角度值的函数
   * 功    能：获取累计角度值
   * 输    入：无
   * 参    数：无
   * 返 回 值：累计角度值
   */
float GetAngle(void)
{
    float angle = AS5600_GetRawAngle(); 
    float d_angle = angle - Last_Angle;

    if (d_angle > M_PI) 
    {
        full_rotations -= 1;
    } 
    else if (d_angle < -M_PI) 
    {
        full_rotations += 1;
    }

    Last_Angle = angle;

    return  AS5600_DIRECT * (full_rotations * _2PI + angle);
}

 /**
   * 函    数：获取不带累计的角度值的函数
   * 功    能：获取不带累计的角度值
   * 输    入：无
   * 参    数：无
   * 返 回 值：角度值
   */
float GetAngle_Without_Track()
{
    float Angle = AS5600_DIRECT * AS5600_GetRawAngle();
    while (Angle < 0.0f)
    {
        Angle += _2PI;
    }
    while (Angle >= _2PI)
    {
        Angle -= _2PI;
    }
    return Angle;

}