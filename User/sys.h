#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx.h"
#include "motor_config.h"

/* 位带操作，像51一样直接控制IO口，参考CM4权威指南 */
#define BITBAND(addr, bitnum)		((addr & 0xF0000000)+0x2000000+((addr & 0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)			*((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum)		MEM_ADDR(BITBAND(addr, bitnum))

/* IO口地址映射，F4的ODR偏移是0x14 */
#define GPIOA_ODR_Addr			(GPIOA_BASE+0x14)
#define GPIOB_ODR_Addr			(GPIOB_BASE+0x14)
#define GPIOC_ODR_Addr			(GPIOC_BASE+0x14)
#define GPIOD_ODR_Addr			(GPIOD_BASE+0x14)
#define GPIOE_ODR_Addr			(GPIOE_BASE+0x14)
#define GPIOF_ODR_Addr			(GPIOF_BASE+0x14)
#define GPIOG_ODR_Addr			(GPIOG_BASE+0x14)

#define GPIOA_IDR_Addr			(GPIOA_BASE+0x10)
#define GPIOB_IDR_Addr			(GPIOB_BASE+0x10)
#define GPIOC_IDR_Addr			(GPIOC_BASE+0x10)
#define GPIOD_IDR_Addr			(GPIOD_BASE+0x10)
#define GPIOE_IDR_Addr			(GPIOE_BASE+0x10)
#define GPIOF_IDR_Addr			(GPIOF_BASE+0x10)
#define GPIOG_IDR_Addr			(GPIOG_BASE+0x10)

/* IO口操作，只对单一IO口，n小于16 */
#define PAout(n)			BIT_ADDR(GPIOA_ODR_Addr,n)	//输出
#define PAin(n)				BIT_ADDR(GPIOA_IDR_Addr,n)	//输入
#define PBout(n)			BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)				BIT_ADDR(GPIOB_IDR_Addr,n)
#define PCout(n)			BIT_ADDR(GPIOC_ODR_Addr,n)
#define PCin(n)				BIT_ADDR(GPIOC_IDR_Addr,n)
#define PDout(n)			BIT_ADDR(GPIOD_ODR_Addr,n)
#define PDin(n)				BIT_ADDR(GPIOD_IDR_Addr,n)
#define PEout(n)			BIT_ADDR(GPIOE_ODR_Addr,n)
#define PEin(n)				BIT_ADDR(GPIOE_IDR_Addr,n)
#define PFout(n)			BIT_ADDR(GPIOF_ODR_Addr,n)
#define PFin(n)				BIT_ADDR(GPIOF_IDR_Addr,n)
#define PGout(n)			BIT_ADDR(GPIOG_ODR_Addr,n)
#define PGin(n)				BIT_ADDR(GPIOG_IDR_Addr,n)

/* 全局变量 */
extern int Motor_Left,Motor_Right;			//左右轮PWM（千分比，-1000~1000）
extern int Encoder_Left,Encoder_Right;			//左右轮编码器读数（每10ms）
extern float Encoder_Speed[MOTOR_COUNT];		//左右轮转速RPM
extern float Speed_Target[MOTOR_COUNT];			//目标转速RPM
extern float Speed_Measured[MOTOR_COUNT];		//滤波后的实际转速RPM
extern float Speed_Integral[MOTOR_COUNT];		//PI积分项
extern float Speed_Output[MOTOR_COUNT];			//速度环输出（千分比）
extern u8 Speed_Filter_Flag[MOTOR_COUNT];		//滤波初值标志
extern u8 Speed_Startup_Flag[MOTOR_COUNT];		//启动补偿状态
extern u16 Speed_Stall_Time[MOTOR_COUNT];		//连续低速计时ms
extern u8 Speed_Stall_Fault[MOTOR_COUNT];		//堵转保护锁存

/* 各模块头文件 */
#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "speed_control.h"
#include "control_timer.h"

#endif
