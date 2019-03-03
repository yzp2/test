 
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "settings.h"
#include "utils.h"
#include "led.h"
#include "gyro.h"
#include "i2c.h"
#include "usart.h"
#include "communication.h"
#include "debug.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "main.h"

/* coprocessor control register (fpu) 浮点数运算单元寄存器地址 */
#ifndef SCB_CPACR
#define SCB_CPACR (*((uint32_t*) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif


/* prototypes 样板？ */
void delay(unsigned msec);
 
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

 
/* boot time in milliseconds ticks */
volatile uint32_t boot_time_ms = 0;
/* boot time in 10 microseconds ticks */
volatile uint32_t boot_time10_us = 0;

/* timer constants 定时器 常数 */
#define NTIMERS         			9		//定时器数目
#define TIMER_CIN       			0		//没有用到该宏定义
#define TIMER_LED       			1		//LED定时编号
#define TIMER_DELAY     			2		//延时用定时编号
 
#define TIMER_SYSTEM_STATE		4		//系统状态切换用定时编号
#define TIMER_RECEIVE					5		//接收信号定时编号
#define TIMER_PARAMS					6		//参数发送定时编号
 
#define TIMER_LPOS						8		//位置发送定时编号
/*  各定时器的计数值 每隔计数值长的时间，在 timer_update_ms() 函数中就会执行相应动作 */
#define MS_TIMER_COUNT				100 /* steps in 10 microseconds ticks */
#define LED_TIMER_COUNT				500 /* steps in milliseconds ticks */
 
#define SYSTEM_STATE_COUNT		1000/* steps in milliseconds ticks */
#define PARAMS_COUNT					100	/* steps in milliseconds ticks */
#define LPOS_TIMER_COUNT 			100	/* steps in milliseconds ticks */

static volatile unsigned timer[NTIMERS];
static volatile unsigned timer_ms = MS_TIMER_COUNT;

/* timer/system booleans 定时器/系统布尔值 */
bool send_system_state_now = true;
bool receive_now = true;
bool send_params_now = true;
 
 

 
/**
  * @brief  Increment boot_time_ms variable and decrement timer array.
						增加boot_time_ms变量并减少计时器数组
  * @param  None
  * @retval None
  */
void timer_update_ms(void)
{
	boot_time_ms++;

	/* each timer decrements every millisecond if > 0  每 个计时器每毫秒递减一次*/
	for (unsigned i = 0; i < NTIMERS; i++)
	if (timer[i] > 0)
			timer[i]--;

	if (timer[TIMER_LED] == 0)
	{
		/* blink activitiy */
		LEDToggle(LED_ACT);
		timer[TIMER_LED] = LED_TIMER_COUNT;
	}

	if (timer[TIMER_SYSTEM_STATE] == 0)
	{
		send_system_state_now = true;
		timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_RECEIVE] == 0)
	{
		receive_now = true;
		timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_PARAMS] == 0)
	{
		send_params_now = true;
		timer[TIMER_PARAMS] = PARAMS_COUNT;
	}

	if (timer[TIMER_LPOS] == 0)
	{
 
		timer[TIMER_LPOS] = LPOS_TIMER_COUNT;
	}
}

/**
  * @brief  Increment boot_time10_us variable and decrement millisecond timer, triggered by timer interrupt
	*					增加boot_time10_us变量并减少毫秒定时器，由SysTick_Handler系统定时器中断触发
  * @param  None
  * @retval None
  */
void timer_update(void)
{
	boot_time10_us++;

	/*  decrements every 10 microseconds*/
	timer_ms--;

	if (timer_ms == 0)
	{
		timer_update_ms();
		timer_ms = MS_TIMER_COUNT;
	}

}


uint32_t get_boot_time_ms(void)
{
	return boot_time_ms;
}

uint32_t get_boot_time_us(void)
{
	return boot_time10_us*10;// *10 to return microseconds
}

void delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;
	while (timer[TIMER_DELAY] > 0) {};
}

 

/*****************************************************************************************************************
  * @brief  Main function.
  */
float x_rate_sensor, y_rate_sensor, z_rate_sensor;
float x_rate;
float y_rate;
float z_rate;
int main(void)
{
	/* load settings and parameters 初始化参数 */
	global_data_reset_param_defaults();
	global_data_reset();

	/* init led 初始化LED */
	LEDInit(LED_ACT);
	LEDInit(LED_COM);
	LEDInit(LED_ERR);
	LEDOff(LED_ACT);
	LEDOff(LED_COM);
	LEDOff(LED_ERR);

	/* enable FPU on Cortex-M4F core 使能浮点数计算单元 */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access设置cp10和cp11完全访问 */

	/* init clock 初始化时钟*/
	if (SysTick_Config(SystemCoreClock / 100000))/*set timer to trigger interrupt every 10 microsecond */
	{
		/* capture clock error 若系统时钟出错LED亮红灯 */
		LEDOn(LED_ERR);
		while (1);
	}

	/* init usb           usb虚拟串口 */
	USBD_Init(	&USB_OTG_dev,
				USB_OTG_FS_CORE_ID,
				&USR_desc,
				&USBD_CDC_cb,
				&USR_cb);

	/* init mavlink */
	communication_init();
  /* gyro config 陀螺仪初始化 */
	gyro_config();
  /* usart config*/
	usart_init();
  /* i2c config*/
  i2c_init();
  /* reset/start timers */
 	timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT / 2;
	timer[TIMER_PARAMS] = PARAMS_COUNT;
  /* variables */
	uint32_t counter = 0;
	uint8_t qual = 0;
 
 
	static float accumulated_gyro_x = 0;     
	static float accumulated_gyro_y = 0;
	static float accumulated_gyro_z = 0;
 


	/* main loop */
	while (1)
	{
		int16_t gyro_temp;
		gyro_read(&x_rate_sensor, &y_rate_sensor, &z_rate_sensor,&gyro_temp);//读取值并赋给新设定的变量

		/* gyroscope coordinate transformation陀螺仪坐标变换 */
		 x_rate = y_rate_sensor; // change x and y rates
		 y_rate = - x_rate_sensor;
	   z_rate = z_rate_sensor; // z is correct
	}
}
