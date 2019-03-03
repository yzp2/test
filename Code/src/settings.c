/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <mavlink.h>
#include "settings.h"

enum global_param_id_t global_param_id;
struct global_struct global_data;

extern uint8_t debug_int_message_buffer(const char* string, int32_t num);

/**
 * @brief reset all parameters to default 初始化所有会用到的参数
 */
void global_data_reset_param_defaults(void){

	global_data.param[PARAM_SYSTEM_ID] = 81;
	strcpy(global_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");
	global_data.param_access[PARAM_SYSTEM_ID] = READ_WRITE;

	global_data.param[PARAM_COMPONENT_ID] = 50;
	strcpy(global_data.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");
	global_data.param_access[PARAM_COMPONENT_ID] = READ_WRITE;

	global_data.param[PARAM_SENSOR_ID] = 77;
	strcpy(global_data.param_name[PARAM_SENSOR_ID], "SYS_SENSOR_ID");
	global_data.param_access[PARAM_SENSOR_ID] = READ_WRITE;

	global_data.param[PARAM_SYSTEM_TYPE] = MAV_TYPE_GENERIC;
	strcpy(global_data.param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");
	global_data.param_access[PARAM_SYSTEM_TYPE] = READ_WRITE;

	global_data.param[PARAM_AUTOPILOT_TYPE] = MAV_AUTOPILOT_GENERIC;
	strcpy(global_data.param_name[PARAM_AUTOPILOT_TYPE], "SYS_AP_TYPE");
	global_data.param_access[PARAM_AUTOPILOT_TYPE] = READ_WRITE;

	global_data.param[PARAM_SW_VERSION] = 1300;
	strcpy(global_data.param_name[PARAM_SW_VERSION], "SYS_SW_VER");
	global_data.param_access[PARAM_SW_VERSION] = READ_WRITE;

	global_data.param[PARAM_SYSTEM_SEND_STATE] = 1;
	strcpy(global_data.param_name[PARAM_SYSTEM_SEND_STATE], "SYS_SEND_STATE");
	global_data.param_access[PARAM_SYSTEM_SEND_STATE] = READ_WRITE;

	global_data.param[PARAM_SYSTEM_SEND_LPOS] = 0;
	strcpy(global_data.param_name[PARAM_SYSTEM_SEND_LPOS], "SYS_SEND_LPOS");
	global_data.param_access[PARAM_SYSTEM_SEND_LPOS] = READ_WRITE;

	global_data.param[PARAM_SENSOR_POSITION] = 0; // BOTTOM
	strcpy(global_data.param_name[PARAM_SENSOR_POSITION], "POSITION");
	global_data.param_access[PARAM_SENSOR_POSITION] = READ_WRITE;

	global_data.param[PARAM_USART2_BAUD] = 115200;
	strcpy(global_data.param_name[PARAM_USART2_BAUD], "USART_2_BAUD");
	global_data.param_access[PARAM_USART2_BAUD] = READ_ONLY;

	global_data.param[PARAM_USART3_BAUD] = 115200;
//	global_data.param[PARAM_USART3_BAUD] = 921600;
	strcpy(global_data.param_name[PARAM_USART3_BAUD], "USART_3_BAUD");
	global_data.param_access[PARAM_USART3_BAUD] = READ_ONLY;

	global_data.param[PARAM_FOCAL_LENGTH_MM] = 16.0f;
	strcpy(global_data.param_name[PARAM_FOCAL_LENGTH_MM], "LENS_FOCAL_LEN");
	global_data.param_access[PARAM_FOCAL_LENGTH_MM] = READ_WRITE;

	global_data.param[PARAM_GYRO_SENSITIVITY_DPS] = 250;
	strcpy(global_data.param_name[PARAM_GYRO_SENSITIVITY_DPS], "GYRO_SENS_DPS");
	global_data.param_access[PARAM_GYRO_SENSITIVITY_DPS] = READ_WRITE;

	global_data.param[PARAM_GYRO_COMPENSATION_THRESHOLD] = 0.01;
	strcpy(global_data.param_name[PARAM_GYRO_COMPENSATION_THRESHOLD], "GYRO_COMP_THR");
	global_data.param_access[PARAM_GYRO_COMPENSATION_THRESHOLD] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_GYRO] = 1; // send gyro debug values over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_GYRO], "USB_SEND_GYRO");
	global_data.param_access[PARAM_USB_SEND_GYRO] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_FORWARD] = 0; // send forward flow over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_FORWARD], "USB_SEND_FWD");
	global_data.param_access[PARAM_USB_SEND_FORWARD] = READ_WRITE;

	global_data.param[PARAM_USB_SEND_DEBUG] = 1; // send debug msgs over USB
	strcpy(global_data.param_name[PARAM_USB_SEND_DEBUG], "USB_SEND_DEBUG");
	global_data.param_access[PARAM_USB_SEND_DEBUG] = READ_WRITE;

	global_data.param[DEBUG_VARIABLE] = 1;
	strcpy(global_data.param_name[DEBUG_VARIABLE], "DEBUG");
	global_data.param_access[DEBUG_VARIABLE] = READ_WRITE;

}

/**
 * @brief resets the global data struct to all-zero values
 */
void global_data_reset(void)
{
	// not in use anymore
}

/**
 * @brief changes read only settings depending on sensor position
 */
void set_sensor_position_settings(uint8_t sensor_position)
{

	switch(sensor_position)
	{
		case(BOTTOM):
			break;

		default:
			debug_int_message_buffer("Unused sensor position:", sensor_position);
			return;
	}

	debug_int_message_buffer("Set sensor position:", sensor_position);
	return;
}

