#include "uart_prot.h"
#include "CircularQueue.h"
#include "crc.h"
#include "ds3232.h"
#include "interflash.h"
#include "sw6301.h"
#include <string.h>

CircularQueue_Str Uart_Recv_Queue;
uint8_t Uart_Recv_Fifo[UART_MSG_LEN];
extern UART_HandleTypeDef hlpuart1;

common_rep global_discharge_state;
uint8_t msg_content[UART_MSG_LEN];
msg_proc msgpro = {0};
//uint16_t total_index = 0;
uint32_t last_tick = 0;
uint8_t timeout_cnt = 0;
static uint8_t uart_content[UART_MSG_LEN];
common_rep global_mcu_log;
soc_sys_state soc_sys_state_ins;
mcu_version mcu_version_ins;
shutdown_soc shutdown_soc_ins;
rtc_datetime rtc_datetime_ins;
battery_info global_battery_info;
battery_ctrl battery_ctrl_ins;
mcu_self_check mcu_self_check_ins;
soc_power_num global_soc_power_num;
uint8_t log_fill[530];
uint8_t actual_log_fill[512];
uint8_t upgrade_pack[1024];
req_upgrade_mcu global_upgrade_mcu = {0};
common_rep global_soc_ack = {0};
mcu_upgrade_pack mcu_pack = {0};
uint8_t enter_upgrade_flow = 0;
uint8_t crc_fail_num = 0;
uint8_t update_percentage = 0;
heartbeat_pro heartbeat_value = {0};
shutdown_soc shutdown_value = {0};
shutdown_state shutdown_state_value = {0};
charge_state chargestate = {0};
mcu_req_timeout mcu_timeout = {0};




#define APPLICATION_ADDRESS	10

typedef void (*pFunction)(void);

uint32_t flashdestination = APPLICATION_ADDRESS;
uint32_t write_index = 0;
uint32_t JumpAddress;
pFunction Jump_To_Application;

#define MSG_FIXED_SIZE	sizeof(common_rep)	


void shutdown_soc_func(uint8_t shutdown)
{
	uart_msg msg_inst;
	if(shutdown == 1)
	{
		shutdown_value.shutdown = 1;
		msg_inst.msg_id = CMD_REQ_SHUTDOWN_SOC;
		uart_msg_proc_flow(1, &msg_inst);
		if(HAL_GetTick() - shutdown_state_value.curr_tick >= 3000)
		{
			//关机
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			memset(&shutdown_value, 0, sizeof(shutdown_value));
		}
	}
	else if(shutdown == 2)
	{
		shutdown_value.shutdown = 2;
		msg_inst.msg_id = CMD_REQ_SHUTDOWN_SOC;
		if(shutdown_state_value.req_msg_send )
		uart_msg_proc_flow(1, &msg_inst);
		if(HAL_GetTick() - shutdown_state_value.curr_tick >= 3000)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);//关机
		}

		if(HAL_GetTick() - shutdown_state_value.curr_tick >= 5000)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);//开机
			memset(&shutdown_state_value, 0, sizeof(shutdown_state_value));
		}
	}

}


void JumpToApplication(void)
{
	if(((*(__IO uint32_t *)APPLICATION_ADDRESS) & 0x2FFE0000) == 0x20000000)
	{
		JumpAddress = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
		Jump_To_Application = (pFunction) JumpAddress;

		__set_MSP(*(__IO uint32_t*)APPLICATION_ADDRESS);
		Jump_To_Application();
	}
}

void bsp_update_jumptoapp_evt_cbk(void)
{
	HAL_DeInit();
	__disable_irq();
	JumpToApplication();
	//bsp_Update_No_Application();
}

//fill_msg_flag fill_msg[0]= {.msg_len = MSG_FIXED_SIZE, .fixed_msg = 1, .};


void fill_msg(fill_msg_flag *msg_flag)
{
	common_rep rep_value;
	uint8_t temp;
	uint16_t calc_crc;
	msg_flag->msg_output[0] = 0xAA;
	msg_flag->msg_output[1] = 0xAA;
	memcpy(&msg_flag->msg_output[2], &msg_flag->msg_id, 2);

	if(msg_flag->errcode != EC_OK || msg_flag->msg_len == 0)
	{
		temp = MSG_FIXED_SIZE;
		memcpy(&msg_flag->msg_output[4], &temp, 2);
		rep_value.errcode = msg_flag->errcode;
		memcpy(&msg_flag->msg_output[6], &rep_value, sizeof(rep_value));
		CRC16_PAR(msg_flag->msg_output, 6 + MSG_FIXED_SIZE, &calc_crc);
		memcpy(&msg_flag->msg_output[6 + MSG_FIXED_SIZE], &calc_crc, 2);
		msg_flag->msg_output[8 + MSG_FIXED_SIZE] = 0x0D;
		msg_flag->msg_output[9 + MSG_FIXED_SIZE] = 0x0A;
		*msg_flag->msg_total_len = 10 + MSG_FIXED_SIZE;

	}
	else
	{
		temp = msg_flag->msg_len;
		memcpy(&msg_flag->msg_output[4], &temp, 2);
		rep_value.errcode = EC_OK;
		memcpy(&msg_flag->msg_output[6], &rep_value, sizeof(rep_value));
		memcpy(&msg_flag->msg_output[6 + sizeof(rep_value)], msg_flag->msg_content, msg_flag->msg_len);
		CRC16_PAR(msg_flag->msg_output, 6 + msg_flag->msg_len + sizeof(rep_value), &calc_crc);
		memcpy(&msg_flag->msg_output[6 + msg_flag->msg_len + sizeof(rep_value)], &calc_crc, 2);
		msg_flag->msg_output[8 + msg_flag->msg_len + sizeof(rep_value)] = 0x0D;
		msg_flag->msg_output[9 + msg_flag->msg_len + sizeof(rep_value)] = 0x0A;
		*msg_flag->msg_total_len = 10 + msg_flag->msg_len + sizeof(rep_value);	
	}
}

void form_ack_uart_msg(uart_msg *msg_ptr, uint8_t errcode)
{
	uint16_t msg_id;
	msg_id = msg_ptr->msg_id;
	fill_msg_flag msg_flag;
	msg_flag.msg_output = uart_content;
	msg_flag.errcode = errcode;
	
	switch(msg_id)
	{
		case CMD_REQ_HEARTBEAT://fixed_msg
			msg_flag.msg_id = CMD_REP_HEARTBEAT;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_SOC_STATE://fixed_msg
			msg_flag.msg_id = CMD_REP_SOC_STATE;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_GET_MCU_VER:
			msg_flag.msg_id = CMD_REP_GET_MCU_VER;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(mcu_version);
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_GET_SOC_POWER_NUM:
			msg_flag.msg_id = CMD_REP_GET_SOC_POWER_NUM;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(global_soc_power_num);
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_GET_RTC:
			msg_flag.msg_id = CMD_REP_GET_RTC;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(struct rtc_time);
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_SET_RTC://fixed_msg
			msg_flag.msg_id = CMD_REP_SET_RTC;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_MCU_CHARGE_CTRL://fixed_msg
			msg_flag.msg_id = CMD_REP_MCU_CHARGE_CTRL;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_UPGRADE_MCU://fixed_msg
			msg_flag.msg_id = CMD_REP_UPGRADE_MCU;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_UPLOAD_MCU_FW://fixed_msg
			msg_flag.msg_id = CMD_REP_UPLOAD_MCU_FW;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_MCU_UPD_STATE:
			msg_flag.msg_id = CMD_REP_MCU_UPD_STATE;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(upgrade_state);
			fill_msg(&msg_flag);
			break;
		default:
			break;
	}

	HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, *(msg_flag.msg_total_len) , 1000);
}

uint8_t API_UART_RxFifoEnable(uint8_t *rx_data, uint16_t length)
{
	if(rx_data == NULL || length == 0)
		return 1;
	Init_CircularQueue(&Uart_Recv_Queue, rx_data, length, sizeof(uint8_t));
	return 0;
}

uint8_t mcu_action_flow_before_ack(uart_msg *msg_ptr)
{
	uint8_t msg_id;
	int ret;
	rtc_datetime rtc_datetime;
	struct rtc_time rtc_time;
	msg_id = msg_ptr->msg_id;
	soc_sys_state __soc_sys_state;
	mcu_version mcu_version;
	battery_charge_state charge_state;
	uint8_t last_pack;
	uint16_t pack_index;
	
	switch((enum cmd_id_t)msg_id)
	{
		case CMD_REQ_SOC_STATE:
			memcpy(&__soc_sys_state, msg_ptr->uart_msg_data.msg_ptr, sizeof(__soc_sys_state));
			memcpy(&global_soc_power_num, &__soc_sys_state, sizeof(__soc_sys_state));
			ret = EC_OK;
			break;
		case CMD_REQ_SET_RTC:
			memcpy(&rtc_datetime, msg_ptr->uart_msg_data.msg_ptr, sizeof(rtc_datetime));
			rtc_time.tm_sec = rtc_datetime.second;
			rtc_time.tm_min = rtc_datetime.minute;
			rtc_time.tm_hour = rtc_datetime.hour;
			rtc_time.tm_mday = rtc_datetime.day;
			rtc_time.tm_mon = rtc_datetime.month;
			rtc_time.tm_year = rtc_datetime.year;
			ret = ds3232_set_time(&rtc_time);
			if(ret == 0)
				ret = EC_OK;
			else if(ret == 1)
				ret = EC_FAIL;
			break;
		case CMD_REQ_GET_RTC:
			ret = ds3232_read_time(&rtc_time);
			if(ret == 0)
			{
				rtc_datetime.second = rtc_time.tm_sec;
				rtc_datetime.minute = rtc_time.tm_min;
				rtc_datetime.hour = rtc_time.tm_hour;
				rtc_datetime.day = rtc_time.tm_mday;
				rtc_datetime.month = rtc_time.tm_mon;
				rtc_datetime.year = rtc_time.tm_year;
				memcpy(msg_ptr->uart_msg_data.msg_ptr, &rtc_datetime, sizeof(rtc_datetime));			
				ret = EC_OK;
			}
			else if(ret == 1)
			{
				ret = EC_FAIL;
			}
			break;
		case CMD_REQ_GET_MCU_VER:
			mcu_version.major = 1;
			mcu_version.minor = 0;
			memcpy(msg_ptr->uart_msg_data.msg_ptr, &mcu_version, sizeof(mcu_version));
			ret = EC_OK;
			break;
		case CMD_REQ_GET_SOC_POWER_NUM:
			memcpy(msg_ptr->uart_msg_data.msg_ptr, &global_soc_power_num, sizeof(global_soc_power_num));
			ret = EC_OK;
			break;

		case CMD_REP_BAT_DISCHARGE_STATE:
			memcpy(&global_discharge_state, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_discharge_state));
			mcu_timeout.soc_ack_flag = 1;
			ret = EC_OK;
			break;
		case CMD_REP_GET_BAT_INFO:
			memcpy(&global_battery_info, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_battery_info));
			ret = EC_OK;
			break;
		case CMD_REQ_MCU_CHARGE_CTRL:
			memcpy(&charge_state, msg_ptr->uart_msg_data.msg_ptr, sizeof(charge_state));
			ret = charge_switch_func(charge_state.state);
			break;
		case CMD_REP_MCU_LOG:
			memcpy(&global_mcu_log, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_mcu_log));
			ret = EC_OK;
			break;
		case CMD_REQ_UPGRADE_MCU:
			memcpy(&global_upgrade_mcu, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_upgrade_mcu));
			//擦除FLASH流程
			ret = erase_flash(APPLICATION_ADDRESS, 0x20000);
			if(ret != 0)
			{
				ret = EC_FAIL;
			}
			break;
		case CMD_REP_MCU_UPD_READY:
			memcpy(&global_soc_ack, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_soc_ack));
			enter_upgrade_flow = 1;
			break;
		case CMD_REQ_UPLOAD_MCU_FW:
			memcpy((void *)&mcu_pack.index, msg_ptr->uart_msg_data.msg_ptr, 2);
			last_pack = mcu_pack.index >> 15;
			pack_index = mcu_pack.index & 0x0F;
			msg_ptr->uart_msg_data.msg_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + 2;
			memcpy((void *)&mcu_pack.size, msg_ptr->uart_msg_data.msg_ptr, 2);
			msg_ptr->uart_msg_data.msg_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + 2;
			if(crc_fail_num == 1)
			{
				crc_fail_num = 0;
			}
			memcpy(upgrade_pack, msg_ptr->uart_msg_data.msg_ptr, mcu_pack.size);
			//写flash流程
			ret = flash_write_bytes(upgrade_pack, flashdestination + write_index, mcu_pack.size);
			if(ret != 0)
			{
				ret = EC_FAIL;
			}
			else
			{
				write_index += mcu_pack.size;
				update_percentage = write_index * 100 / global_upgrade_mcu.filesize;
				ret = EC_OK;
			}
			
			if(last_pack == 1)
			{
				if(write_index != global_upgrade_mcu.filesize)
				{
					ret = EC_FAIL;
				}
				else//跳转流程
				{
					HAL_Delay(100);
					bsp_update_jumptoapp_evt_cbk();
					ret = EC_OK;
				}
			}
			break;
		case CMD_REQ_HEARTBEAT:
			heartbeat_value.last_tick_value = HAL_GetTick();
			ret = EC_OK;
			break;
		case CMD_REQ_MCU_UPD_STATE:
			memcpy(msg_ptr->uart_msg_data.msg_ptr, &update_percentage, sizeof(update_percentage));
			ret = EC_OK;
			break;
		default:
			ret = EC_FAIL;
			break;
	}

	return ret;
}

uint8_t uart_msg_dispatch(uart_msg *msg_ptr, uint8_t crc_error)
{
	uint8_t ret;
	ret = mcu_action_flow_before_ack(msg_ptr);
	form_ack_uart_msg(msg_ptr, ret);
	return 0;
}

uint8_t uart_msg_proc_flow(uint8_t active_passive, uart_msg *active_msg)
{
	uint8_t recv_data;
	int32_t ret;
	uint16_t calc_crc;
	uart_msg uart_msg_instance = {0};
	battery_discharge_state discharge_state;
	uint8_t msg_total_len;
	int errcode;
	upgrade_state update_value;	
	fill_msg_flag msg_flag;
	
	short log_size;
	mcu_log *log_ptr;

	msg_flag.msg_output = uart_content;

	if(active_passive == 1)//mcu主动发送消息
	{
		switch(active_msg->msg_id)
		{
			case CMD_REQ_BAT_DISCHARGE_STATE:
				discharge_state.state = 1;
				msg_flag.msg_id = active_msg->msg_id;
				msg_flag.errcode = 0;
				msg_flag.msg_content = &discharge_state;
				msg_flag.msg_len = sizeof(discharge_state);
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, *(msg_flag.msg_total_len), 1000);
				break;
			case CMD_REQ_MCU_LOG:
				log_ptr = (mcu_log *)(active_msg->uart_msg_data.msg_ptr);
				log_size = log_ptr->size;
				memcpy(log_fill, active_msg->uart_msg_data.msg_ptr, sizeof(mcu_log));
				memcpy(log_fill + sizeof(mcu_log), actual_log_fill, log_size);
				msg_flag.msg_id = active_msg->msg_id;
				msg_flag.errcode = 0;
				msg_flag.msg_content = log_fill;
				msg_flag.msg_len = log_size;
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, *(msg_flag.msg_total_len), 1000);
				break;
			case CMD_REQ_GET_BAT_INFO:
				msg_flag.msg_id = active_msg->msg_id;
				msg_flag.errcode = 0;
				msg_flag.msg_len = 0;
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, *(msg_flag.msg_total_len), 1000);
				break;
			case CMD_REQ_SHUTDOWN_SOC:
				if(shutdown_state_value.req_msg_send == 0)
				{
					msg_flag.msg_id = active_msg->msg_id;
					msg_flag.errcode = 0;
					msg_flag.msg_len = sizeof(shutdown_soc);
					msg_flag.msg_content = active_msg->uart_msg_data.msg_ptr;
					fill_msg(&msg_flag);
					HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output , *(msg_flag.msg_total_len), 1000);
					shutdown_state_value.req_msg_send = 1;
					shutdown_state_value.curr_tick = HAL_GetTick();
				}
				break;
			case CMD_REQ_MCU_UPD_READY:
				msg_flag.msg_id = active_msg->msg_id;
				msg_flag.errcode = 0;
				msg_flag.msg_len = 0;
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, *(msg_flag.msg_total_len), 1000);
				break;
			default:
				break;
		}
	}
	else if(active_passive == 0)//mcu接收消息
	{
		ret = GetSingleData_CircularQueue(&Uart_Recv_Queue, &recv_data);
		if(ret == 0)
		{
			msg_content[msgpro.total_index] = recv_data;
			msgpro.total_index++;
			if((msgpro.total_index >= 2) && (msg_content[0] == 0xAA) && (msg_content[1] == 0xAA))
			{
				if(msgpro.total_index >= 10)
				{
					if(msg_content[msgpro.total_index-1] == 0x0A && msg_content[msgpro.total_index -2] == 0x0D)
					{
						uart_msg_instance.msg_len = msg_content[4] << 8 | msg_content[5];
						if(uart_msg_instance.msg_len == msgpro.total_index - 10)
						{
							uart_msg_instance.msg_id = msg_content[2] << 8 | msg_content[3];
							memcpy(&uart_msg_instance.uart_msg_data.msg_ptr, &msg_content[6], uart_msg_instance.msg_len);
							uart_msg_instance.msg_crc = msg_content[6 + uart_msg_instance.msg_len] << 8 | msg_content[7 + uart_msg_instance.msg_len];
							CRC16_PAR(msg_content, uart_msg_instance.msg_len + 6, &calc_crc);
							uart_msg_instance.msg_tail = msg_content[8 + uart_msg_instance.msg_len] << 8 | msg_content[9 + uart_msg_instance.msg_len];
							if(calc_crc == uart_msg_instance.msg_crc)
							{
								uart_msg_dispatch(&uart_msg_instance, 0);
								memset(&msgpro, 0, sizeof(msgpro));
								return 0;				
							}
							else if(calc_crc != uart_msg_instance.msg_crc && enter_upgrade_flow == 1)
							{
								crc_fail_num++;
								if(crc_fail_num >= 5)
								{
									return 1;
								}

								memset(&uart_msg_instance, 0, sizeof(uart_msg_instance));
								uart_msg_instance.msg_id = CMD_REQ_UPLOAD_MCU_FW;
								errcode = EC_CRC;
								form_ack_uart_msg(&uart_msg_instance, errcode);
							}
							else if(calc_crc != uart_msg_instance.msg_crc)
							{
								uart_msg_dispatch(&uart_msg_instance, 1);
							}
						}
					}
				}
			}
		}
	}

	
	return 1;
}


