#include "uart_prot.h"
#include "CircularQueue.h"
#include "crc16.h"
#include "ds3232.h"
#include "interflash.h"
#include "sw6301.h"
#include "bsp_hwdg.h"
#include <string.h>

CircularQueue_Str Uart_Recv_Queue;
uint8_t Uart_Recv_Fifo[UART_MSG_LEN];
extern UART_HandleTypeDef hlpuart1;

common_rep global_discharge_state;
uint8_t msg_content[UART_MSG_LEN] = {0};
msg_proc msgpro = {0};
static uint8_t uart_content[UART_MSG_LEN];
common_rep global_mcu_log;
soc_sys_state soc_sys_state_ins;
mcu_version mcu_version_ins = {0, 1};
battery_info global_battery_info;
mcu_self_check mcu_self_check_ins;
soc_power_num global_soc_power_num;
uint8_t log_fill[530];
uint8_t actual_log_fill[512];
uint8_t upgrade_pack[1024] MEM_ALIGNED(8);
req_upgrade_mcu global_upgrade_mcu = {0};
common_rep global_soc_ack = {0};
mcu_upgrade_pack mcu_pack = {0};
uint16_t last_pack_index = 0;
uint8_t enter_upgrade_flow = 0;
uint8_t crc_fail_num = 0;
uint8_t update_percentage = 0;
heartbeat_pro heartbeat_value = {0};
shutdown_soc shutdown_value = {0};
shutdown_state shutdown_state_value = {0};
charge_state chargestate = {0};
mcu_req_timeout mcu_timeout = {0};
battery_state global_battery_state = {0};
uint8_t notify_upgrade_flag = 0;
uart_msg uart_msg_instance = {0};


typedef void (*pFunction)(void);

uint32_t flashdestination = APPLICATION_ADDRESS;
uint32_t write_index = 0;
uint32_t JumpAddress;
pFunction Jump_To_Application;

uint16_t data_pack_num = 0;
uint8_t enter_last_flow = 0;

extern uint16_t match_succ;

#define MSG_FIXED_SIZE	sizeof(common_rep)	

enum cmd_id_t cmd_id_array[34] =
{
	//CMD_MIN,//1
	CMD_REQ_UPGRADE_MCU,//2
	CMD_REP_UPGRADE_MCU,//3
	CMD_REQ_MCU_UPD_READY,//4
	CMD_REP_MCU_UPD_READY,//5
	CMD_REQ_UPLOAD_MCU_FW,//6
	CMD_REP_UPLOAD_MCU_FW,//7
	CMD_REQ_MCU_UPD_STATE,//8
	CMD_REP_MCU_UPD_STATE,//9
	CMD_REQ_SHUTDOWN_SOC,//10
	CMD_REP_SHUTDOWN_SOC,//11
	CMD_REQ_SET_RTC,//12
	CMD_REP_SET_RTC,//13
	CMD_REQ_MCU_SELF_CHECK,//14
	CMD_REP_MCU_SELF_CHECK,//15
	CMD_REQ_MCU_CHARGE_CTRL,//16
	CMD_REP_MCU_CHARGE_CTRL,//17
	CMD_REQ_GET_MCU_VER,//18
	CMD_REP_GET_MCU_VER,//19
	CMD_REQ_GET_SOC_POWER_NUM,//20
	CMD_REP_GET_SOC_POWER_NUM,//21
	CMD_REQ_GET_RTC,//22
	CMD_REP_GET_RTC,//23
	//CMD_REQ_GET_MCU_GPIO_STATE	= MK_CMDID(3,7),
	//CMD_REP_GET_MCU_GPIO_STATE = MK_CMDID(3,8),
	CMD_REQ_GET_BAT_INFO,//24
	CMD_REP_GET_BAT_INFO,//25
	CMD_REQ_HEARTBEAT,//26
	CMD_REP_HEARTBEAT,//27
	CMD_REQ_MCU_LOG,//28
	CMD_REP_MCU_LOG,//29
	CMD_REQ_SOC_STATE,//30
	CMD_REP_SOC_STATE,//31
	CMD_REQ_BAT_CHARGE_STATE,//32
	CMD_REP_BAT_CHARGE_STATE,//33
	CMD_REQ_MCU_SELF_CHECK_RES,//34
	CMD_REP_MCU_SELF_CHECK_RES,//35
};


uint8_t msgid_match_flow(uint16_t msgid)
{
	uint16_t i;
	for(i = 0; i < sizeof(cmd_id_array) / sizeof(cmd_id_array[0]); i++)
	{
		if(msgid == cmd_id_array[i])
			return 1;
	}

	return 0;
}

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
			global_soc_power_num.off_num += 1;
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
			global_soc_power_num.off_num += 1;
		}

		if(HAL_GetTick() - shutdown_state_value.curr_tick >= 5000)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);//开机
			memset(&shutdown_state_value, 0, sizeof(shutdown_state_value));
			global_soc_power_num.on_num += 1;
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

void fill_msg(fill_msg_flag *msg_flag)
{
	common_rep rep_value;
	uint8_t temp;
	uint16_t calc_crc;
	uint16_t fixed_length;
	msg_flag->msg_output[0] = 0xAA;
	msg_flag->msg_output[1] = 0xAA;
	temp = (msg_flag->msg_id & 0xFF00) >> 8;
	memcpy(&msg_flag->msg_output[2], &temp, 1);
	temp = msg_flag->msg_id & 0xFF;
	memcpy(&msg_flag->msg_output[3], &temp, 1);
	
	if(msg_flag->errcode != EC_OK || msg_flag->msg_len == 0)
	{
		fixed_length = MSG_FIXED_SIZE;
		temp = (fixed_length & 0xFF00) >> 8;
		memcpy(&msg_flag->msg_output[4], &temp, 1);
		temp = fixed_length & 0x00FF;
		memcpy(&msg_flag->msg_output[5], &temp, 1);
		rep_value.errcode = msg_flag->errcode;
		memcpy(&msg_flag->msg_output[6], &rep_value, sizeof(rep_value));
		calc_crc = crc16(0, msg_flag->msg_output, 6 + MSG_FIXED_SIZE);		
		temp = (calc_crc & 0xFF00) >> 8;
		memcpy(&msg_flag->msg_output[6 + MSG_FIXED_SIZE], &temp, 1);
		temp = calc_crc & 0x00FF;
		memcpy(&msg_flag->msg_output[7 + MSG_FIXED_SIZE], &temp, 1);
		msg_flag->msg_output[8 + MSG_FIXED_SIZE] = 0x0D;
		msg_flag->msg_output[9 + MSG_FIXED_SIZE] = 0x0A;
		msg_flag->msg_total_len = 10 + MSG_FIXED_SIZE;

	}
	else
	{
		temp = ((msg_flag->msg_len + MSG_FIXED_SIZE) & 0xFF00) >> 8;;
		memcpy(&msg_flag->msg_output[4], &temp, 1);
		temp = (msg_flag->msg_len + MSG_FIXED_SIZE) & 0x00FF;
		memcpy(&msg_flag->msg_output[5], &temp, 1);
		rep_value.errcode = EC_OK;
		memcpy(&msg_flag->msg_output[6], &rep_value, sizeof(rep_value));
		memcpy(&msg_flag->msg_output[6 + sizeof(rep_value)], msg_flag->msg_content, msg_flag->msg_len);
		calc_crc = crc16(0, msg_flag->msg_output, 6 + msg_flag->msg_len + sizeof(rep_value));
		temp = (calc_crc & 0xFF00) >> 8;
		memcpy(&msg_flag->msg_output[6 + msg_flag->msg_len + sizeof(rep_value)], &temp, 1);
		temp = calc_crc & 0x00FF;
		memcpy(&msg_flag->msg_output[7 + msg_flag->msg_len + sizeof(rep_value)], &temp, 1);
		msg_flag->msg_output[8 + msg_flag->msg_len + sizeof(rep_value)] = 0x0D;
		msg_flag->msg_output[9 + msg_flag->msg_len + sizeof(rep_value)] = 0x0A;
		msg_flag->msg_total_len = 10 + msg_flag->msg_len + sizeof(rep_value);	
	}
}


extern uint16_t heart_beat;
extern uint16_t version_beat;
extern uint16_t get_rtc_beat;
extern uint16_t test_soc_system_beat;
extern uint16_t soc_state_beat;
extern uint16_t set_rtc_beat;


void form_ack_uart_msg(uart_msg *msg_ptr, int8_t errcode)
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
			heart_beat++;
			break;
		case CMD_REQ_SOC_STATE://fixed_msg
			msg_flag.msg_id = CMD_REP_SOC_STATE;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			soc_state_beat++;
			break;
		case CMD_REQ_GET_MCU_VER:
			msg_flag.msg_id = CMD_REP_GET_MCU_VER;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(mcu_version);
			fill_msg(&msg_flag);
			version_beat++;
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
			msg_flag.msg_len = sizeof(struct rtc_datetime);
			fill_msg(&msg_flag);
			get_rtc_beat++;
			break;
		case CMD_REQ_SET_RTC://fixed_msg
			msg_flag.msg_id = CMD_REP_SET_RTC;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			set_rtc_beat++;
			break;
		case CMD_REQ_MCU_CHARGE_CTRL://fixed_msg
			msg_flag.msg_id = CMD_REP_MCU_CHARGE_CTRL;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_UPGRADE_MCU://fixed_msg
			msg_flag.msg_id = CMD_REP_UPGRADE_MCU;
			msg_flag.msg_len = 0;
			if(msg_flag.errcode == EC_OK)
			{
				notify_upgrade_flag	= 1;
			}
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
		case CMD_REQ_BAT_CHARGE_STATE://fixed_msg
			msg_flag.msg_id = CMD_REP_BAT_CHARGE_STATE;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		case CMD_REQ_MCU_SELF_CHECK://fixed_msg
			msg_flag.msg_id = CMD_REP_MCU_SELF_CHECK;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			break;
		default:
			fill_msg(&msg_flag);
			msg_flag.msg_total_len = 0;
			break;
	}

	HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , 1000);
}

uint8_t API_UART_RxFifoEnable(uint8_t *rx_data, uint16_t length)
{
	if(rx_data == NULL || length == 0)
		return 1;
	Init_CircularQueue(&Uart_Recv_Queue, rx_data, length, sizeof(uint8_t));
	return 0;
}

int board_slef_check_func(mcu_self_check self_check_value)
{
	int ret = EC_OK;
	if(self_check_value.mod == MCU_MOD_ALL)
	{
		
	}
	else if(self_check_value.mod == MCU_MOD_FLASH)
	{

	}
	else if(self_check_value.mod == MCU_MOD_BATTERY)
	{

	}

	return ret;
}

uint8_t mcu_action_flow_before_ack(uart_msg *msg_ptr)
{
	uint16_t msg_id;
	int ret;
	rtc_datetime rtc_datetime = {0};
	struct rtc_time rtc_time = {0};
	msg_id = msg_ptr->msg_id;
	battery_charge_state charge_state;
	uint8_t last_pack;
	uart_msg jump_uart_msg;
	uint16_t pack_index;
	mcu_self_check self_check_value;
	
	switch(msg_id)
	{
		case CMD_REQ_SOC_STATE:
			memcpy(&soc_sys_state_ins, msg_ptr->uart_msg_data.msg_ptr, sizeof(soc_sys_state_ins));
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
			memcpy(msg_ptr->uart_msg_data.msg_ptr, &mcu_version_ins, sizeof(mcu_version_ins));
			ret = EC_OK;
			break;
		case CMD_REQ_GET_SOC_POWER_NUM:
			memcpy(msg_ptr->uart_msg_data.msg_ptr, &global_soc_power_num, sizeof(global_soc_power_num));
			ret = EC_OK;
			break;
		case CMD_REP_GET_BAT_INFO:
			memcpy(&global_battery_info, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_battery_info));
			ret = EC_OK;
			mcu_timeout.soc_ack_get_bat_info_flag = 1;
			break;
		case CMD_REQ_MCU_CHARGE_CTRL:
			memcpy(&charge_state, msg_ptr->uart_msg_data.msg_ptr, sizeof(charge_state));
			ret = charge_switch_func(charge_state.state);
			if(ret == 0)
			{
				ret = EC_OK;
			}
			else
			{
				ret = EC_FAIL;
			}
			break;
		case CMD_REP_MCU_LOG:
			memcpy(&global_mcu_log, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_mcu_log));
			ret = EC_OK;
			mcu_timeout.soc_ack_log_flag = 1;
			break;
		case CMD_REQ_UPGRADE_MCU:
			memcpy(&global_upgrade_mcu, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_upgrade_mcu));
			ret = EC_OK;
			break;
		case CMD_REP_MCU_UPD_READY:
			memcpy(&global_soc_ack, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_soc_ack));

			mcu_timeout.soc_ack_udp_ready_flag = 1;
			enter_upgrade_flow = 1;
			break;
		case CMD_REP_SHUTDOWN_SOC:
			mcu_timeout.soc_ack_shutdown_info_flag = 1;
			break;
		case CMD_REQ_UPLOAD_MCU_FW:
			data_pack_num++;
			memcpy((void *)&mcu_pack.index, msg_ptr->uart_msg_data.msg_ptr, 2);
			last_pack = mcu_pack.index >> 15;
			pack_index = mcu_pack.index & 0x7FFF;
			if((pack_index != (last_pack_index + 1)) && (pack_index != 0))
			{
				ret = EC_FAIL;
			}
			msg_ptr->uart_msg_data.msg_ptr = (uint8_t *)(msg_ptr->uart_msg_data.msg_ptr) + 2;
			memcpy((void *)&mcu_pack.size, msg_ptr->uart_msg_data.msg_ptr, 2);
			msg_ptr->uart_msg_data.msg_ptr = (uint8_t *)(msg_ptr->uart_msg_data.msg_ptr) + 2;
			if(crc_fail_num != 0)
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
				last_pack_index = pack_index;
				write_index += mcu_pack.size;
				update_percentage = write_index * 100 / global_upgrade_mcu.filesize;
				ret = EC_OK;
			}
			
			if(last_pack == 1)
			{
				if(write_index != global_upgrade_mcu.filesize)
				{
					enter_last_flow = 0xff;
					ret = EC_FAIL;
				}
				else//跳转流程
				{
					enter_last_flow = 1;
					memset(&jump_uart_msg, 0, sizeof(jump_uart_msg));					
					ret = EC_OK;
					jump_uart_msg.msg_id = CMD_REQ_UPLOAD_MCU_FW;
					form_ack_uart_msg(&jump_uart_msg, ret);					
					HAL_Delay(100);
					bsp_update_jumptoapp_evt_cbk();
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
		case CMD_REQ_BAT_CHARGE_STATE:
			memcpy(&global_battery_state, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_battery_state));
			ret = EC_OK;
			break;
		case CMD_REQ_MCU_SELF_CHECK:
			memcpy(&self_check_value, msg_ptr->uart_msg_data.msg_ptr, sizeof(self_check_value));
			ret = board_slef_check_func(self_check_value);
			if(ret != 0)
			{
				ret = EC_FAIL;
			}
			else
			{
				ret = EC_OK;
			}
			break;
		default:
			ret = EC_FAIL;
			break;
	}

	return ret;
}

uint8_t uart_msg_dispatch(uart_msg *msg_ptr)
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
//	battery_discharge_state discharge_state;
	int errcode;
	fill_msg_flag msg_flag;
	uint8_t msg_tail_hi;
	uint8_t msg_tail_lo;
	short log_size;
	mcu_log *log_ptr;

	msg_flag.msg_output = uart_content;

	if(active_passive == 1)//mcu主动发送消息
	{
		switch(active_msg->msg_id)
		{
			/*case CMD_REQ_BAT_DISCHARGE_STATE:
				discharge_state.state = 1;
				msg_flag.msg_id = active_msg->msg_id;
				msg_flag.errcode = 0;
				msg_flag.msg_content = &discharge_state;
				msg_flag.msg_len = sizeof(discharge_state);
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, 1000);
				mcu_timeout.mcu_req_bat_discharge_tick = HAL_GetTick();
				mcu_timeout.mcu_req_bat_discharge_flag = 1;
				break;*/
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
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, 1000);
				API_WatchDog_FeedDog();
				mcu_timeout.mcu_req_log_tick = HAL_GetTick();
				mcu_timeout.mcu_req_log_flag = 1;
				break;
			case CMD_REQ_GET_BAT_INFO:
				msg_flag.msg_id = active_msg->msg_id;
				msg_flag.errcode = 0;
				msg_flag.msg_len = 0;
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, 1000);
				mcu_timeout.mcu_req_get_bat_info_tick = HAL_GetTick();
				mcu_timeout.mcu_req_get_bat_info_flag = 1;
				break;
			case CMD_REQ_SHUTDOWN_SOC:
				if(shutdown_state_value.req_msg_send == 0)
				{
					msg_flag.msg_id = active_msg->msg_id;
					msg_flag.errcode = 0;
					msg_flag.msg_len = sizeof(shutdown_soc);
					msg_flag.msg_content = active_msg->uart_msg_data.msg_ptr;
					fill_msg(&msg_flag);
					HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output , msg_flag.msg_total_len, 1000);
					shutdown_state_value.req_msg_send = 1;
					shutdown_state_value.curr_tick = HAL_GetTick();
					mcu_timeout.mcu_req_shutdown_soc_tick = HAL_GetTick();
					mcu_timeout.mcu_req_shutdown_soc_flag = 1;					
				}
				break;
			case CMD_REQ_MCU_UPD_READY:
				msg_flag.msg_id = active_msg->msg_id;
				API_WatchDog_FeedDog();
		//擦除FLASH流程
				ret = erase_flash(APPLICATION_ADDRESS, 0x20000);
				if(ret != 0)
				{
					msg_flag.errcode = EC_FAIL;
				}
				else
				{
					msg_flag.errcode = EC_OK;
				}
				
				API_WatchDog_FeedDog();
				msg_flag.msg_len = 0;
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, 1000);
				mcu_timeout.mcu_req_udp_ready_tick = HAL_GetTick();
				mcu_timeout.mcu_req_udp_ready_flag = 1;
				break;
			default:
				break;
		}	
	}
	else if(active_passive == 0)//mcu接收消息
	{
#if 1
	ret = GetSingleData_CircularQueue(&Uart_Recv_Queue, &recv_data);
	if(ret == 0)
	{
		msg_content[msgpro.total_index] = recv_data;
		msgpro.total_index++;
		if((msgpro.total_index >= 2) && (msg_content[0] == 0xAA) && (msg_content[1] == 0xAA))
		{
			if(msgpro.total_index >= 6)
			{
				if(msgpro.pre_very == 0)
				{
					uart_msg_instance.msg_len = msg_content[4] << 8 | msg_content[5];
					uart_msg_instance.msg_id = msg_content[2] << 8 | msg_content[3];
					ret = msgid_match_flow(uart_msg_instance.msg_id);

					if(ret == 0 || uart_msg_instance.msg_len > 1024)
					{
						memmove(msg_content, &msg_content[2], 4);
						msgpro.total_index -= 2;
						msgpro.pre_very = 0;
					}
					else
					{
						msgpro.pre_very = 1;
					}					
				}
				
				if(msgpro.total_index >= (10 + uart_msg_instance.msg_len))
				{
					uart_msg_instance.msg_crc = msg_content[6 + uart_msg_instance.msg_len] << 8 | msg_content[7 + uart_msg_instance.msg_len];
					calc_crc = crc16(0, msg_content, uart_msg_instance.msg_len + 6);
					msg_tail_hi = msg_content[msgpro.total_index - 2];
					msg_tail_lo = msg_content[msgpro.total_index - 1];
					if((calc_crc == uart_msg_instance.msg_crc) && (msg_tail_hi == 0x0D) && (msg_tail_lo == 0x0A))
					{
						uart_msg_instance.uart_msg_data.msg_ptr = &msg_content[6];
						uart_msg_dispatch(&uart_msg_instance);
						match_succ++;
						msgpro.total_index = 0;
						msgpro.pre_very = 0;
					}
					else
					{	
						memmove(msg_content, &msg_content[2], 8 + uart_msg_instance.msg_len);
						msgpro.total_index -= 2;
						msgpro.pre_very = 0;
					}
				}
			}
		}
		else if((msgpro.total_index >= 2) && (msg_content[1] != 0xAA))
		{
			msgpro.total_index -= 2;
			memmove(msg_content, &msg_content[2], msgpro.total_index);
			msgpro.pre_very = 0;
		}
		else if(msgpro.total_index >= 2)
		{
			msgpro.total_index -= 1;
			memmove(msg_content, &msg_content[1], msgpro.total_index);
			msgpro.pre_very = 0;
		}
	}


#else
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

							if(uart_msg_instance.msg_len > 1024)
							{
								errcode = EC_INVAL;
								form_ack_uart_msg(&uart_msg_instance, errcode);
								return 1;
							}
							//memcpy(&uart_msg_instance.uart_msg_data.msg_ptr, &msg_content[6], uart_msg_instance.msg_len);
							uart_msg_instance.uart_msg_data.msg_ptr = &msg_content[6];
							uart_msg_instance.msg_crc = msg_content[6 + uart_msg_instance.msg_len] << 8 | msg_content[7 + uart_msg_instance.msg_len];
							//CRC16_PAR(msg_content, uart_msg_instance.msg_len + 6, &calc_crc);
							calc_crc = crc16(0, msg_content, uart_msg_instance.msg_len + 6);
							//uart_msg_instance.msg_tail = msg_content[8 + uart_msg_instance.msg_len] << 8 | msg_content[9 + uart_msg_instance.msg_len];
							if(calc_crc == uart_msg_instance.msg_crc)
							{
								uart_msg_dispatch(&uart_msg_instance);
								//extern uint16_t match_succ;
								//match_succ++;
								msgpro.total_index = 0;
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
								errcode = EC_CRC;
								form_ack_uart_msg(&uart_msg_instance, errcode);
							}

							memset(&msgpro, 0, sizeof(msgpro));
						}
						//if(msgpro.total_index >= 1038)
						//{
						//	msgpro.total_index = 0;
						//}
						else
						{
							memset(&msgpro, 0, sizeof(msgpro));
							memset(msg_content, 0, sizeof(msg_content));
						}
					}
				}
			}
			else if((msgpro.total_index >= 2) && ((msg_content[0] != 0xAA) || (msg_content[1] != 0xAA)))
			{
				msgpro.total_index = 0;
			}
		}
#endif		
	}

	
	return 1;
}


