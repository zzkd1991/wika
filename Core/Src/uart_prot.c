#include "uart_prot.h"
#include "CircularQueue.h"
#include "crc16.h"
#include "ds3232.h"
#include "interflash.h"
#include "sw6301.h"
#include "bsp_hwdg.h"
#include "interflash_test.h"
#include "button.h"
#include "md5.h"
#include <string.h>

CircularQueue_Str Uart_Recv_Queue;
uint8_t Uart_Recv_Fifo[UART_MSG_LEN];
extern UART_HandleTypeDef hlpuart1;

common_rep global_discharge_state;
uint8_t msg_content[UART_MSG_LEN] = {0};
static msg_proc msgpro = {0};
static uint8_t uart_content[UART_MSG_LEN];
static common_rep global_mcu_log;
static common_rep global_selfcheck_ret;
static mcu_version mcu_version_ins = {0, 1};
static battery_info global_battery_info;
mcu_self_check_ins self_check_ins = {0};
soc_power_num global_soc_power_num;
uint8_t log_fill[530];
uint8_t actual_log_fill[512];
uint8_t upgrade_pack[1024] MEM_ALIGNED(8);
req_upgrade_mcu global_upgrade_mcu = {0};
common_rep global_soc_ack = {0};
mcu_upgrade_pack mcu_pack = {0};
uint16_t last_pack_index = 0;
uint8_t update_percentage = 0;
heartbeat_pro heartbeat_value = {0};
shutdown_state shutdown_state_value = {0};
mcu_req_timeout mcu_timeout = {0};
battery_state global_battery_state = {0};
uint8_t notify_upgrade_flag = 0;
uart_msg uart_msg_instance = {0};
uint32_t soc_shutdown_tick;
uint8_t soc_shutdown_flag = 0;
uint8_t soc_shutdown_heartbeat = 0;
uint16_t myheartbeat_timeout_cnt = 0;
uint8_t switch_en_flag_12v = 0;
uint8_t ab_system_arr[512]MEM_ALIGNED(8) = {0};
bin_file_info binfile = {0};



typedef void (*pFunction)(void);

uint32_t flashdestination;
uint32_t write_index = 0;
uint32_t JumpAddress;
pFunction Jump_To_Application;
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

int is_little_endian(void)
{
	uint32_t val = 0x01020304;
	uint8_t *p = (uint8_t *)&val;
	return (*p == 0x04);
}

void swap_endian(uint8_t *data, size_t size)
{
	size_t i;
	for(i = 0; i < size / 2; i++)
	{
		uint8_t temp = data[i];
		data[i] = data[size - 1 - i];
		data[size - 1 - i] = temp;
	}
}

void endian_conved_func(void *value, uint8_t type)
{
	if(type == 0)
	{
		swap_endian((uint8_t *)value, sizeof(uint16_t));
	}
	else if(type == 1)
	{
		swap_endian((uint8_t *)value, sizeof(uint32_t));
	}
}


void turnoff_soc_func(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);//关机
}

void turnon_soc_func(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void heartbeat_timeout_func(void)
{
	if(soc_shutdown_heartbeat == 1)
		return;
	if(HAL_GetTick() - heartbeat_value.last_tick_value >= 9000)
	{
		myheartbeat_timeout_cnt++;
		heartbeat_value.last_tick_value = HAL_GetTick();
		//断电/上电重启开发板
		turnoff_soc_func();
		global_soc_power_num.off_num += 1;
		HAL_Delay(50);
		turnon_soc_func();
		global_soc_power_num.on_num += 1;
	}
}

void shutdown_func_from_soc(void)
{
	if(soc_shutdown_flag == 1)
	{
		if(HAL_GetTick() - soc_shutdown_tick >= 3000)
		{
			soc_shutdown_flag = 0;
			turnoff_soc_func();
		}
	}
}

static void shutdown_func(uint8_t shutdown)//按键检测函数中调用
{
	uart_msg msg_inst;
	shutdown_soc shutdown_value;	
	if(shutdown == 1)
	{
		shutdown_value.shutdown = 1;
		msg_inst.msg_id = CMD_REQ_SHUTDOWN_SOC;
		msg_inst.uart_msg_data.msg_ptr = &shutdown_value;
		mcu_send_msg_flow(&msg_inst);
		if(HAL_GetTick() - shutdown_state_value.curr_tick >= 3000)
		{
			//关机
			turnoff_soc_func();
			memset(&shutdown_value, 0, sizeof(shutdown_value));
			global_soc_power_num.off_num += 1;
		}
	}
	else if(shutdown == 2)
	{
		shutdown_value.shutdown = 2;
		msg_inst.msg_id = CMD_REQ_SHUTDOWN_SOC;
		msg_inst.uart_msg_data.msg_ptr = &shutdown_value;
		if(shutdown_state_value.req_msg_send )
		mcu_send_msg_flow(&msg_inst);
		if(HAL_GetTick() - shutdown_state_value.curr_tick >= 3000)
		{
			turnoff_soc_func();
			shutdown_state_value.curr_tick = HAL_GetTick();
			global_soc_power_num.off_num += 1;
		}

		if(HAL_GetTick() - shutdown_state_value.curr_tick >= 5000)
		{
			turnon_soc_func();
			shutdown_state_value.curr_tick = HAL_GetTick();
			memset(&shutdown_state_value, 0, sizeof(shutdown_state_value));
			global_soc_power_num.on_num += 1;
		}
	}
}

void shutdown_func_from_button(void)
{
	if(Key_Scan() == 1)
	{
		shutdown_func(1);
	}	
}

void power_manager_func(void)
{
	if(soc_shutdown_heartbeat == 1)
		return;
	if(REAL_VALUE < 18)
	{
		turnon_soc_func();
		switch_en_flag_12v = 1;
	}

	if(switch_en_flag_12v == 1)
	{
		if(REAL_VALUE > 20)
		{
			turnoff_soc_func();
		}
		switch_en_flag_12v = 0;
	}
}

void no_charge_func(void)
{
	if(REAL_VALUE >= 18 && REAL_VALUE <= 20)
	{
		charge_switch_func(0);
	}

}

void file_md5_func(uint32_t flashaddr, uint8_t *md5value)
{
	uint32_t flashaddr_tmp;
	flashaddr_tmp = flashaddr;
	char tmp_value[MY_MD5_READ_DATA_SIZE];
	char md5_value[MY_MD5_SIZE];
	while(global_upgrade_mcu.filesize > 0)
	{
		FLASH_If_Read(flashaddr_tmp, (uint8_t *)tmp_value, MY_MD5_READ_DATA_SIZE);
		Compute_file_md5(tmp_value, md5_value, MY_MD5_READ_DATA_SIZE);
		flashaddr_tmp += MY_MD5_READ_DATA_SIZE;
		global_upgrade_mcu.filesize -= MY_MD5_READ_DATA_SIZE;
	}

	memcpy(md5value, md5_value, MY_MD5_SIZE);
}

void PreJumpToApplication(void)
{
	if(binfile.curr_partition == 1)//当前在A系统，向B系统跳转
	{
		erase_flash(AB_SYSTEM_FLAG_ADDRESS, FLASH_PAGE_SIZE);
		binfile.curr_partition = 0;
		binfile.file_size = global_upgrade_mcu.filesize;
		file_md5_func(B_SYSTEM_APPLICATION_ADDRESS, binfile.md5_value);
		memcpy(ab_system_arr, &binfile, sizeof(binfile));
		flash_write_bytes(ab_system_arr, AB_SYSTEM_FLAG_ADDRESS, sizeof(ab_system_arr));
	}
	else if(binfile.curr_partition == 0)//当前在B系统，向A系统跳转
	{
		erase_flash(AB_SYSTEM_FLAG_ADDRESS, FLASH_PAGE_SIZE);
		binfile.curr_partition = 1;
		binfile.file_size = global_upgrade_mcu.filesize;
		file_md5_func(A_SYSTEM_APPLICATION_ADDRESS, binfile.md5_value);
		memcpy(ab_system_arr, &binfile, sizeof(binfile));
		flash_write_bytes(ab_system_arr, AB_SYSTEM_FLAG_ADDRESS, sizeof(ab_system_arr));
	}
}

void JumpToApplication(void)
{
	NVIC_SystemReset();
}

void bsp_update_jumptoapp_evt_cbk(void)
{
	int i;
	PreJumpToApplication();
	HAL_DeInit();
	__disable_irq();
	for(i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}
	HAL_RCC_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
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
		if(is_little_endian())
		{
			endian_conved_func((void *)&rep_value.errcode, 0);
		}
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
		if(is_little_endian())
		{
			endian_conved_func((void *)&rep_value.errcode, 0);
		}
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
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_SOC_STATE://fixed_msg
			msg_flag.msg_id = CMD_REP_SOC_STATE;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_GET_MCU_VER:
			msg_flag.msg_id = CMD_REP_GET_MCU_VER;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(mcu_version);
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_GET_SOC_POWER_NUM:
			msg_flag.msg_id = CMD_REP_GET_SOC_POWER_NUM;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(global_soc_power_num);
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_GET_RTC:
			msg_flag.msg_id = CMD_REP_GET_RTC;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(struct rtc_datetime);
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_SET_RTC://fixed_msg
			msg_flag.msg_id = CMD_REP_SET_RTC;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
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
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_UPLOAD_MCU_FW://fixed_msg
			msg_flag.msg_id = CMD_REP_UPLOAD_MCU_FW;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_MCU_UPD_STATE:
			msg_flag.msg_id = CMD_REP_MCU_UPD_STATE;
			msg_flag.msg_content = msg_ptr->uart_msg_data.msg_ptr;
			msg_flag.msg_len = sizeof(upgrade_state);
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_BAT_CHARGE_STATE://fixed_msg
			msg_flag.msg_id = CMD_REP_BAT_CHARGE_STATE;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_MCU_SELF_CHECK://fixed_msg
			msg_flag.msg_id = CMD_REP_MCU_SELF_CHECK;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len , UART_SEND_TIMEOUT_LENGTH);
			break;
		default:
			//fill_msg(&msg_flag);
			//msg_flag.msg_total_len = 0;
			break;
	}
}

uint8_t API_UART_RxFifoEnable(uint8_t *rx_data, uint16_t length)
{
	if(rx_data == NULL || length == 0)
		return 1;
	Init_CircularQueue(&Uart_Recv_Queue, rx_data, length, sizeof(uint8_t));
	return 0;
}

void board_self_check_func(mcu_self_check_ins self_check_value, mcu_self_check_ret *self_check_ret)
{
	int ret = EC_OK;	
 	chip_version version = {0};
	memset((void *)&self_check_ret->result, 0, sizeof(self_check_ret->result));
	if(self_check_value.inst.mod == MCU_MOD_ALL)
	{
		ret = interflash_test();
		if(ret != 0)
		{
			self_check_ret->result |= MCU_MOD_FLASH;
		}

		
	 	ret = get_chip_version(&version);
		if(!((ret == 0) && (version.verion == 0x02)))
		{
			self_check_ret->result |= MCU_MOD_BATTERY;
		}

		self_check_ret->mod |= (MCU_MOD_FLASH | MCU_MOD_BATTERY);
	}
	else if(self_check_value.inst.mod == MCU_MOD_FLASH)
	{
		ret = interflash_test();
		if(ret != 0)
		{
			self_check_ret->result |= MCU_MOD_FLASH;
		}
		self_check_ret->mod = MCU_MOD_FLASH;
	}
	else if(self_check_value.inst.mod == MCU_MOD_BATTERY)
	{
		
	 	ret = get_chip_version(&version);
		if(!((ret == 0) && (version.verion == 0x02)))
		{
			self_check_ret->result |= MCU_MOD_BATTERY;
		}
		self_check_ret->mod = MCU_MOD_BATTERY;
	}
}


void self_check_pro_flow(void)
{
	uart_msg uart_msg_temp;
	mcu_self_check_ret self_check_ret_ins;
	if(self_check_ins.self_check_flow_flag == 1)
	{
		uart_msg_temp.msg_id = CMD_REQ_MCU_SELF_CHECK_RES;
		memset(&self_check_ret_ins, 0, sizeof(self_check_ret_ins));
		board_self_check_func(self_check_ins, &self_check_ret_ins);
		if(is_little_endian())
		{
			endian_conved_func((void *)&self_check_ret_ins.mod, 1);
			endian_conved_func((void *)&self_check_ret_ins.result, 1);
		}
		uart_msg_temp.uart_msg_data.msg_ptr = &self_check_ret_ins;
		uart_msg_temp.ret = EC_OK;
		mcu_send_msg_flow(&uart_msg_temp);
		self_check_ins.self_check_flow_flag = 0;
	}
}

void mcu_send_log_flow(mcu_log *log, char *log_content)
{
	uart_msg uart_msg_tmp;
	uart_msg_tmp.uart_msg_data.msg_ptr = log;
	uart_msg_tmp.msg_id = CMD_REQ_MCU_LOG;
	memcpy(actual_log_fill, log_content, log->size);
	mcu_send_msg_flow(&uart_msg_tmp);
}

void mcu_send_readymsg_func(void)
{
	uart_msg msg_inst;

	if(notify_upgrade_flag == 1)
	{	
		msg_inst.msg_id = CMD_REQ_MCU_UPD_READY;
		mcu_send_msg_flow(&msg_inst);
		notify_upgrade_flag = 0;
	}
	if(mcu_timeout.timeout_udp_ready_flag == 1)
	{
		msg_inst.msg_id = CMD_REQ_MCU_UPD_READY;
		mcu_send_msg_flow(&msg_inst);
		mcu_timeout.timeout_udp_ready_flag = 0;
	}
}

void get_battery_info_func(void)
{
	uart_msg uart_msg_tmp;
	static uint32_t cur_time = 3000;
	uart_msg_tmp.msg_id = CMD_REQ_GET_BAT_INFO;
	if(uwTick >= cur_time)
	{
		cur_time += 3000;
		mcu_send_msg_flow(&uart_msg_tmp);
	}
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
	soc_sys_state soc_sys_state_ins;	
	void *my_ptr;
	
	switch(msg_id)
	{
		case CMD_REQ_SOC_STATE:
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + MSG_FIXED_SIZE;
			memcpy(&soc_sys_state_ins, my_ptr, sizeof(soc_sys_state_ins));
			if(soc_sys_state_ins.state == 1)
			{
				global_soc_power_num.off_num +=1;
				soc_shutdown_tick = HAL_GetTick();
				soc_shutdown_flag = 1;
				soc_shutdown_heartbeat = 1;
			}
			else if(soc_sys_state_ins.state == 2)
			{
				global_soc_power_num.on_num += 1;
			}
			ret = EC_OK;
			break;
		case CMD_REQ_SET_RTC:
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + MSG_FIXED_SIZE;
			memcpy(&rtc_datetime, my_ptr, sizeof(rtc_datetime));
			if(is_little_endian())
			{
				endian_conved_func((void *)&rtc_datetime.year, 0);
			}
			rtc_time.tm_sec = rtc_datetime.second;
			rtc_time.tm_min = rtc_datetime.minute;
			rtc_time.tm_hour = rtc_datetime.hour;
			rtc_time.tm_mday = rtc_datetime.day;
			rtc_time.tm_mon = rtc_datetime.month;
			rtc_time.tm_year = rtc_datetime.year - 1900;
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
				rtc_datetime.year = rtc_time.tm_year + 1900;
				if(is_little_endian())
				{
					endian_conved_func((void *)&rtc_datetime.year, 0);
				}
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
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + MSG_FIXED_SIZE;			
			memcpy(&global_battery_info, my_ptr, sizeof(global_battery_info));
			ret = EC_OK;
			mcu_timeout.soc_ack_get_bat_info_flag = 1;
			break;
		case CMD_REQ_MCU_CHARGE_CTRL:
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + MSG_FIXED_SIZE;
			memcpy(&charge_state, my_ptr, sizeof(charge_state));
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
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr;
			memcpy(&global_mcu_log, my_ptr, sizeof(global_mcu_log));
			if(is_little_endian())
			{
				endian_conved_func((void *)&global_mcu_log.errcode, 0);
			}
			ret = EC_OK;
			mcu_timeout.soc_ack_log_flag = 1;
			break;
		case CMD_REQ_UPGRADE_MCU:
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + MSG_FIXED_SIZE;			
			memcpy(&global_upgrade_mcu, my_ptr, sizeof(global_upgrade_mcu));
			if(is_little_endian())
			{
				endian_conved_func((void *)&global_upgrade_mcu.filesize, 1);
				endian_conved_func((void *)&global_upgrade_mcu.minor, 1);
			}
			ret = EC_OK;
			break;
		case CMD_REP_MCU_UPD_READY:
			memcpy(&global_soc_ack, msg_ptr->uart_msg_data.msg_ptr, sizeof(global_soc_ack));
			if(is_little_endian())
			{
				endian_conved_func((void *)&global_soc_ack.errcode, 0);
			}
			mcu_timeout.soc_ack_udp_ready_flag = 1;
			break;
		case CMD_REP_SHUTDOWN_SOC:
			mcu_timeout.soc_ack_shutdown_info_flag = 1;
			break;
		case CMD_REQ_UPLOAD_MCU_FW:
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr + MSG_FIXED_SIZE;
			memcpy((void *)&mcu_pack.index, my_ptr, 2);
			if(is_little_endian())
			{
				endian_conved_func((void *)&mcu_pack.index, 0);
			}
			last_pack = mcu_pack.index >> 15;
			pack_index = mcu_pack.index & 0x7FFF;
			if((pack_index != (last_pack_index + 1)) && (pack_index != 0))
			{
				ret = EC_FAIL;
				return ret;
			}
			my_ptr = (uint8_t *)(msg_ptr->uart_msg_data.msg_ptr) + 4;
			memcpy((void *)&mcu_pack.size, my_ptr, 2);
			if(is_little_endian())
			{
				endian_conved_func((void *)&mcu_pack.size, 0);
			}
			my_ptr = (uint8_t *)(msg_ptr->uart_msg_data.msg_ptr) + 6;
			memcpy(upgrade_pack, my_ptr, mcu_pack.size);
			//写flash流程
			ret = flash_write_bytes(upgrade_pack, flashdestination + write_index, mcu_pack.size);
			if(ret != 0)
			{
				ret = EC_FAIL;
				return ret;
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
					ret = EC_FAIL;
				}
				else//跳转流程
				{
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
			if(is_little_endian())
			{
				endian_conved_func((void *)&self_check_value.mod, 1);
			}
			memcpy((void *)&self_check_ins.inst, (void *)&self_check_value, sizeof(self_check_value));
			self_check_ins.self_check_flow_flag = 1;			
			ret = EC_OK;
			break;
		case CMD_REP_MCU_SELF_CHECK_RES:
			my_ptr = (uint8_t *)msg_ptr->uart_msg_data.msg_ptr;
			memcpy(&global_selfcheck_ret, my_ptr, sizeof(global_selfcheck_ret));
			if(is_little_endian())
			{
				endian_conved_func((void *)&global_selfcheck_ret.errcode, 0);
			}
			ret = EC_OK;			
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

void mcu_send_msg_flow(uart_msg *active_msg)
{
	short log_size;
	mcu_log *log_ptr;
	fill_msg_flag msg_flag;
	msg_flag.msg_output = uart_content;
	uint8_t ret;

	switch(active_msg->msg_id)
	{
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
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, UART_SEND_TIMEOUT_LENGTH);
			API_WatchDog_FeedDog();
			mcu_timeout.mcu_req_log_tick = HAL_GetTick();
			mcu_timeout.mcu_req_log_flag = 1;
			break;
		case CMD_REQ_GET_BAT_INFO:
			msg_flag.msg_id = active_msg->msg_id;
			msg_flag.errcode = 0;
			msg_flag.msg_len = 0;
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, UART_SEND_TIMEOUT_LENGTH);
			mcu_timeout.mcu_req_get_bat_info_tick = HAL_GetTick();
			mcu_timeout.mcu_req_get_bat_info_flag = 1;
			break;
		case CMD_REQ_MCU_SELF_CHECK_RES:
			msg_flag.msg_id = active_msg->msg_id;
			msg_flag.errcode = active_msg->ret;
			msg_flag.msg_len = sizeof(mcu_self_check_ret);
			msg_flag.msg_content = active_msg->uart_msg_data.msg_ptr;
			fill_msg(&msg_flag);
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, UART_SEND_TIMEOUT_LENGTH);
			break;
		case CMD_REQ_SHUTDOWN_SOC:
			if(shutdown_state_value.req_msg_send == 0)
			{
				msg_flag.msg_id = active_msg->msg_id;
				msg_flag.errcode = 0;
				msg_flag.msg_len = sizeof(shutdown_soc);
				msg_flag.msg_content = active_msg->uart_msg_data.msg_ptr;
				fill_msg(&msg_flag);
				HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output , msg_flag.msg_total_len, UART_SEND_TIMEOUT_LENGTH);
				shutdown_state_value.req_msg_send = 1;
				shutdown_state_value.curr_tick = HAL_GetTick();
				mcu_timeout.mcu_req_shutdown_soc_tick = HAL_GetTick();
				mcu_timeout.mcu_req_shutdown_soc_flag = 1;					
			}
			break;
		case CMD_REQ_MCU_UPD_READY:
			msg_flag.msg_id = active_msg->msg_id;
			API_WatchDog_FeedDog();
			ret = erase_flash(B_SYSTEM_APPLICATION_ADDRESS, ERASE_FILE_FLASH_SIZE);
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
			HAL_UART_Transmit(&hlpuart1, msg_flag.msg_output, msg_flag.msg_total_len, UART_SEND_TIMEOUT_LENGTH);
			mcu_timeout.mcu_req_udp_ready_tick = HAL_GetTick();
			mcu_timeout.mcu_req_udp_ready_flag = 1;
			break;
		default:
			break;
	}
}

uint8_t uart_msg_proc_flow(void)
{
	uint8_t recv_data;
	int32_t ret;
	uint16_t calc_crc;
	uint8_t msg_tail_hi;
	uint8_t msg_tail_lo;
	uint32_t cur_tick;

	cur_tick = HAL_GetTick();

	while(1)
	{
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
							msgpro.total_index = 0;
							msgpro.pre_very = 0;
							break;
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

		if(HAL_GetTick() - cur_tick >= 1000)
		{
			return 0;
		}
	}

	return 1;
}


