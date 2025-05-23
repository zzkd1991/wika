#ifndef _uart_prot_h_
#define _uart_prot_h_

#include "main.h"

#define A_SYSTEM_APPLICATION_ADDRESS	0x8010000
#define B_SYSTEM_APPLICATION_ADDRESS	0x8020000
#define AB_SYSTEM_FLAG_ADDRESS	0x8030000
#define ERASE_FILE_FLASH_SIZE	0x8000
#define UART_SEND_TIMEOUT_LENGTH		1000

#define MY_MD5_SIZE		16
#define MY_MD5_READ_DATA_SIZE 512


#define MK_CMDID(t,id)		((((t)&0x0f) << 12) | (((id)&0xfff)))		
#define MSG_LENG	1024

enum cmd_id_t {
	CMD_MIN = MK_CMDID(0x00, 0x00),
	CMD_REQ_UPGRADE_MCU = MK_CMDID(1,1),
	CMD_REP_UPGRADE_MCU = MK_CMDID(1,2),
	CMD_REQ_MCU_UPD_READY	= MK_CMDID(1,3),
	CMD_REP_MCU_UPD_READY = MK_CMDID(1,4),
	CMD_REQ_UPLOAD_MCU_FW = MK_CMDID(1,5),
	CMD_REP_UPLOAD_MCU_FW = MK_CMDID(1,6),
	CMD_REQ_MCU_UPD_STATE = MK_CMDID(1,7),
	CMD_REP_MCU_UPD_STATE = MK_CMDID(1,8),
	CMD_REQ_SHUTDOWN_SOC = MK_CMDID(2,1),
	CMD_REP_SHUTDOWN_SOC = MK_CMDID(2,2),
	CMD_REQ_SET_RTC = MK_CMDID(2,3),
	CMD_REP_SET_RTC = MK_CMDID(2,4),
	CMD_REQ_MCU_SELF_CHECK = MK_CMDID(2,5),
	CMD_REP_MCU_SELF_CHECK = MK_CMDID(2,6),
	CMD_REQ_MCU_CHARGE_CTRL = MK_CMDID(2,7),
	CMD_REP_MCU_CHARGE_CTRL = MK_CMDID(2,8),
	CMD_REQ_GET_MCU_VER = MK_CMDID(3,1),
	CMD_REP_GET_MCU_VER = MK_CMDID(3,2),
	CMD_REQ_GET_SOC_POWER_NUM = MK_CMDID(3,3),
	CMD_REP_GET_SOC_POWER_NUM = MK_CMDID(3,4),
	CMD_REQ_GET_RTC			= MK_CMDID(3,5),
	CMD_REP_GET_RTC			= MK_CMDID(3,6),
	CMD_REQ_GET_BAT_INFO = MK_CMDID(3,7),
	CMD_REP_GET_BAT_INFO = MK_CMDID(3,8),
	CMD_REQ_HEARTBEAT = MK_CMDID(4,1),
	CMD_REP_HEARTBEAT = MK_CMDID(4,2),
	CMD_REQ_MCU_LOG = MK_CMDID(5,1),
	CMD_REP_MCU_LOG = MK_CMDID(5,2),
	CMD_REQ_SOC_STATE = MK_CMDID(5,3),
	CMD_REP_SOC_STATE = MK_CMDID(5,4),
	CMD_REQ_BAT_CHARGE_STATE = MK_CMDID(5,5),
	CMD_REP_BAT_CHARGE_STATE = MK_CMDID(5,6),
	CMD_REQ_MCU_SELF_CHECK_RES = MK_CMDID(5,7),
	CMD_REP_MCU_SELF_CHECK_RES = MK_CMDID(5,8),
};

enum battery_state_type_t {
	BATTERY_STATUS_UNKNOWN = 0,
	BATTERY_STATUS_CHARGING = 1,
	BATTERY_STATUS_DISCHARGING = 2,
	BATTERY_STATUS_NOT_CHARGING = 3,
	BATTERY_STATUS_FULL,
};

enum mcu_module_t {
	MCU_MOD_ALL = 0x00,
	MCU_MOD_FLASH = 0x01,
	MCU_MOD_BATTERY = 0x02,
};

enum mcu_log_level_t {
	LOG_MEMRG = 0x00,
	LOG_ALERT = 0x01,
	LOG_CRIT = 0x02,
	LOG_ERR = 0x03,
	LOG_WARNING = 0x04,
	LOG_NOTICE = 0x05,
	LOG_INFO = 0x06,
	LOG_DEBUG = 0x07,
};

#pragma pack(1)

__packed typedef struct common_rep_t {
	short errcode;
}common_rep;

__packed typedef struct battery_discharge_state_t {
	uint8_t state;
}battery_discharge_state;

__packed typedef struct battery_charge_state_t {
	uint8_t state;
}battery_charge_state;

__packed typedef struct soc_sys_state_t {
	uint8_t state;
}soc_sys_state;

__packed typedef struct mcu_version_t {
	uint8_t major;
	uint8_t minor;
}mcu_version;

__packed typedef struct soc_power_num_t {
	uint8_t on_num;
	uint8_t off_num;
}soc_power_num;

__packed typedef struct shutdown_soc_t {
	uint8_t shutdown;
}shutdown_soc;

__packed typedef struct rtc_datetime {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t pad;
}rtc_datetime;

__packed typedef struct battery_info_t {
	uint8_t temp;
	uint8_t percentage;
}battery_info;

__packed typedef struct mcu_self_check_t {
	int32_t mod;
}mcu_self_check;

__packed typedef struct mcu_self_check_ins_t
{
	mcu_self_check inst;
	uint8_t self_check_flow_flag;
}mcu_self_check_ins;

__packed typedef struct mcu_self_check_ret_t {
	uint32_t mod;
	uint32_t result;
}mcu_self_check_ret;

__packed typedef struct mcu_log_t {
	char level;
	char errcode;
	short size;
}mcu_log;

__packed typedef struct req_upgrade_mcu_t {
	int32_t filesize;
	int8_t type;
	int8_t major;
	int minor;
	int8_t pad;
}req_upgrade_mcu;

__packed typedef struct bin_file_info_t {
	uint8_t curr_partition;
	int32_t file_size;
	uint8_t md5_value[MY_MD5_SIZE];
}bin_file_info;


__packed typedef struct mcu_upgrade_pack_t {
	uint16_t index;
	uint16_t size;
}mcu_upgrade_pack;

__packed typedef struct upgrade_state_t {
	int8_t percentage;
}upgrade_state;

__packed typedef struct battery_state_t {
	uint8_t state;
}battery_state;


#pragma pack()



typedef struct msg_data_t {
	void* msg_ptr;
}msg_data;

typedef struct fill_msg_flag_t {
	uint16_t msg_id;
	//uint8_t fixed_msg;
	uint16_t msg_len;
	int8_t errcode;
	uint16_t msg_total_len;
	uint8_t *msg_output;
	void *msg_content;
}fill_msg_flag;


typedef struct uart_message_t {
	uint16_t msg_header; 
	uint16_t  msg_id;
	uint16_t msg_len;
	msg_data uart_msg_data;
	uint16_t msg_crc;	
 	uint16_t msg_tail;
	uint8_t ret;
}uart_msg;

typedef struct msg_proc_t
{
	uint16_t total_index;
	uint8_t msgid_recv;
	uint8_t msg_len_recv;
	uint8_t msg_cont_recv;
	uint8_t msg_crc_recv;
	uint8_t msg_tail_recv;
	uint8_t pre_very;
}msg_proc;

typedef struct heartbeat_pro_t
{
	uint32_t last_tick_value;
}heartbeat_pro;

typedef struct shutdown_state_t
{
	uint32_t curr_tick;
	uint8_t req_msg_send;
}shutdown_state;

typedef struct mcu_req_timeout_t
{
	uint32_t mcu_req_bat_discharge_tick;
	uint32_t mcu_req_log_tick;
	uint32_t mcu_req_get_bat_info_tick;
	uint32_t mcu_req_shutdown_soc_tick;
	uint32_t mcu_req_udp_ready_tick;
	uint32_t soc_ack_bat_discharge_tick;
	uint32_t soc_ack_mcu_log_tick;
	uint32_t soc_ack_get_bat_info_tick;
	uint32_t soc_ack_shutdown_info_tick;
	uint32_t soc_ack_mcu_udp_ready_tick;
	uint8_t mcu_req_bat_discharge_flag;
	uint8_t mcu_req_log_flag;
	uint8_t mcu_req_get_bat_info_flag;
	uint8_t mcu_req_shutdown_soc_flag;
	uint8_t mcu_req_udp_ready_flag;
	uint8_t soc_ack_bat_discharge_flag;
	uint8_t soc_ack_log_flag;
	uint8_t soc_ack_get_bat_info_flag;
	uint8_t soc_ack_shutdown_info_flag;
	uint8_t soc_ack_udp_ready_flag;
	uint8_t timeout_bat_discharge_flag;
	uint8_t timeout_mcu_log_flag;
	uint8_t timeout_get_bat_info_flag;
	uint8_t timeout_shutdown_soc_flag;
	uint8_t timeout_udp_ready_flag;
}mcu_req_timeout;



#define EC_OK		0
#define EC_FAIL		1
#define EC_PERM		2
#define EC_INVAL	3
#define EC_NOSPACE	4
#define EC_CRC		5
#define EC_NODEV	6
#define EC_BUSY		7

#define UART_MSG_EXTRA_LEN	10

#define UART_MSG_LEN (1024U + UART_MSG_EXTRA_LEN)

extern uint8_t Uart_Recv_Fifo[UART_MSG_LEN];



uint8_t API_UART_RxFifoEnable(uint8_t *rx_data, uint16_t length);
uint8_t parse_dispacth_msg_flow(void);
uint8_t uart_msg_proc_flow(void);
void endian_conved_func(void *value, uint8_t type);
int is_little_endian(void);
void endian_conved_func(void *value, uint8_t type);
void mcu_send_msg_flow(uart_msg *active_msg);
void shutdown_func_from_soc(void);
void self_check_pro_flow(void);
void turnoff_soc_func(void);
void turnon_soc_func(void);
void heartbeat_timeout_func(void);
void mcu_send_log_flow(mcu_log *log, char *log_content);
void power_manager_func(void);
void no_charge_func(void);
void mcu_send_readymsg_func(void);
void shutdown_func_from_button(void);
void get_battery_info_func(void);
void PreJumpToApplication(void);


extern heartbeat_pro heartbeat_value;
extern soc_power_num global_soc_power_num;
extern uint16_t myheartbeat_timeout_cnt;
extern uint8_t switch_en_flag_12v;
extern uint16_t REAL_VALUE;
extern bin_file_info binfile;


#endif



