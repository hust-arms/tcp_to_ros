#ifndef MESSAGE719_H
#define MESSAGE719_H

#include <stdint.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define SAT(IN,MAX,MIN) (IN>MAX?MAX:(IN<MIN?MIN:IN))

//#define MDebug  //取消注释该行，输出解析过程

#define MESSAGE_MAX_PAYLOAD_LEN 127 ///< Maximum payload length
#define MESSAGE_NUM_CHECKSUM_BYTES 1
#define MESSAGE_NUM_FOOTER_BYTES 2
#define MESSAGE_HEADER1 0xEB
#define MESSAGE_RXHEADER2 0xA2 //帧头
#define MESSAGE_TXHEADER2 0xA1 //帧头
#define MESSAGE_FOOTER 0xEE
#define MESSAGE_NUM_HEADER_BYTES 5
#define MESSAGE_NUM_NON_PAYLOAD_BYTES (MESSAGE_NUM_HEADER_BYTES + MESSAGE_NUM_CHECKSUM_BYTES)
#define _MSG_PAYLOAD_NON_CONST(msg) ((unsigned char *)(&((msg).payload64[0])))
#define _MSG_PAYLOAD_NON_CONST1(msg) ((unsigned char *)(&((msg)->payload64[0])))

typedef enum
{
    MESSAGE_PARSE_STATE_UNINIT = 0,
    MESSAGE_PARSE_STATE_IDLE,
    MESSAGE_PARSE_STATE_GOT_HEARDER1,
	MESSAGE_PARSE_STATE_GOT_HEARDER2,
    MESSAGE_PARSE_STATE_GOT_SEQH,
    MESSAGE_PARSE_STATE_GOT_SEQL,
    MESSAGE_PARSE_STATE_GOT_LENGTH,
    MESSAGE_PARSE_STATE_GOT_PAYLOAD,
	MESSAGE_PARSE_STATE_GOT_FCS,
    //MESSAGE_PARSE_STATE_GOT_CRC,
    //MESSAGE_PARSE_STATE_GOT_BAD_CRC,
} message_parse_state_t; ///< The state machine for the comm parser

typedef enum
{
    MESSAGE_FRAMING_INCOMPLETE = 0,
    MESSAGE_FRAMING_OK = 1,
    MESSAGE_FRAMING_BAD_CRC = 2
} messgae_framing_t;

typedef struct __message_status
{
    uint8_t msg_received;                                     ///< Number of received messages
    uint8_t buffer_overrun;                                   ///< Number of buffer overruns
    uint8_t parse_error;                                      ///< Number of parse errors
    message_parse_state_t parse_state; ///< Parsing state machine
    uint8_t packet_idx;                                       ///< Index in current packet
    uint16_t packet_rx_success_count;                         ///< Received packets
    uint16_t packet_rx_drop_count;                            ///< Number of packet drops
} message_status_t;

typedef struct __message_message
{
	uint8_t undefined;
    uint8_t checksum; ///< sent at end of packet
	uint8_t fcs;
    uint8_t header1;
	uint8_t header2;
    uint8_t seqH;
    uint8_t seqL;   
    uint8_t len; ///< Length of payload
    uint64_t payload64[(MESSAGE_MAX_PAYLOAD_LEN + MESSAGE_NUM_CHECKSUM_BYTES + 7) / 8];
} message_message_t;

/*初始化函数，对两个结构体内部成员变量进行初始化*/
void message_init();

uint8_t message_parse(uint8_t);
uint16_t message_msg_to_send_buffer(uint8_t *buffer, uint16_t index);

void message_start_checksum();
void message_update_checksum(uint8_t);
void message_cal_checksum(message_message_t *_msg);

void message_status_pack(uint8_t mode, float binocularX, float binocularY, float binocularZ,float binocularHeel,float binocularPitch,float binocularYaw,float usblX,float usblY,float usblZ,float usblHeel,float usblPitch,float usblYaw,
						float height, float depth, float insCourse, float insPitch, float insHeel, float insVE, float insVN, float insVD, double insLng, double insLat, float insAE, float insAN, float insAD, float rudder1, float rudder2, float rudder3, float rudder4 ,uint8_t cmd,
						uint8_t light, uint8_t elevator);
void message_get_status(uint8_t *_mode, float *_binocularX, float *_binocularY, float *_binocularZ,float *_binocularHeel,float *_binocularPitch,float *_binocularYaw,float *_usblX,float *_usblY,float *_usblZ,float *_usblHeel,float *_usblPitch,float *_usblYaw,
						float *_height, float *_depth, float *_insCourse, float *_insPitch, float *_insHeel, float *_insVE, float *_insVN, float *_insVD, double *_insLng, double *_insLat, float *_insAE, float *_insAN, float *_insAD, float *_rudder1, float *_rudder2, float *_rudder3, float *_rudder4 ,uint8_t *_cmd,
						uint8_t *_light, uint8_t *_elevator);
void message_get_control_cmd(uint8_t *_mode, uint8_t *_statusFd, uint8_t *_vel, float *_course, float *_depth, float *_rudder1,
							float *_rudder2, float *_rudder3, float *_rudder4, uint8_t *_ligth, uint8_t *_elevator);
void message_control_cmd_pack(uint8_t mode, uint8_t statusFd, uint8_t vel, float course, float depth, float rudder1,
							 float rudder2, float rudder3, float rudder4, uint8_t ligth, uint8_t elevator);

void message_test_pack();
#endif // MESSAGE719_H
