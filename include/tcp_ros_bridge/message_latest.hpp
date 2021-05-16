#ifndef MESSAGE719_H
#define MESSAGE719_H

#include <stdint.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define SAT(IN,MAX,MIN) (IN>MAX?MAX:(IN<MIN?MIN:IN))

// #define WOKEINGONJETSON

//#define MDebug  //取消注释该行，输出解析过程

#define MESSAGE_MAX_PAYLOAD_LEN 127 ///< Maximum payload length
#define MESSAGE_NUM_CHECKSUM_BYTES 1
#define MESSAGE_NUM_FOOTER_BYTES 2

#ifdef WOKEINGONJETSON
#define MESSAGE_HEADER1 0xEB
#define MESSAGE_RXHEADER2 0xA1 //帧头
#define MESSAGE_TXHEADER2 0xA2 //帧头
#define MESSAGE_FOOTER 0xEE
#define MESSAGE_TXFOOTER2 0x2A
#else
#define MESSAGE_HEADER1 0xEB
#define MESSAGE_RXHEADER2 0xA2 //帧头
#define MESSAGE_TXHEADER2 0xA1 //帧头
#define MESSAGE_FOOTER 0xEE
#define MESSAGE_TXFOOTER2 0x1A
#endif

#define MESSAGE_NUM_HEADER_BYTES 5
#define MESSAGE_NUM_NON_PAYLOAD_BYTES (MESSAGE_NUM_HEADER_BYTES + MESSAGE_NUM_CHECKSUM_BYTES)
#define _MSG_PAYLOAD_NON_CONST(msg) ((unsigned char *)(&((msg).payload64[0])))
#define _MSG_PAYLOAD_NON_CONST1(msg) ((unsigned char *)(&((msg)->payload64[0])))

message_status_t status;
message_message_t rxmsg;
message_message_t txmsg;

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
/**
 * @brief 消息结构体初始化
 */
void message_init()
{
    status.msg_received = 0;
    status.buffer_overrun = 0;
    status.parse_error = 0;
    status.parse_state = MESSAGE_PARSE_STATE_IDLE;
    status.packet_idx = 0;
    status.packet_rx_success_count = 0;
    status.packet_rx_drop_count = 0;

    rxmsg.checksum = 0;
    rxmsg.seqH = 0;
    rxmsg.seqL = 0;
    rxmsg.len = 0;
    
	txmsg.header1 = MESSAGE_HEADER1;
	txmsg.header2 = MESSAGE_TXHEADER2;
}

/**
 * @brief 从字节流中提取出完整的消息结构，将接受到的字节流一个字节一个字节的解析
 * @param c 待解析的字节
 * @return 解析结果（0-还没解析成完成的消息结构，1-解析出完整的消息结构，2-解析出完整的消息结构但校验错误）
 */
uint8_t message_parse(uint8_t)
{
    status.msg_received = MESSAGE_FRAMING_INCOMPLETE;
#ifdef MDebug
    printf("Parse this uint8_t: 0x%x\n", c);
#endif
    switch (status.parse_state)
    {
    case MESSAGE_PARSE_STATE_UNINIT:
    case MESSAGE_PARSE_STATE_IDLE:
        if (c == MESSAGE_HEADER1)
        {
#ifdef MDebug
            printf("GOT_HEADER\n");
#endif
            status.parse_state = MESSAGE_PARSE_STATE_GOT_HEARDER1;
            rxmsg.header1 = c;
        }
        break;
	case MESSAGE_PARSE_STATE_GOT_HEARDER1:
		if(c == MESSAGE_RXHEADER2)
		{
#ifdef MDebug
			printf("GOT_HEADER2\n");
#endif			
			status.parse_state = MESSAGE_PARSE_STATE_GOT_HEARDER2;
			rxmsg.len = 0;
			rxmsg.header2 = c;
			message_start_checksum();
		}else{
			status.parse_state = MESSAGE_PARSE_STATE_IDLE;
			status.parse_error++;
		}
		break;
    case MESSAGE_PARSE_STATE_GOT_HEARDER2:
#ifdef MDebug
        printf("GOT_SEQH\n");
#endif
        rxmsg.seqH = c;
        status.parse_state = MESSAGE_PARSE_STATE_GOT_SEQH;
        break;
    case MESSAGE_PARSE_STATE_GOT_SEQH:
#ifdef MDebug
        printf("GOT_SEQL\n");
#endif
        rxmsg.seqL = c;
        status.parse_state = MESSAGE_PARSE_STATE_GOT_SEQL;
        break;
    case MESSAGE_PARSE_STATE_GOT_SEQL:
        if (c > MESSAGE_MAX_PAYLOAD_LEN)
        {
#ifdef MDebug
            printf("buffer_overrun\n");
#endif
            status.buffer_overrun++;
            status.parse_error++;
            status.msg_received = MESSAGE_FRAMING_INCOMPLETE;
            status.parse_state = MESSAGE_PARSE_STATE_IDLE;
        }
        else
        {
            rxmsg.len = c;
#ifdef MDebug
            printf("GOT_LENGTH: %d\n", c);
#endif
            status.packet_idx = 0;
            if (rxmsg.len == 0)
                status.parse_state = MESSAGE_PARSE_STATE_GOT_PAYLOAD;
            else
                status.parse_state = MESSAGE_PARSE_STATE_GOT_LENGTH;
        }
        break;
    case MESSAGE_PARSE_STATE_GOT_LENGTH:
        _MSG_PAYLOAD_NON_CONST(rxmsg)[status.packet_idx++] = c;
        message_update_checksum(c);
#ifdef MDebug
        printf("pack_idx: %d\n", status.packet_idx);
#endif
        if (status.packet_idx == rxmsg.len)
        {
#ifdef MDebug
            printf("GOT_PAYLOAD\n");
            printf("GOT_CHECKSUM 0x%x\n",rxmsg.checksum);
#endif
            status.parse_state = MESSAGE_PARSE_STATE_GOT_PAYLOAD;
        }
        break;
    case MESSAGE_PARSE_STATE_GOT_PAYLOAD:
#ifdef MDebug
            printf("GOT_FCS\n");
#endif	
		rxmsg.fcs = c;
		status.parse_state = MESSAGE_PARSE_STATE_GOT_FCS;
		break;
	case MESSAGE_PARSE_STATE_GOT_FCS:
#ifdef MDebug
		printf("GOT_FOOTER\n");
#endif	
		if(MESSAGE_FOOTER == c)
		{
			if (rxmsg.checksum  != rxmsg.fcs)
			{
#ifdef MDebug
				printf("GOT_BAD_CRC\n");
#endif
				status.msg_received = MESSAGE_FRAMING_BAD_CRC;
				status.packet_rx_drop_count++;
				status.parse_error++;
			}
			else
			{
#ifdef MDebug
				printf("GOT_CRC\n");
#endif
				status.msg_received = MESSAGE_FRAMING_OK;
			}
		}
		else
		{
#ifdef MDebug
			printf("GOT_ERROR_FOOTER\n");
#endif		
			status.msg_received = MESSAGE_FRAMING_BAD_CRC;
			status.packet_rx_drop_count++;
			status.parse_error++;
		}
		status.parse_state = MESSAGE_PARSE_STATE_IDLE;
        break;
	}

    if (status.msg_received == MESSAGE_FRAMING_OK)
    {
        if (status.packet_rx_success_count == 0)
            status.packet_rx_drop_count = 0;
        status.packet_rx_success_count++;
		
    }

    return status.msg_received;
}

/**
 * @brief 将待发送的消息结构转化字节流
 * @param buffer 字节流
 * @param _msg 待转化的消息结构
 * @return buffer的长度
 */
uint16_t message_msg_to_send_buffer(uint8_t *buffer, uint16_t index)
{
	txmsg.seqH = (index>>8);
	txmsg.seqL = (index&0xFF);
	
    memcpy(buffer, (const uint8_t *)&txmsg.header1, MESSAGE_NUM_HEADER_BYTES + (uint16_t)txmsg.len);
   
    uint8_t *ck = buffer + (MESSAGE_NUM_HEADER_BYTES + (uint16_t)txmsg.len);

    ck[0] = txmsg.checksum;

    uint8_t *foot = buffer + (MESSAGE_NUM_HEADER_BYTES + MESSAGE_NUM_CHECKSUM_BYTES + (uint16_t)txmsg.len);
    foot[0] = MESSAGE_FOOTER;
    foot[1] = MESSAGE_TXFOOTER2;

    return MESSAGE_NUM_NON_PAYLOAD_BYTES + MESSAGE_NUM_FOOTER_BYTES + (uint16_t)txmsg.len;
}

/**
 * @brief 接收校验初始化
 */
void message_start_checksum()
{
    rxmsg.checksum = 0;
}

/**
 * @brief 更新接收校验
 */
void message_update_checksum(uint8_t c)
{
    rxmsg.checksum += c;
}

/**
 * @brief 计算发送校验
 */
void message_cal_checksum(message_message_t *_msg)
{
    _msg->checksum = 0;
    for (int i = 0; i < _msg->len; i++)
    {
        _msg->checksum += _MSG_PAYLOAD_NON_CONST1(_msg)[i];
    }
}

void message_status_pack(uint8_t mode, float binocularX, float binocularY, float binocularZ,float binocularHeel,float binocularPitch,float binocularYaw,float usblX,float usblY,float usblZ,float usblHeel,float usblPitch,float usblYaw, double usblLng, double usblLat,
						float height, float depth, float insCourse, float insPitch, float insHeel, float insVE, float insVN, float insVD, double insLng, double insLat, float insAE, float insAN, float insAD, int16_t propeller1, int16_t propeller2, int16_t propeller3, int16_t propeller4 ,uint8_t cmd,
						uint8_t light, uint8_t elevator, float insWz, float insWx, float insWy, float batteryVoltage, uint8_t batteryLeft, float batteryTemp, uint32_t leak, uint8_t batteryEmerg,
						uint8_t AddedBatteryEmerg, uint8_t propellerEmerg, uint8_t nodeEmerg, uint8_t estarFd, uint8_t paoZaiFd, uint8_t sensorSwitchFd, uint8_t shiFangFd)
{
	uint8_t buf[103];
	txmsg.len = 103;
	buf[0] = mode;
	binocularX = SAT(binocularX,30,-30);
	binocularY = SAT(binocularY,30,-30);
	binocularZ = SAT(binocularZ,30,-30);
	if(binocularX > 0)
	{
		buf[1] = ((uint16_t)(binocularX*1000) >> 8);
		buf[2] = ((uint16_t)(binocularX*1000) & 0xFF);
	}else{
		buf[1] = (((uint16_t)(fabs(binocularX)*1000)>>8)|0x80);
		buf[2] = ((uint16_t)(fabs(binocularX)*1000) & 0xFF);
	}
	if(binocularY > 0)
	{
		buf[3] = ((uint16_t)(binocularY*1000) >> 8);
		buf[4] = ((uint16_t)(binocularY*1000) & 0xFF);
	}else{
		buf[3] = (((uint16_t)(fabs(binocularY)*1000)>>8)|0x80);
		buf[4] = ((uint16_t)(fabs(binocularY)*1000) & 0xFF);
	}
	if(binocularZ > 0)
	{
		buf[5] = ((uint16_t)(binocularZ*1000) >> 8);
		buf[6] = ((uint16_t)(binocularZ*1000) & 0xFF);
	}else{
		buf[5] = (((uint16_t)(fabs(binocularZ)*1000)>>8)|0x80);
		buf[6] = ((uint16_t)(fabs(binocularZ)*1000) & 0xFF);
	}
	binocularHeel = SAT(binocularHeel,180,-180);
	binocularPitch = SAT(binocularPitch,180,-180);
	binocularYaw = SAT(binocularYaw,180,-180);
	if(binocularHeel>0)
	{
		buf[7] = ((uint16_t)(binocularHeel*100) >> 8);
		buf[8] = ((uint16_t)(binocularHeel*100) & 0xFF);
	}
	else{
		buf[7] = (((uint16_t)(fabs(binocularHeel)*100)>>8)|0x80);
		buf[8] = ((uint16_t)(fabs(binocularHeel)*100) & 0xFF);
	}
	if(binocularPitch>0)
	{
		buf[9] = ((uint16_t)(binocularPitch*100) >> 8);
		buf[10] = ((uint16_t)(binocularPitch*100) & 0xFF);
	}
	else{
		buf[9] = (((uint16_t)(fabs(binocularPitch)*100)>>8)|0x80);
		buf[10] = ((uint16_t)(fabs(binocularPitch)*100) & 0xFF);
	}
	if(binocularYaw>0)
	{
		buf[11] = ((uint16_t)(binocularYaw*100) >> 8);
		buf[12] = ((uint16_t)(binocularYaw*100) & 0xFF);
	}
	else{
		buf[11] = (((uint16_t)(fabs(binocularYaw)*100)>>8)|0x80);
		buf[12] = ((uint16_t)(fabs(binocularYaw)*100) & 0xFF);
	}
	usblX = SAT(usblX,1000,-1000);
	usblY = SAT(usblY,1000,-1000);
	usblZ = SAT(usblZ,1000,-1000);
	if(usblX > 0)
	{
		buf[13] = ((uint32_t)(usblX*1000) >> 24);
		buf[14] = (((uint32_t)(usblX*1000) >> 16)&0xFF);
		buf[15] = (((uint32_t)(usblX*1000) >> 8)&0xFF);
		buf[16] = ((uint32_t)(usblX*1000) & 0xFF);
	}else{
		buf[13] = (((uint32_t)(abs(usblX)*1000) >> 24)|0x80);
		buf[14] = (((uint32_t)(abs(usblX)*1000) >> 16)&0xFF);
		buf[15] = (((uint32_t)(abs(usblX)*1000) >> 8)&0xFF);
		buf[16] = ((uint32_t)(abs(usblX)*1000) & 0xFF);
	}
	if(usblY > 0)
	{
		buf[17] = ((uint32_t)(usblY*1000) >> 24);
		buf[18] = (((uint32_t)(usblY*1000) >> 16)&0xFF);
		buf[19] = (((uint32_t)(usblY*1000) >> 8)&0xFF);
		buf[20] = ((uint32_t)(usblY*1000) & 0xFF);
	}else{
		buf[17] = (((uint32_t)(abs(usblY)*1000) >> 24)|0x80);
		buf[18] = (((uint32_t)(abs(usblY)*1000) >> 16)&0xFF);
		buf[19] = (((uint32_t)(abs(usblY)*1000) >> 8)&0xFF);
		buf[20] = ((uint32_t)(abs(usblY)*1000) & 0xFF);
	}
	if(usblZ > 0)
	{
		buf[21] = ((uint32_t)(usblZ*1000) >> 24);
		buf[22] = (((uint32_t)(usblZ*1000) >> 16)&0xFF);
		buf[23] = (((uint32_t)(usblZ*1000) >> 8)&0xFF);
		buf[24] = ((uint32_t)(usblZ*1000) & 0xFF);
	}else{
		buf[21] = (((uint32_t)(abs(usblZ)*1000) >> 24)|0x80);
		buf[22] = (((uint32_t)(abs(usblZ)*1000) >> 16)&0xFF);
		buf[23] = (((uint32_t)(abs(usblZ)*1000) >> 8)&0xFF);
		buf[24] = ((uint32_t)(abs(usblZ)*1000) & 0xFF);
	}
	usblHeel = SAT(usblHeel,90,-90);
	usblPitch = SAT(usblPitch,90,-90);
	usblYaw = SAT(usblYaw,180,-180);
	if(usblHeel>0)
	{
		buf[25] = ((uint16_t)(usblHeel*100) >> 8);
		buf[26] = ((uint16_t)(usblHeel*100) & 0xFF);
	}
	else{
		buf[25] = (((uint16_t)(fabs(usblHeel)*100)>>8)|0x80);
		buf[26] = ((uint16_t)(fabs(usblHeel)*100) & 0xFF);
	}
	if(usblPitch>0)
	{
		buf[27] = ((uint16_t)(usblPitch*100) >> 8);
		buf[28] = ((uint16_t)(usblPitch*100) & 0xFF);
	}
	else{
		buf[27] = (((uint16_t)(fabs(usblPitch)*100)>>8)|0x80);
		buf[28] = ((uint16_t)(fabs(usblPitch)*100) & 0xFF);
	}
	if(usblYaw>0)
	{
		buf[29] = ((uint16_t)(usblYaw*100) >> 8);
		buf[30] = ((uint16_t)(usblYaw*100) & 0xFF);
	}
	else{
		buf[29] = (((uint16_t)(fabs(usblYaw)*100)>>8)|0x80);
		buf[30] = ((uint16_t)(fabs(usblYaw)*100) & 0xFF);
	}
	int32_t intUsblLng = usblLng*11930464.7111;
	int32_t intUsblLat = usblLat*11930464.7111;
	if(intUsblLng>0)
	{
		buf[31] = ((uint32_t)(intUsblLng) >> 24);
		buf[32] = (((uint32_t)(intUsblLng) >> 16)&0xFF);
		buf[33] = (((uint32_t)(intUsblLng) >> 8)&0xFF);
		buf[34] = ((uint32_t)(intUsblLng) & 0xFF);
	}else{
		buf[31] = (((uint32_t)(abs(intUsblLng)) >> 24)|0x80);
		buf[32] = (((uint32_t)(abs(intUsblLng)) >> 16)&0xFF);
		buf[33] = (((uint32_t)(abs(intUsblLng)) >> 8)&0xFF);
		buf[34] = ((uint32_t)(abs(intUsblLng)) & 0xFF);
	}
	if(intUsblLat>0)
	{
		buf[35] = ((uint32_t)(intUsblLat) >> 24);
		buf[36] = (((uint32_t)(intUsblLat) >> 16)&0xFF);
		buf[37] = (((uint32_t)(intUsblLat) >> 8)&0xFF);
		buf[38] = ((uint32_t)(intUsblLat) & 0xFF);
	}else{
		buf[35] = (((uint32_t)(abs(intUsblLat)) >> 24)|0x80);
		buf[36] = (((uint32_t)(abs(intUsblLat)) >> 16)&0xFF);
		buf[37] = (((uint32_t)(abs(intUsblLat)) >> 8)&0xFF);
		buf[38] = ((uint32_t)(abs(intUsblLat)) & 0xFF);
	}
	buf[39] = ((uint16_t)(height*100) >> 8);
	buf[40] = ((uint16_t)(height*100) & 0xFF);
	buf[41] = ((uint16_t)(depth*100) >> 8);
	buf[42] = ((uint16_t)(depth*100) & 0xFF);
	buf[43] = ((uint16_t)(insCourse*100) >> 8);
	buf[44] = ((uint16_t)(insCourse*100) & 0xFF);
	if(insPitch>0)
	{
		buf[45] = ((uint16_t)(insPitch*100) >> 8);
		buf[46] = ((uint16_t)(insPitch*100) & 0xFF);
	}else{
		buf[45] = (((uint16_t)(fabs(insPitch*100)) >> 8)|0x80);
		buf[46] = ((uint16_t)(fabs(insPitch*100)) & 0xFF);
	}
	if(insHeel>0)
	{
		buf[47] = ((uint16_t)(insHeel*100) >> 8);
		buf[48] = ((uint16_t)(insHeel*100) & 0xFF);
	}else{
		buf[47] = (((uint16_t)(fabs(insHeel*100)) >> 8)|0x80);
		buf[48] = ((uint16_t)(fabs(insHeel*100)) & 0xFF);
	}
	if(insVE>0)
	{
		buf[49] = ((uint16_t)(insVE*100) >> 8);
		buf[50] = ((uint16_t)(insVE*100) & 0xFF);
	}else{
		buf[49] = (((uint16_t)(fabs(insVE*100)) >> 8)|0x80);
		buf[50] = ((uint16_t)(fabs(insVE*100)) & 0xFF);
	}
	if(insVN>0)
	{
		buf[51] = ((uint16_t)(insVN*100) >> 8);
		buf[52] = ((uint16_t)(insVN*100) & 0xFF);
	}else{
		buf[51] = (((uint16_t)(fabs(insVN*100)) >> 8)|0x80);
		buf[52] = ((uint16_t)(fabs(insVN*100)) & 0xFF);
	}
	if(insVD>0)
	{
		buf[53] = ((uint16_t)(insVD*100) >> 8);
		buf[54] = ((uint16_t)(insVD*100) & 0xFF);
	}else{
		buf[53] = (((uint16_t)(fabs(insVD*100)) >> 8)|0x80);
		buf[54] = ((uint16_t)(fabs(insVD*100)) & 0xFF);
	}
	int32_t intLng = insLng*11930464.7111;
	if(intLng>0)
	{
		buf[55] = ((uint32_t)(intLng) >> 24);
		buf[56] = (((uint32_t)(intLng) >> 16)&0xFF);
		buf[57] = (((uint32_t)(intLng) >> 8)&0xFF);
		buf[58] = ((uint32_t)(intLng) & 0xFF);
	}else{
		buf[55] = (((uint32_t)(abs(intLng)) >> 24)|0x80);
		buf[56] = (((uint32_t)(abs(intLng)) >> 16)&0xFF);
		buf[57] = (((uint32_t)(abs(intLng)) >> 8)&0xFF);
		buf[58] = ((uint32_t)(abs(intLng)) & 0xFF);
	}
	int32_t intLat = insLat*11930464.7111;
	if(intLat>0)
	{
		buf[59] = ((uint32_t)(intLat) >> 24);
		buf[60] = (((uint32_t)(intLat) >> 16)&0xFF);
		buf[61] = (((uint32_t)(intLat) >> 8)&0xFF);
		buf[62] = ((uint32_t)(intLat) & 0xFF);
	}else{
		buf[59] = (((uint32_t)(abs(intLat)) >> 24)|0x80);
		buf[60] = (((uint32_t)(abs(intLat)) >> 16)&0xFF);
		buf[61] = (((uint32_t)(abs(intLat)) >> 8)&0xFF);
		buf[62] = ((uint32_t)(abs(intLat)) & 0xFF);
	}
	if(insAE>0)
	{
		buf[63] = ((uint16_t)(insAE*100) >> 8);
		buf[64] = ((uint16_t)(insAE*100) & 0xFF);
	}else{
		buf[63] = (((uint16_t)(fabs(insAE*100)) >> 8)|0x80);
		buf[64] = ((uint16_t)(fabs(insAE*100)) & 0xFF);
	}
	if(insAN>0)
	{
		buf[65] = ((uint16_t)(insAN*100) >> 8);
		buf[66] = ((uint16_t)(insAN*100) & 0xFF);
	}else{
		buf[65] = (((uint16_t)(fabs(insAN*100)) >> 8)|0x80);
		buf[66] = ((uint16_t)(fabs(insAN*100)) & 0xFF);
	}
	if(insAD>0)
	{
		buf[67] = ((uint16_t)(insAD*100) >> 8);
		buf[68] = ((uint16_t)(insAD*100) & 0xFF);
	}else{
		buf[67] = (((uint16_t)(fabs(insAD*100)) >> 8)|0x80);
		buf[68] = ((uint16_t)(fabs(insAD*100)) & 0xFF);
	}
	if(propeller1>0)
	{
		buf[69] = ((uint16_t)(propeller1) >> 8);
		buf[70] = ((uint16_t)(propeller1) & 0xFF);
	}else{
		buf[69] = (((uint16_t)(fabs(propeller1)) >> 8)|0x80);
		buf[70] = ((uint16_t)(fabs(propeller1)) & 0xFF);
	}
	if(propeller2>0)
	{
		buf[71] = ((uint16_t)(propeller2) >> 8);
		buf[72] = ((uint16_t)(propeller2) & 0xFF);
	}else{
		buf[71] = (((uint16_t)(fabs(propeller2)) >> 8)|0x80);
		buf[72] = ((uint16_t)(fabs(propeller2)) & 0xFF);
	}
	if(propeller3>0)
	{
		buf[73] = ((uint16_t)(propeller3) >> 8);
		buf[74] = ((uint16_t)(propeller3) & 0xFF);
	}else{
		buf[73] = (((uint16_t)(fabs(propeller3)) >> 8)|0x80);
		buf[74] = ((uint16_t)(fabs(propeller3)) & 0xFF);
	}
	if(propeller4>0)
	{
		buf[75] = ((uint16_t)(propeller4) >> 8);
		buf[76] = ((uint16_t)(propeller4) & 0xFF);
	}else{
		buf[75] = (((uint16_t)(fabs(propeller4)) >> 8)|0x80);
		buf[76] = ((uint16_t)(fabs(propeller4)) & 0xFF);
	}
	buf[77] = cmd;
	buf[78] = light;
	buf[79] = elevator;
	if(insWz>0)
	{
		buf[80] = ((uint16_t)(insWz*100) >> 8);
		buf[81] = ((uint16_t)(insWz*100) & 0xFF);
	}else{
		buf[80] = (((uint16_t)(fabs(insWz*100)) >> 8)|0x80);
		buf[81] = ((uint16_t)(fabs(insWz*100)) & 0xFF);
	}
	if(insWx>0)
	{
		buf[82] = ((uint16_t)(insWx*100) >> 8);
		buf[83] = ((uint16_t)(insWx*100) & 0xFF);
	}else{
		buf[82] = (((uint16_t)(fabs(insWx*100)) >> 8)|0x80);
		buf[83] = ((uint16_t)(fabs(insWx*100)) & 0xFF);
	}
	if(insWy>0)
	{
		buf[84] = ((uint16_t)(insWy*100) >> 8);
		buf[85] = ((uint16_t)(insWy*100) & 0xFF);
	}else{
		buf[84] = (((uint16_t)(fabs(insWy*100)) >> 8)|0x80);
		buf[85] = ((uint16_t)(fabs(insWy*100)) & 0xFF);
	}
	buf[86] = ((uint16_t)(batteryVoltage/1000.f) >> 8);
	buf[87] = ((uint16_t)(batteryVoltage/1000.f) & 0xFF);
	buf[88] = batteryLeft;
	buf[89] = ((uint16_t)(batteryTemp*10) >> 8);
	buf[90] = ((uint16_t)(batteryTemp*10) & 0xFF);
	buf[91] = leak >> 24;
	buf[92] = (leak >> 26)&0xFF;
	buf[93] = (leak >> 8)&0xFF;
	buf[94] = leak&0xFF;
	buf[95] = batteryEmerg;
	buf[96] = AddedBatteryEmerg;
	buf[97] = propellerEmerg;
	buf[98] = nodeEmerg;
	buf[99] = estarFd;
	buf[100] = paoZaiFd;
	buf[101] = sensorSwitchFd;
	buf[102] = shiFangFd;
	memcpy(_MSG_PAYLOAD_NON_CONST(txmsg), buf, txmsg.len);
	message_cal_checksum(&txmsg);
}
						
void message_get_status(uint8_t *_mode, float *_binocularX, float *_binocularY, float *_binocularZ,float *_binocularHeel,float *_binocularPitch,float *_binocularYaw,float *_usblX,float *_usblY,float *_usblZ,float *_usblHeel,float *_usblPitch,float *_usblYaw,double *_usblLng,double*_usblLat,
						float *_height, float *_depth, float *_insCourse, float *_insPitch, float *_insHeel, float *_insVE, float *_insVN, float *_insVD, double *_insLng, double *_insLat, float *_insAE, float *_insAN, float *_insAD, int16_t *_prope1ler1, int16_t *_prope1ler2, int16_t *_prope1ler3, int16_t *_prope1ler4 ,uint8_t *_cmd,
						uint8_t *_light, uint8_t *_elevator, float *_insWz, float *_insWx, float *_insWy, float *_batteryVoltage, uint8_t *_batteryLeft, float *_batteryTemp, uint32_t *_leak, uint8_t *_batteryEmerg, uint8_t *_AddedBatteryEmerg, uint8_t *_propellerEmerg, uint8_t *_nodeEmerg, uint8_t *_estarFd, uint8_t *_paoZaiFd, uint8_t *_sensorSwitchFd, uint8_t *_shiFangFd)
{
	*_mode = _MSG_PAYLOAD_NON_CONST(rxmsg)[0];
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[1]>>7)
		*_binocularX = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[1]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[2])/1000.0f;
	else
		*_binocularX = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[1]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[2])/1000.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[3]>>7)
		*_binocularY = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[3]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[4])/1000.0f;
	else
		*_binocularY = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[3]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[4])/1000.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[5]>>7)
		*_binocularZ = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[5]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[6])/1000.0f;
	else
		*_binocularZ = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[5]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[6])/1000.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[7]>>7)
		*_binocularHeel = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[7]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[8])/100.0f;
	else
		*_binocularHeel = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[7]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[8])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[9]>>7)
		*_binocularPitch = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[9]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[10])/100.0f;
	else
		*_binocularPitch = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[9]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[10])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[11]>>7)
		*_binocularYaw = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[11]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[12])/100.0f;
	else
		*_binocularYaw = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[11]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[12])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[13]>>7)
		*_usblX = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[13]&0x7f)<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[14]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[15]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[16])/1000.0f;
	else
		*_usblX = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[13]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[14]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[15]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[16])/1000.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[17]>>7)
		*_usblY = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[17]&0x7f)<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[18]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[19]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[20])/1000.0f;
	else
		*_usblY = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[17]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[18]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[19]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[20])/1000.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[21]>>7)
		*_usblZ = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[21]&0x7f)<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[22]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[23]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[24])/1000.0f;
	else
		*_usblZ = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[21]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[22]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[23]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[24])/1000.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[25]>>7)
		*_usblHeel = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[25]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[26])/100.0f;
	else
		*_usblHeel = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[25]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[26])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[27]>>7)
		*_usblPitch = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[27]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[28])/100.0f;
	else
		*_usblPitch = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[27]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[28])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[29]>>7)
		*_usblYaw = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[29]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[30])/100.0f;
	else
		*_usblYaw = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[29]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[30])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[31]>>7)
		*_usblLng = -1.0*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[31]&0x7f)<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[32]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[33]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[34]))/11930464.7111;
	else
		*_usblLng = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[31]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[32]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[33]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[34]))/11930464.7111;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[35]>>7)
		*_usblLat = -1.0*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[35]&0x7f)<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[36]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[37]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[38]))/11930464.7111;
	else
		*_usblLat = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[35]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[36]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[37]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[38]))/11930464.7111;
	
	*_height = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[39]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[40]))/100.0f;
	*_depth = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[41]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[42]))/100.0f;
	*_insCourse = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[43]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[44]))/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[45]>>7)
		*_insPitch = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[45]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[46])/100.0f;
	else
		*_insPitch = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[45]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[46])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[47]>>7)
		*_insHeel = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[47]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[48])/100.0f;
	else
		*_insHeel = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[47]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[48])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[49]>>7)
		*_insVE = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[49]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[50])/100.0f;
	else
		*_insVE = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[49]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[50])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[51]>>7)
		*_insVN = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[51]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[52])/100.0f;
	else
		*_insVN = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[51]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[52])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[53]>>7)
		*_insVD = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[53]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[54])/100.0f;
	else
		*_insVD = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[53]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[54])/100.0f;
	
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[55]>>7)
		*_usblLng = -1.0*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[55]&0x7f)<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[56]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[57]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[58]))/11930464.7111;
	else
		*_usblLng = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[55]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[56]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[57]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[58]))/11930464.7111;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[59]>>7)
		*_insLng = -1.0*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[59]&0x7f)<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[60]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[61]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[62]))/11930464.7111;
	else
		*_insLat = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[59]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[60]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[61]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[62]))/11930464.7111;
	
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[63]>>7)
		*_insAE = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[63]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[64])/100.0f;
	else
		*_insAE = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[63]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[64])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[65]>>7)
		*_insAN = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[65]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[66])/100.0f;
	else
		*_insAN = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[65]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[66])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[67]>>7)
		*_insAD = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[67]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[68])/100.0f;
	else
		*_insAD = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[67]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[68])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[69]>>7)
		*_prope1ler1 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[69]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[70]);
	else
		*_prope1ler1 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[69]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[70]);
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[71]>>7)
		*_prope1ler2 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[71]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[72]);
	else
		*_prope1ler2 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[71]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[72]);
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[73]>>7)
		*_prope1ler3 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[73]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[74]);
	else
		*_prope1ler3 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[73]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[74])/10.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[75]>>7)
		*_prope1ler4 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[75]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[76]);
	else
		*_prope1ler4 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[75]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[76]);
	*_cmd = _MSG_PAYLOAD_NON_CONST(rxmsg)[77];
	*_light = _MSG_PAYLOAD_NON_CONST(rxmsg)[78];
	*_elevator = _MSG_PAYLOAD_NON_CONST(rxmsg)[79];
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[80]>>7)
		*_insWz = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[80]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[81])/100.0f;
	else
		*_insWz = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[80]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[81])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[82]>>7)
		*_insWx = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[82]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[83])/100.0f;
	else
		*_insWx = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[82]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[83])/100.0f;	
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[84]>>7)	
		*_insWy = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[84]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[85])/100.0f;
	else
		*_insWy = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[84]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[85])/100.0f;
	*_batteryVoltage = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[86]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[87])/1000.0f;
	*_batteryLeft = _MSG_PAYLOAD_NON_CONST(rxmsg)[88];
	*_batteryTemp = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[89]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[90])/10.0f;
	*_leak = (_MSG_PAYLOAD_NON_CONST(rxmsg)[91]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[92]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[93]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[94]);
	*_batteryEmerg = _MSG_PAYLOAD_NON_CONST(rxmsg)[95];
	*_AddedBatteryEmerg = _MSG_PAYLOAD_NON_CONST(rxmsg)[96];
	*_propellerEmerg = _MSG_PAYLOAD_NON_CONST(rxmsg)[97];
	*_nodeEmerg = _MSG_PAYLOAD_NON_CONST(rxmsg)[98];
	*_estarFd = _MSG_PAYLOAD_NON_CONST(rxmsg)[99];
	*_paoZaiFd = _MSG_PAYLOAD_NON_CONST(rxmsg)[100];
	*_sensorSwitchFd = _MSG_PAYLOAD_NON_CONST(rxmsg)[101];
	*_shiFangFd = _MSG_PAYLOAD_NON_CONST(rxmsg)[102];
}
						
 /**
 * @brief 获得控制指令
 * @param _mode 控制模式：0xFF-默认，00-基本控制，01-外环控制，02-直接控制
 * @param _statusFd 状态反馈：0xFF-默认，0x03-外环控制启动对接，0x04-外环控制终止对接
 * @param _propeller 航速指令：-100-100占空比
 * @param _course 航向指令：0-360
 * @param _depth 深度/高度指令：0-500
 * @param _propeller1 -4000-4000
 * @param _propeller2 -4000-4000
 * @param _propeller3 -4000-4000
 * @param _propeller4 -4000-4000
 * @param _ligth 
 * @param _elevator 
 * @param _rudder1 舵角1，-43-43度 
 * @param _rudder2 舵角2，-43-43度
 * @param _rudder3 舵角3，-43-43度
 * @param _rudder4 舵角4，-43-43度
 * @param _sensorSwitch
 * @param _paoZai
 * @param _shiFang
 */
void message_get_control_cmd(uint8_t *_mode, uint8_t *_statusFd, int8_t *_propeller, float *_course, float *_depth,
							  int16_t *_propeller1, int16_t *_propeller2, int16_t *_propeller3, int16_t *_propeller4,
							  uint8_t *_ligth, uint8_t *_elevator, float *_rudder1, float *_rudder2, float *_rudder3, float *_rudder4,
							  uint16_t *_sensorSwitch, uint8_t *_paoZai, uint8_t *_shiFang)
{
	*_mode = _MSG_PAYLOAD_NON_CONST(rxmsg)[0];
	*_statusFd = _MSG_PAYLOAD_NON_CONST(rxmsg)[1];
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[2]>>7)
		*_propeller = -1*(_MSG_PAYLOAD_NON_CONST(rxmsg)[2]&0x7f);
	else
		*_propeller = _MSG_PAYLOAD_NON_CONST(rxmsg)[2];
	*_course = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[3]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[4]))/10.f;
	*_depth = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[5]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[6]))/10.f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[7]>>7)
		*_propeller1 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[7]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[8]);
	else
		*_propeller1 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[7]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[8]);
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[9]>>7)
		*_propeller2 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[9]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[10]);
	else
		*_propeller2 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[9]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[10]);
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[11]>>7)
		*_propeller3 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[11]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[12]);
	else
		*_propeller3 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[11]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[12]);
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[13]>>7)
		*_propeller4 = -1*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[13]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[14]);
	else
		*_propeller4 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[13]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[14]);
	*_ligth = _MSG_PAYLOAD_NON_CONST(rxmsg)[15];
	*_elevator = _MSG_PAYLOAD_NON_CONST(rxmsg)[16];	
	
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[17]>>7)
		*_rudder1 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[17]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[18])/10.0f;
	else
		*_rudder1 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[17]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[18])/10.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[19]>>7)
		*_rudder2 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[19]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[20])/10.0f;
	else
		*_rudder2 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[19]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[20])/10.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[21]>>7)
		*_rudder3 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[21]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[22])/10.0f;
	else
		*_rudder3 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[21]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[22])/10.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[23]>>7)
		*_rudder4 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[23]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[24])/10.0f;
	else
		*_rudder4 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[23]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[24])/10.0f;
	*_sensorSwitch = _MSG_PAYLOAD_NON_CONST(rxmsg)[25];
	*_paoZai = _MSG_PAYLOAD_NON_CONST(rxmsg)[26];
	*_shiFang = _MSG_PAYLOAD_NON_CONST(rxmsg)[27];
}

void message_control_cmd_pack(uint8_t modeSet, uint8_t statusFd, int8_t propeller, float course, float depth,
							 int16_t propeller1,int16_t propeller2,int16_t propeller3,int16_t propeller4,
							 uint8_t light, uint8_t elevator,float rudder1,float rudder2, float rudder3, float rudder4,uint16_t sensorSwitch,
							 uint8_t paoZai, uint8_t shiFang)
{
	uint8_t buf[28];
	txmsg.len = 28;
	buf[0] = modeSet;
	buf[1] = statusFd;
	if(propeller>0)
		buf[2] = propeller;
	else
		buf[2] = (uint8_t)(abs(propeller))|0x80;
	buf[3] = (uint16_t)(course*10)>>8;
	buf[4] = (uint16_t)(course*10)&0xFF;
	buf[5] = (uint16_t)(depth*10)>>8;
	buf[6] = (uint16_t)(depth*10)&0xFF;
	if(propeller1<0)
	{
		buf[7] = ((uint16_t)(abs(propeller1))>>8)|0x80;
		buf[8] = (uint16_t)(abs(propeller1)) & 0xFF;
	}else{
		buf[7] = (uint16_t)(propeller1) >> 8;
		buf[8] = (uint16_t)(propeller1) & 0xFF;
	}
	if(propeller2<0)
	{
		buf[9] = ((uint16_t)(abs(propeller2))>>8)|0x80;
		buf[10] = (uint16_t)(abs(propeller2)) & 0xFF;
	}else{
		buf[9] = (uint16_t)(propeller2) >> 8;
		buf[10] = (uint16_t)(propeller2) & 0xFF;
	}
	if(propeller3<0)
	{
		buf[11] = ((uint16_t)(abs(propeller3))>>8)|0x80;
		buf[12] = (uint16_t)(abs(propeller3)) & 0xFF;
	}else{
		buf[11] = (uint16_t)(propeller3) >> 8;
		buf[12] = (uint16_t)(propeller3) & 0xFF;
	}
	if(propeller4<0)
	{
		buf[13] = ((uint16_t)(abs(propeller4))>>8)|0x80;
		buf[14] = (uint16_t)(abs(propeller4)) & 0xFF;
	}else{
		buf[13] = (uint16_t)(propeller4) >> 8;
		buf[14] = (uint16_t)(propeller4) & 0xFF;
	}
	buf[15] = light;
	buf[16] = elevator;
	if(rudder1<0)
	{
		buf[17] = (((uint16_t)(fabs(rudder1)*10)>>8)|0x80);
		buf[18] = (((uint16_t)(fabs(rudder1)*10)&0xFF));
	}else{
		buf[17] = ((uint16_t)(rudder1*10)>>8);
		buf[18] = (((uint16_t)(rudder1*10)&0xFF));
	}
	if(rudder2<0)
	{
		buf[19] = (((uint16_t)(fabs(rudder2)*10)>>8)|0x80);
		buf[20] = (((uint16_t)(fabs(rudder2)*10)&0xFF));
	}else{
		buf[19] = ((uint16_t)(rudder2*10)>>8);
		buf[20] = (((uint16_t)(rudder2*10)&0xFF));
	}
	if(rudder3<0)
	{
		buf[21] = (((uint16_t)(fabs(rudder3)*10)>>8)|0x80);
		buf[22] = (((uint16_t)(fabs(rudder3)*10)&0xFF));
	}else{
		buf[21] = ((uint16_t)(rudder3*10)>>8);
		buf[22] = (((uint16_t)(rudder3*10)&0xFF));
	}
	if(rudder4<0)
	{
		buf[23] = (((uint16_t)(fabs(rudder4)*10)>>8)|0x80);
		buf[24] = (((uint16_t)(fabs(rudder4)*10)&0xFF));
	}else{
		buf[23] = ((uint16_t)(rudder4*10)>>8);
		buf[24] = (((uint16_t)(rudder4*10)&0xFF));
	}
	buf[25] = sensorSwitch;
	buf[26] = paoZai;
	buf[27] = shiFang;
	memcpy(_MSG_PAYLOAD_NON_CONST(txmsg), buf, txmsg.len);
	message_cal_checksum(&txmsg);
}

void message_test_pack()
{
	uint8_t buf[3];
	txmsg.len = 3;
	buf[0] = 1;
	buf[1] = 2;
	buf[2] = 3;
	memcpy(_MSG_PAYLOAD_NON_CONST(txmsg), buf, txmsg.len);
	message_cal_checksum(&txmsg);
}
#endif // MESSAGE719_H
