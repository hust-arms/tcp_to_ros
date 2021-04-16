#include "tcp_ros_bridge/message719.h"

message_status_t status;
message_message_t rxmsg;
message_message_t txmsg;

/**
 * @brief 从字节流中提取出完整的消息结构，将接受到的字节流一个字节一个字节的解析
 * @param c 待解析的字节
 * @return 解析结果（0-还没解析成完成的消息结构，1-解析出完整的消息结构，2-解析出完整的消息结构但校验错误）
 */
uint8_t message_parse(uint8_t c)
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
            printf("GOT_HEADER1\n");
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
				//status.parse_state = MESSAGE_PARSE_STATE_GOT_BAD_CRC;
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
				//status.parse_state = MESSAGE_PARSE_STATE_GOT_CRC;
			}
		}
		else
		{
#ifdef MDebug
			printf("GOT_ERROR_FOOTER\n");
#endif		
			//status.parse_state = MESSAGE_PARSE_STATE_GOT_BAD_CRC;
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
    foot[0] = 0xEE;
    foot[1] = 0x2A;

    return MESSAGE_NUM_NON_PAYLOAD_BYTES + MESSAGE_NUM_FOOTER_BYTES + (uint16_t)txmsg.len;
}

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
    rxmsg.header1 = MESSAGE_HEADER1;
	rxmsg.header2 = MESSAGE_RXHEADER2;
    rxmsg.seqH = 0;
    rxmsg.seqL = 0;
    rxmsg.len = 0;
    
	txmsg.header1 = MESSAGE_HEADER1;
	txmsg.header2 = MESSAGE_TXHEADER2;
}

void message_test_pack()
{
	uint8_t buf[3];
	txmsg.len = 3;
	buf[0] = 1;
	buf[1] = 2;
	buf[2] = 3;
	memcpy((void *)(txmsg.payload64), buf, txmsg.len);
	message_cal_checksum(&txmsg);
}

void message_status_pack(uint8_t mode, float binocularX, float binocularY, float binocularZ,float binocularHeel,float binocularPitch,float binocularYaw,float usblX,float usblY,float usblZ,float usblHeel,float usblPitch,float usblYaw,
						float height, float depth, float insCourse, float insPitch, float insHeel, float insVE, float insVN, float insVD, double insLng, double insLat, float insAE, float insAN, float insAD, float rudder1, float rudder2, float rudder3, float rudder4 ,uint8_t cmd,
						uint8_t light, uint8_t elevator)
{
	uint8_t buf[84];
	txmsg.len = 84;
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
	buf[31] = ((uint16_t)(height*100) >> 8);
	buf[32] = ((uint16_t)(height*100) & 0xFF);
	buf[33] = ((uint16_t)(depth*100) >> 8);
	buf[34] = ((uint16_t)(depth*100) & 0xFF);
	buf[35] = ((uint16_t)(insCourse*100) >> 8);
	buf[36] = ((uint16_t)(insCourse*100) & 0xFF);
	if(insPitch>0)
	{
		buf[37] = ((uint16_t)(insPitch*100) >> 8);
		buf[38] = ((uint16_t)(insPitch*100) & 0xFF);
	}else{
		buf[37] = (((uint16_t)(fabs(insPitch*100)) >> 8)|0x80);
		buf[38] = ((uint16_t)(fabs(insPitch*100)) & 0xFF);
	}
	if(insHeel>0)
	{
		buf[39] = ((uint16_t)(insHeel*100) >> 8);
		buf[40] = ((uint16_t)(insHeel*100) & 0xFF);
	}else{
		buf[39] = (((uint16_t)(fabs(insHeel*100)) >> 8)|0x80);
		buf[40] = ((uint16_t)(fabs(insHeel*100)) & 0xFF);
	}
	if(insVE>0)
	{
		buf[41] = ((uint16_t)(insVE*100) >> 8);
		buf[42] = ((uint16_t)(insVE*100) & 0xFF);
	}else{
		buf[41] = (((uint16_t)(fabs(insVE*100)) >> 8)|0x80);
		buf[42] = ((uint16_t)(fabs(insVE*100)) & 0xFF);
	}
	if(insVN>0)
	{
		buf[43] = ((uint16_t)(insVN*100) >> 8);
		buf[44] = ((uint16_t)(insVN*100) & 0xFF);
	}else{
		buf[43] = (((uint16_t)(fabs(insVN*100)) >> 8)|0x80);
		buf[44] = ((uint16_t)(fabs(insVN*100)) & 0xFF);
	}
	if(insVD>0)
	{
		buf[45] = ((uint16_t)(insVD*100) >> 8);
		buf[46] = ((uint16_t)(insVD*100) & 0xFF);
	}else{
		buf[45] = (((uint16_t)(fabs(insVD*100)) >> 8)|0x80);
		buf[46] = ((uint16_t)(fabs(insVD*100)) & 0xFF);
	}
	uint32_t uintLng = insLng*10000000;
	buf[47] = (uintLng >> 24);
	buf[48] = ((uintLng >> 16)&0xFF);
	buf[49] = ((uintLng >> 8)&0xFF);
	buf[50] = (uintLng & 0xFF);
	uint32_t uintLat = insLat*10000000;
	buf[51] = (uintLat >> 24);
	buf[52] = ((uintLat >> 16)&0xFF);
	buf[53] = ((uintLat >> 8)&0xFF);
	buf[54] = (uintLat & 0xFF);
	if(insAE>0)
	{
		buf[55] = ((uint16_t)(insAE*100) >> 8);
		buf[56] = ((uint16_t)(insAE*100) & 0xFF);
	}else{
		buf[55] = (((uint16_t)(fabs(insAE*100)) >> 8)|0x80);
		buf[56] = ((uint16_t)(fabs(insAE*100)) & 0xFF);
	}
	if(insAN>0)
	{
		buf[57] = ((uint16_t)(insAN*100) >> 8);
		buf[58] = ((uint16_t)(insAN*100) & 0xFF);
	}else{
		buf[57] = (((uint16_t)(fabs(insAN*100)) >> 8)|0x80);
		buf[58] = ((uint16_t)(fabs(insAN*100)) & 0xFF);
	}
	if(insAD>0)
	{
		buf[59] = ((uint16_t)(insAD*100) >> 8);
		buf[60] = ((uint16_t)(insAD*100) & 0xFF);
	}else{
		buf[59] = (((uint16_t)(fabs(insAD*100)) >> 8)|0x80);
		buf[60] = ((uint16_t)(fabs(insAD*100)) & 0xFF);
	}
	if(rudder1>0)
	{
		buf[61] = ((uint16_t)(rudder1*10) >> 8);
		buf[62] = ((uint16_t)(rudder1*10) & 0xFF);
	}else{
		buf[61] = (((uint16_t)(fabs(rudder1*10)) >> 8)|0x80);
		buf[62] = ((uint16_t)(fabs(rudder1*10)) & 0xFF);
	}
	if(rudder2>0)
	{
		buf[63] = ((uint16_t)(rudder2*10) >> 8);
		buf[64] = ((uint16_t)(rudder2*10) & 0xFF);
	}else{
		buf[63] = (((uint16_t)(fabs(rudder2*10)) >> 8)|0x80);
		buf[64] = ((uint16_t)(fabs(rudder2*10)) & 0xFF);
	}
	if(rudder3>0)
	{
		buf[65] = ((uint16_t)(rudder3*10) >> 8);
		buf[66] = ((uint16_t)(rudder3*10) & 0xFF);
	}else{
		buf[65] = (((uint16_t)(fabs(rudder3*10)) >> 8)|0x80);
		buf[66] = ((uint16_t)(fabs(rudder3*10)) & 0xFF);
	}
	if(rudder4>0)
	{
		buf[67] = ((uint16_t)(rudder4*10) >> 8);
		buf[68] = ((uint16_t)(rudder4*10) & 0xFF);
	}else{
		buf[67] = (((uint16_t)(fabs(rudder4*10)) >> 8)|0x80);
		buf[68] = ((uint16_t)(fabs(rudder4*10)) & 0xFF);
	}
	buf[69] = cmd;
	buf[70] = light;
	buf[71] = elevator;
	for(int i=0;i<12;i++)
		buf[72+i] = 0xFF;
	memcpy(_MSG_PAYLOAD_NON_CONST(txmsg), buf, txmsg.len);
	message_cal_checksum(&txmsg);
}

void message_get_status(uint8_t *_mode, float *_binocularX, float *_binocularY, float *_binocularZ,float *_binocularHeel,float *_binocularPitch,float *_binocularYaw,float *_usblX,float *_usblY,float *_usblZ,float *_usblHeel,float *_usblPitch,float *_usblYaw,
						float *_height, float *_depth, float *_insCourse, float *_insPitch, float *_insHeel, float *_insVE, float *_insVN, float *_insVD, double *_insLng, double *_insLat, float *_insAE, float *_insAN, float *_insAD, float *_rudder1, float *_rudder2, float *_rudder3, float *_rudder4 ,uint8_t *_cmd,
						uint8_t *_light, uint8_t *_elevator)
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
	*_height = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[31]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[32]))/100.0f;
	*_depth = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[33]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[34]))/100.0f;
	*_insCourse = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[35]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[36]))/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[37]>>7)
		*_insPitch = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[37]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[38])/100.0f;
	else
		*_insPitch = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[37]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[38])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[39]>>7)
		*_insHeel = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[39]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[40])/100.0f;
	else
		*_insHeel = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[39]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[40])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[41]>>7)
		*_insVN = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[41]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[42])/100.0f;
	else
		*_insVN = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[41]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[42])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[43]>>7)
		*_insVE = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[43]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[44])/100.0f;
	else
		*_insVE = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[43]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[44])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[45]>>7)
		*_insVD = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[45]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[46])/100.0f;
	else
		*_insVD = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[45]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[46])/100.0f;
	*_insLng = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[47]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[48]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[49]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[50]))/10000000.0f;
	*_insLat = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[51]<<24)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[52]<<16)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[53]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[54]))/10000000.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[55]>>7)
		*_insAE = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[55]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[56])/100.0f;
	else
		*_insAE = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[55]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[56])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[57]>>7)
		*_insAN = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[57]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[58])/100.0f;
	else
		*_insAN = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[57]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[58])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[59]>>7)
		*_insAD = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[59]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[60])/100.0f;
	else
		*_insAD = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[59]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[60])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[61]>>7)
		*_rudder1 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[61]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[62])/100.0f;
	else
		*_rudder1 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[61]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[62])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[63]>>7)
		*_rudder2 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[63]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[64])/100.0f;
	else
		*_rudder2 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[63]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[64])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[65]>>7)
		*_rudder3 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[65]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[66])/100.0f;
	else
		*_rudder3 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[65]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[66])/100.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[67]>>7)
		*_rudder4 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[67]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[68])/100.0f;
	else
		*_rudder4 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[67]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[68])/100.0f;
	*_cmd = _MSG_PAYLOAD_NON_CONST(rxmsg)[69];
	*_light = _MSG_PAYLOAD_NON_CONST(rxmsg)[70];
	*_elevator = _MSG_PAYLOAD_NON_CONST(rxmsg)[71];
}

void message_control_cmd_pack(uint8_t mode, uint8_t statusFd, uint8_t vel, float course, float depth, float rudder1,
							 float rudder2, float rudder3, float rudder4, uint8_t light, uint8_t elevator)
{
	uint8_t buf[25];
	txmsg.len = 25;
	buf[0] = mode;
	buf[1] = statusFd;
	buf[2] = vel;
	buf[3] = ((uint16_t)(course*10)>>8);
	buf[4] = ((uint16_t)(course*10)&0xFF);
	buf[5] = ((uint16_t)(depth*10)>>8);
	buf[6] = ((uint16_t)(depth*10)&0xFF);
	if(rudder1<0)
	{
		buf[7] = (((uint16_t)(fabs(rudder1)*10)>>8)|0x80);
		buf[8] = (((uint16_t)(fabs(rudder1)*10)&0xFF));
	}else{
		buf[7] = ((uint16_t)(rudder1*10)>>8);
		buf[8] = (((uint16_t)(rudder1*10)&0xFF));
	}
	if(rudder2<0)
	{
		buf[9] = (((uint16_t)(fabs(rudder2)*10)>>8)|0x80);
		buf[10] = (((uint16_t)(fabs(rudder2)*10)&0xFF));
	}else{
		buf[9] = ((uint16_t)(rudder2*10)>>8);
		buf[10] = (((uint16_t)(rudder2*10)&0xFF));
	}
	if(rudder3<0)
	{
		buf[11] = (((uint16_t)(fabs(rudder3)*10)>>8)|0x80);
		buf[12] = (((uint16_t)(fabs(rudder3)*10)&0xFF));
	}else{
		buf[11] = ((uint16_t)(rudder3*10)>>8);
		buf[12] = (((uint16_t)(rudder3*10)&0xFF));
	}
	if(rudder4<0)
	{
		buf[13] = (((uint16_t)(fabs(rudder4)*10)>>8)|0x80);
		buf[14] = (((uint16_t)(fabs(rudder4)*10)&0xFF));
	}else{
		buf[13] = ((uint16_t)(rudder4*10)>>8);
		buf[14] = (((uint16_t)(rudder4*10)&0xFF));
	}
	buf[15] = light;
	buf[16] = elevator;
	for(int i=0; i<8; i++)
		buf[17+i] = 0xFF;
	memcpy(_MSG_PAYLOAD_NON_CONST(txmsg), buf, txmsg.len);
	message_cal_checksum(&txmsg);
}

/**
 * @brief 获得控制指令
 * @param _mode 控制模式：0xFF-默认，00-基本控制，01-外环控制，02-直接控制
 * @param _statusFd 状态反馈：0xFF-默认，0x03-外环控制启动对接，0x04-外环控制终止对接
 * @param _vel 航速指令：0-100占空比
 * @param _course 航向指令：0-360
 * @param _depth 深度/高度指令：0-500
 * @param _rudder1 舵角1，度
 * @param _rudder2 舵角2，度
 * @param _rudder3 舵角3，度
 * @param _rudder4 舵角4，度
 * @param _ligth 
 * @param _elevator
 */

 void message_get_control_cmd(uint8_t *_mode, uint8_t *_statusFd, uint8_t *_vel, float *_course, float *_depth, float *_rudder1,
							 float *_rudder2, float *_rudder3, float *_rudder4, uint8_t *_ligth, uint8_t *_elevator)
 {
	 *_mode = _MSG_PAYLOAD_NON_CONST(rxmsg)[0];
	 *_statusFd = _MSG_PAYLOAD_NON_CONST(rxmsg)[1];
	 *_vel = _MSG_PAYLOAD_NON_CONST(rxmsg)[2];
	 *_course = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[3]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[4]))/0.1f;
	 *_depth = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[5]<<8)|(_MSG_PAYLOAD_NON_CONST(rxmsg)[6]))/0.1f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[7]>>7)
		*_rudder1 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[7]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[8])/10.0f;
	else
		*_rudder1 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[7]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[8])/10.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[9]>>7)
		*_rudder2 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[9]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[10])/10.0f;
	else
		*_rudder2 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[9]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[10])/10.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[11]>>7)
		*_rudder3 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[11]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[12])/10.0f;
	else
		*_rudder3 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[11]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[12])/10.0f;
	if(_MSG_PAYLOAD_NON_CONST(rxmsg)[13]>>7)
		*_rudder4 = -1.0f*(((_MSG_PAYLOAD_NON_CONST(rxmsg)[13]&0x7f)<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[14])/10.0f;
	else
		*_rudder4 = ((_MSG_PAYLOAD_NON_CONST(rxmsg)[13]<<8)|_MSG_PAYLOAD_NON_CONST(rxmsg)[14])/10.0f;
	*_ligth = _MSG_PAYLOAD_NON_CONST(rxmsg)[15];
	*_elevator = _MSG_PAYLOAD_NON_CONST(rxmsg)[16];
 }

