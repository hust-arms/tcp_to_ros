/*                                                                                             
 * Filename: crc_check.h
 * Path: tcp_to_ros
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once

#include <stdint.h>

#define X25_INIT_CRC 0xFFFF
#define X25_VALIDATE_CRC 0xF0B8

/**
 * @brief crc check
 * @param data data buffer to hash
 * @param crc_accum the already accumulated check number
 */
static void crc_check(uint8_t data, uint16_t* crc_accum)
{
    uint8_t tmp;

    tmp = data ^ static_cast<uint8_t>(*crc_accum & 0xFF);
    tmp ^= (tmp << 4);
    *crc_accum = (*crc_accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

/**
 * @brief crc initialize
 * @brief target accumulated check number 
 */
static void crc_init(uint16_t* crc_accum)
{
    *crc_accum = X25_INIT_CRC;
}

/**
 * @brief crc calculate
 * @param target data buffer
 * @param len target length of input buffer
 */
static uint16_t crc_calc(const uint8_t* buffer, uint16_t len)
{
    uint16_t crc_tmp;
    crc_init(&crc_tmp);

    while(--len){
        crc_check(*buffer++, &crc_tmp);
    }
    return crc_tmp;
}


