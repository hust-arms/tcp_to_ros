/*                                                                                             
 * Filename: uuv_ctrl_msg_parser.h
 * Path: tcp_to_ros
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once

#include <stdint.h>
#include <string>
#include <string.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

typedef boost::shared_ptr<uint8_t> uint8_t_ptr;

/**
 * @brief Buffer filled with byte data
 */
class byte_buffer
{
public:
    /**
     * @brief constructor
     */
    byte_buffer(size_t len)
    {
        pos_ = 0; len_ = len;

        // request memeory
        // buffer_ = new uint8_t[len_];
        buffer_ = boost::make_shared<uint8_t>(new uint8_t(len_));

        memset(buffer_, 0, len_);
    }

    /**
     * @brief return buffer 
     */
    inline uint8_t_ptr getMsg()const {return buffer_;}

    /**
     * @brief get byte 
     */
    inline uint8_t getByte(size_t bit)const { return buffer_[bit]; }

    /**
     * @brief get length of buffer
     */
    inline size_t getLen()const {return len_;}

    /**
     * @brief append data of uint8
     */
    void append_uint8(uint8_t value)
    {
        buffer_[pos_++] = value;
    }

    /**
     * @brief append data of uint16
     */
    void append_uint16(uint16_t value)
    {
        buffer_[pos_++] = (uint8_t)(value >> 8);
        buffer_[pos_++] = (uint8_t)(value & 0xFF);
    }

    /**
     * @brief append data of uint32
     */
    void append_uint32(uint32_t value)
    {
        buffer_[pos_++] = (uint8_t)(value >> 24);
        buffer_[pos_++] = (uint8_t)((value >> 16) & 0xFF);
        buffer_[pos_++] = (uint8_t)((value >> 8) & 0xFF);
        buffer_[pos_++] = (uint8_t)(value & 0xFF);
    }

    /**
     * @brief turn uint8 to uint16
     */
    uint16_t uint8_to_uint16(int data_pos)
    {
        return static_cast<uint16_t>((buffer_[data_pos] | (buffer_[data_pos + 1] << 8)) & 0xFF);
    }

    /**
     * @brief turn uint8 to uint32
     */
    uint32_t uint8_to_uint32(int data_pos)
    {
        return static_cast<uint32_t>((buffer_[data_pos] | (buffer_[data_pos + 1] << 8) | 
                                                         (buffer_[data_pos + 2] << 16) |
                                                         (buffer_[data_pos + 3] << 24)) & 0xFF);
    }

private:
    uint8_t_ptr buffer_;
    size_t pos_;
    size_t len_;
};


