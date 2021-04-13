/*                                                                                               
 * Filename: uuv_ctrl_msg_parser.h
 * Path: tcp_to_ros
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once

#include "byte_buffer.h"
#include "async_tcp_server.h"

struct uuv_ctrl_msg
{
    uint16_t head_ = 0xEBA2;
    uint16_t cnt_; // 0-65535
    uint8_t len_;
    
    uint8_t nav_mode_;
    uint8_t status_fb_;
    
    // used for control inspection
    uint8_t u_cmd_;
    uint16_t yaw_cmd_;
    uint16_t depth_cmd_; 

    uint16_t upper_port_; 
    uint16_t upper_starboard_; 
    uint16_t lower_port_; 
    uint16_t lower_starboard_;

    uint8_t light_;
    uint8_t lift_;

    uint16_t thruster_;
    uint16_t set1_ = 0xFFFF;
    uint16_t set2_ = 0xFFFF;
    uint16_t set3_ = 0xFFFF;

    uint8_t crc_;
    uint16_t tail_ = 0xEEA2;

    size_t total_len_ = 33;
}; 

class uuv_ctrl_msg_parser : public subscriber
{
public:
    uuv_ctrl_msg_parser() {}
    ~uuv_ctrl_msg_parser() {}

private:
    void parse(const std::string& msg)
    {
        uint8_t buffer[msg_.total_len_];

        for(size_t i = 0; i < msg_.total_len_; ++i)
        {
            buffer[i] = static_cast<uint8_t>(msg[i]);
        }
    }

    void deliver(const std::string& msg)
    {
        /* parse received control message */
        parse(msg);
    }

private:
    uuv_ctrl_msg msg_;
};

