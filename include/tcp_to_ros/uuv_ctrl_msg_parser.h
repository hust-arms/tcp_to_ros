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

/**
 * @brief UUV control message
 */
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
    uint16_t lower_starboard_;
    uint16_t lower_port_; 

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

/**
 * @brief UUV control message parser
 */
class uuv_ctrl_msg_parser : public subscriber
{
public:
    uuv_ctrl_msg_parser() {}
    ~uuv_ctrl_msg_parser() {}

    /**
     * @brief return input control parameters
     * @brief upper_port upper left from tail of X type 
     * @brief upper_starboard upper right from tail of X type 
     * @brief lower_starboard lower right from tail of X type 
     * @brief lower_port lower left from tail of X type 
     * @brief thruster rpm of thruster
     */
    void get_ctrl_msg(double& upper_port, double& upper_starboard, double& lower_starboard, double& lower_port, double& thruster)
    {
        upper_port = msg_.upper_port_;
        upper_starboard = msg_.upper_starboard_;
        lower_starboard = msg_.lower_starboard_;
        lower_port = msg_.lower_port_;
        thruster = msg_.thruster_;
    }

private:
    /**
     * @brief parse control parameters from message
     */
    void parse(const std::string& msg)
    {
        if(msg.length() >= msg_.total_len_)
        {
            byte_buffer buffer(msg.length());
            
            for(size_t i = 0; i < msg.length(); ++i)
            {
                buffer.append_uint8(static_cast<uint8_t>(msg[i]));
            }
            
            uint8_t_ptr data = buffer.getMsg();
            
            size_t head_pos = 0;
            
            // find msg head
            for(size_t head_pos = 0; head_pos < msg.length(); ++head_pos)
            {
                if(data[head_pos] == msg_.head_)
                {
                    break;
                }
            }

            if(msg.length() - head_pos >= msg_.total_len_)
            {
                // crc check

                // parse
                head_pos += 2;
                
                msg_.cnt_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
                
                msg_.len_ = buffer.getByte(head_pos);
                ++head_pos;
                
                msg_.nav_mode_ = buffer.getByte(head_pos);
                ++head_pos;
                
                msg_.status_fb_ = buffer.getByte(head_pos);
                ++head_pos;
                
                msg_.u_cmd_ = buffer.getByte(head_pos);
                ++head_pos;
                
                msg_.yaw_cmd_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
                
                msg_.depth_cmd_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
                
                msg_.upper_port_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
                
                msg_.upper_starboard_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
                
                msg_.lower_starboard_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
                
                msg_.lower_port_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
                
                msg_.light_ = buffer.getByte(head_pos);
                ++head_pos;
                
                msg_.lift_ = buffer.uint8_to_uint16(head_pos);
                head_pos += 2;
            }
            else
                return;
        }
    }

    /**
     * @brief inherit interface from subscriber, which is used to receive message
     */
    void deliver(const std::string& msg)
    {
        /* parse received control message */
        parse(msg);
    }

private:
    uuv_ctrl_msg msg_;
};

