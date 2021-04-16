/*                                                                                     
 * Filename: uuv_ctrl_msg_parser_test.cpp
 * Path: tcp_ros_bridge
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include "tcp_ros_bridge/crc_check.h"
#include "tcp_ros_bridge/uuv_ctrl_msg_parser.h"

#include <iostream>

int main()
{
    // test input UUV control message
    uint16_t head = 0xEBA2;
    uint16_t cnt = 10;
    uint8_t len = 25;
    uint8_t nav_mode = 0xFF;
    uint8_t status_fb = 0xFF;

    uint8_t u_cmd = 0;
    uint16_t yaw_cmd = static_cast<uint16_t>(100.8);
    uint16_t depth_cmd = static_cast<uint16_t>(100.5);

    uint16_t upper_port = static_cast<uint16_t>(12.5);
    uint16_t upper_starboard = static_cast<uint16_t>(14.5);
    uint16_t lower_starboard = static_cast<uint16_t>(14.5);
    uint16_t lower_port = static_cast<uint16_t>(12.5);

    uint8_t light = 0x00;
    uint8_t lift = 0x01;

    uint16_t set1 = static_cast<uint16_t>(1400);
    uint16_t set2 = 0xFFFF;
    uint16_t set3 = 0xFFFF;
    uint16_t set4 = 0xFFFF;

    uint16_t tail = 0xEE2A;

    size_t total_len = 33;

    byte_buffer buffer(total_len);
    buffer.append_uint16(head);
    buffer.append_uint16(cnt);
    buffer.append_uint8(len);
    buffer.append_uint8(nav_mode);
    buffer.append_uint8(status_fb);
    buffer.append_uint8(u_cmd);
    buffer.append_uint16(yaw_cmd);
    buffer.append_uint16(depth_cmd);
    buffer.append_uint16(upper_port);
    buffer.append_uint16(upper_starboard);
    buffer.append_uint16(lower_starboard);
    buffer.append_uint16(lower_port);
    buffer.append_uint8(light);
    buffer.append_uint8(lift);
    buffer.append_uint16(set1);
    buffer.append_uint16(set2);
    buffer.append_uint16(set3);
    buffer.append_uint16(set4);

    uint8_t crc = crc_calc(&buffer.getMsg()[5], len);
    buffer.append_uint8(crc);
    buffer.append_uint16(tail);

    size_t msg_len = buffer.getLen();
    std::string ctrl_msg(msg_len, 0);

    for(size_t i = 0; i < msg_len; ++i)
    {
        ctrl_msg[i] = static_cast<char>(buffer.getByte(i));
    }

    std::cout << "test UUV control message" << std::endl;

    for(size_t i = 0; i < msg_len; ++i)
    {
        std::cout << (ctrl_msg[i] & 0xFF) << std::endl;
    }

    std::cout << "parse test" << std::endl;

    uuv_ctrl_msg_parser parser;
    parser.deliver_test(ctrl_msg);

    double up, us, ls, lp, thruster;
    parser.get_ctrl_msg(up, us, ls, lp, thruster);

    std::cout << "parser result" << std::endl;

    std::cout << up << " " << us << " " << ls << " " << lp << " " << thruster << std::endl;

    return 0;
}

