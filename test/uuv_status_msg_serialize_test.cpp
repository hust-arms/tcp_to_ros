/*                                                                                     
 * Filename: uuv_status_msg_serialize.cpp
 * Path: tcp_ros_bridge
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <exception>

#include "tcp_ros_bridge/crc_check.h"
#include "tcp_ros_bridge/uuv_status_msg_serialize.h"

#include <iostream>

int main()
{
    double height = 90.0;
    double depth = 10.0;
    double x = 40.0;
    double y = 5.0;
    double z = -10.0;
    double roll = 1.0;
    double pitch = 5.0;
    double yaw = -5.0;
    double u = -3.0 * 0.514;
    double v = 0.0;
    double w = 0.05;
    double up = 12.5;
    double us = 14.5;
    double ls = 14.5;
    double lp = 12.5;
    int thruster = 1400;

    std::string message;
    uuv_status_msg_serialize serializer;
    try
    {
        message = serializer.serialize_test(height, depth, x, y, z, roll, pitch, yaw, u, v, w, 
                             up, us, ls, lp, thruster);
    }
    catch(std::exception& e)
    {
        std::cout << "serialize error: " << e.what() << std::endl;
    }

    for(size_t i = 0; i < message.length(); ++i)
    {
        std::cout << (static_cast<uint8_t>(message[i]) & 0xFF) << std::endl;
    }

    return 0;
}

