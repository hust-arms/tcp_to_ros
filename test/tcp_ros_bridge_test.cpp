/*                                                                                   
 * Filename: tcp_ros_bridge.cpp
 * Path: tcp_ros_bridge
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <cstdio>
#include <iostream>
#include "tcp_ros_bridge/tcp_ros_bridge.h"

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cerr << "[tcp_ros_bridge_test]: Usage: tcp_ros_bridge_test <auv_name> <port>" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "tcp_ros_bridge_test");

    boost::asio::io_service io_service;
    tcp_ros_bridge::uuv_tcp_ros_bridge bridge(argv[1], io_service, atoi(argv[2]));

    io_service.run();

    ros::spin();

    return 0;
}

