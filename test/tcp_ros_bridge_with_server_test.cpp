/*                                                                                   
 * Filename: tcp_ros_bridge_with_server.cpp
 * Path: tcp_ros_bridge_with_server
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <cstdio>
#include <iostream>
#include "tcp_ros_bridge/tcp_ros_bridge_with_server.h"

boost::asio::io_service io_service;

void run_service()
{
    std::cout << "[tcp_ros_bridge_with_server_test]: run service" << std::endl;
    io_service.run();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tcp_ros_bridge_with_server_test");

    if(argc != 3)
    {
        std::cerr << "[tcp_ros_bridge_with_server_test]: Usage: tcp_ros_bridge_with_server_test <auv_name> <port>" << std::endl;
        return -1;
    }

    tcp::endpoint listen_endpoint(tcp::v4(), atoi(argv[2]));

    tcp_ros_bridge::server bridge(argv[1], io_service, listen_endpoint);

    boost::thread* io_service_th = new boost::thread(boost::bind(&run_service));

    ros::spin();

    if(io_service_th)
        delete io_service_th;

    return 0;
}

