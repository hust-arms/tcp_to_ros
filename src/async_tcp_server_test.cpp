/*                                                                             
 * Filename: async_tcp_server_test.cpp
 * Path: tcp_ros_bridge
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include "tcp_ros_bridge/async_tcp_server.h"
#include <iostream>
#include <exception>

/**
 * @brief subscriber class used for server test
 */
class test_subscriber : public subscriber
{
public:
    /**
     * @brief test deliver interface 
     */
    void deliver(const std::string& msg) override
    {
        std::cout << "[test_subscriber]: recv: " << msg << std::endl;
    }

};

int main(int argc, char* argv[])
{
    try
    {
        if(argc != 2)
        {
            std::cerr << "[async_tcp_server_test_node]: Usage: async_tcp_server_test <listen_port>" << std::endl;
        }

        boost::asio::io_service io_service;
        tcp::endpoint listen_endpoint(tcp::v4(), atoi(argv[1]));

        subscriber_ptr msg_sub(new test_subscriber());
        std::cout << "[async_tcp_server_test_node]: create TCP server" << std::endl;
        async_tcp_server server(io_service, listen_endpoint, msg_sub);

        server.addSentMsg("async server test0");
        server.addSentMsg("async server test1");
        server.addSentMsg("async server test2");
        server.addSentMsg("async server test3");
        server.addSentMsg("async server test4");
        server.addSentMsg("async server test5");
        server.addSentMsg("async server test6");

        std::cout << "[async_tcp_server_test_node]: run TCP server" << std::endl;
        io_service.run();
    }
    catch(std::exception& e)
    {
        std::cerr << "server exception: " << e.what() << std::endl;
    }

    return 0;
}

