/*                                                                          
 * Filename: tcp_ros_bridge.h
 * Path: tcp_ros_bridge
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once

#define LATEST_MSG

#ifdef LATEST_MSG
#include "message_latest.hpp"
#else
#include "message.h"
#endif


#include <memory>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

using boost::asio::ip::tcp;

#define DEBUG_

namespace tcp_ros_bridge{

//----------------------------------------------------------------------

class subscriber
{
public:
  virtual ~subscriber() {}
  virtual void deliver(uint8_t* msg, size_t bytes) = 0;
};

typedef boost::shared_ptr<subscriber> subscriber_ptr;

//----------------------------------------------------------------------

class channel
{
public:
  void join(subscriber_ptr subscriber)
  {
    subscribers_.insert(subscriber);
  }

  void leave(subscriber_ptr subscriber)
  {
    subscribers_.erase(subscriber);
  }

  void deliver(uint8_t* msg, size_t bytes)
  {
    std::for_each(subscribers_.begin(), subscribers_.end(),
        boost::bind(&subscriber::deliver, _1, boost::ref(msg), bytes));
  }

private:
  std::set<subscriber_ptr> subscribers_;
};

//----------------------------------------------------------------------    

const int BUFFER_MAX_LEN = 1024;

class server;

/**
 * @brief transform messages from TCP client to ROS messages, use for UUV control
 */
class tcp_ros_bridge : 
    public subscriber, 
    public boost::enable_shared_from_this<tcp_ros_bridge>
{
    friend class server;
public:
    tcp_ros_bridge(const std::string& auv_name, boost::asio::io_service& io_service, channel& ch);
    ~tcp_ros_bridge();

private:
    void start()
    {
      channel_.join(shared_from_this());
    
      std::cout << "[tcp_ros_bridge]: start read&write thread" << std::endl;
      parse_th_ = new boost::thread(boost::bind(&tcp_ros_bridge::uuv_ctrl_publish_thread, this));
      send_th_ = new boost::thread(boost::bind(&tcp_ros_bridge::uuv_status_send_thread, this));
    }

    tcp::socket& socket()
    {
      return socket_;
    }

    void stop()
    {
      channel_.leave(shared_from_this());
  
      socket_.close();
  
      std::cout << "[tcp_ros_bridge]: close server" << std::endl;
    }

    bool stopped() const
    {
      return !socket_.is_open();
    }

    /*
     * @brief Parse UUV control message
     */
    void deliver(uint8_t* msg, size_t bytes) override
    {
      // std::cout << "[tcp_ros_bridge]: head of messsage: " << std::hex << *msg << std::endl;
      // for(int i = 0; i < bytes; ++i)
      // {
      //    // parse
      //    if(MESSAGE_FRAMING_OK == message_parse(*msg++))
      //    {
      //        uint8_t mode, status_fd, vel, light, elevator;
      //        float course, depth;
      //        
      //        // get control parameters from command
      //        message_get_control_cmd(&mode, &status_fd, &vel, &course, &depth, 
      //                                &ctrl_info_.fin0_, &ctrl_info_.fin1_, &ctrl_info_.fin2_, &ctrl_info_.fin3_, &light, &elevator);
      //
      //        std::cout << "[tcp_ros_bridge]: recv fin: " << ctrl_info_.fin0_ << " "
      //            << ctrl_info_.fin1_ << " "
      //            << ctrl_info_.fin2_ << " "
      //            << ctrl_info_.fin3_ << " " << std::endl;;
      //
      //        std::cout << "[tcp_ros_bridge]: thruster: " << vel << std::endl;
      //    
      //        ctrl_info_.rpm_ = static_cast<int>(vel) * 0.01 * 2300.0;
      //    }
      //    // else{
      //    //    std::cout << "[tcp_ros_bridge]: parse message failed!" << std::endl;
      //    // }
      //}
    }

    void uuv_ctrl_publish_thread();

    /**
     * @param data send thread
     */
    void uuv_status_send_thread();

    /**
     * @param server manage thread
     */
    void server_manage_thread();

    /**
     * @param server check thread
     */
    void server_check_thread();

    /**
     * @param get control information from subscriber
     */
    void getCtrlInfo()
    {
        if(tcp_msg_sub_)
            ctrl_info_ = tcp_msg_sub_->getCtrlInfo();
    }

    /* Sensors callback func */
    /**
     * @brief IMU data input
     */
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief Pressure sensor data input
     */
    void pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg);

    /**
     * @brief Position & pose input  
     */ 
    void posegtCb(const nav_msgs::Odometry::ConstPtr& msg); 
    
    /**
     * @brief DVL input  
     */ 
    void dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg); 

    /**
     * @brief Upper port 
     */
    void fin0Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief Upper starboard 
     */
    void fin1Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);
    
    /**
     * @brief Lower port 
     */
    void fin2Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);
    
    /**
     * @brief lower starboard 
     */
    void fin3Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief thruster 
     */
    void thruster0Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

private:
    struct uuv_ctrl_info
    {
        uuv_ctrl_info()
        {
            fin0_ = 0.0; fin1_ = 0.0; fin2_ = 0.0; fin3_ = 0.0;
            rpm_ = 0.0;
        }

        uuv_ctrl_info& operator=(const uuv_ctrl_info& ctrl_info)
        {
            if(this != &ctrl_info)
            {
                this->fin0_ = ctrl_info.fin0_;
                this->fin1_ = ctrl_info.fin1_;
                this->fin2_ = ctrl_info.fin2_;
                this->fin3_ = ctrl_info.fin3_;
                this->rpm_ = ctrl_info.rpm_;

                return *this;
            }
        }

        float fin0_;
        float fin1_;
        float fin2_;
        float fin3_;

        float rpm_;
    }; // uuv_ctrl_info

    struct uuv_status_info
    {
        double x_, y_, z_, u_, v_, w_;
        double depth_;
        double roll_, pitch_, yaw_, droll_, dpitch_, dyaw_;
        double fin0_, fin1_, fin2_, fin3_; // up, us, lp, ls
        double rpm_;

        uuv_status_info()
        {
            x_ = 0.0; y_ = 0.0; z_ = 0.0; u_ = 0.0; v_ = 0.0; w_ = 0.0;
            depth_ = 0.0;
            roll_ = 0.0; pitch_ = 0.0; yaw_ = 0.0; droll_ = 0.0; dpitch_ = 0.0; dyaw_ = 0.0;
            fin0_ = 0.0; fin1_ = 0.0; fin2_ = 0.0; fin3_ = 0.0;
            rpm_ = 0.0;
        }
    }; // uuv_status_info

    /**
     * @brief TCP server subscriber for UUV control message
     */
    class uuv_ctrl_subscriber : public subscriber
    {
    public:
        /**
         * @brief Return control information
         */
        uuv_ctrl_info getCtrlInfo()const
        {
            return ctrl_info_;
        }
    
        /**
         * @brief Parse UUV control message
         */
        void deliver(uint8_t* msg, size_t bytes) override
        {
            std::cout << "[tcp_ros_bridge]: head of messsage: " << std::hex << *msg << std::endl;
            for(int i = 0; i < bytes; ++i)
            {
               // parse
               if(MESSAGE_FRAMING_OK == message_parse(*msg++))
               {
                   uint8_t mode, status_fd, vel, light, elevator;
                   float course, depth;
                   uint16_t propeller1, propeller2, propeller3, propeller4; // latest
                   uint16_t sensor_switch, load_reject, release_ins; // latest
                   
                   // get control parameters from command
#ifdef LATEST_MSG
                   message_get_control_cmd(&mode, &status_fd, &vel, &course, &depth,
                                           &propeller1, &propeller2, &propeller3, &propeller4, 
                                           &light, &elevator, &ctrl_info_.fin0_, &ctrl_info_.fin1_, &ctrl_info_.fin2_, &ctrl_info_.fin3,
                                           &sensor_switch, &load_reject, &release_ins);
#else
                   message_get_control_cmd(&mode, &status_fd, &vel, &course, &depth, 
                                           &ctrl_info_.fin0_, &ctrl_info_.fin1_, &ctrl_info_.fin2_, &ctrl_info_.fin3_, &light, &elevator);
#endif
    
                   ctrl_info_.fin0_ = ctrl_info_.fin0_ / 57.3;
                   ctrl_info_.fin1_ = ctrl_info_.fin1_ / 57.3;
                   ctrl_info_.fin2_ = ctrl_info_.fin2_ / 57.3;
                   ctrl_info_.fin3_ = ctrl_info_.fin3_ / 57.3;

                   std::cout << "[tcp_ros_bridge]: recv fin: " << ctrl_info_.fin0_ << " "
                       << ctrl_info_.fin1_ << " "
                       << ctrl_info_.fin2_ << " "
                       << ctrl_info_.fin3_ << " " << std::endl;;
    
                   std::cout << "[tcp_ros_bridge]: thruster: " << vel << std::endl;
               
                   ctrl_info_.rpm_ = static_cast<int>(vel) * 0.01 * 2300.0;
               }
               // else{
               //    std::cout << "[tcp_ros_bridge]: parse message failed!" << std::endl;
               // }
           }
        }
    
    private:
        uuv_ctrl_info ctrl_info_;
    }; // uuv_ctrl_subscriber 

    typedef boost::shared_ptr<uuv_ctrl_subscriber> uuv_ctrl_sub_ptr;

    tcp::socket socket_;
    uint8_t buffer_[BUFFER_MAX_LEN];
    channel& channel_;
    // tcp::acceptor acceptor_;
    // uuv_ctrl_sub_ptr tcp_msg_sub_;

    std::string auv_name_;
    int seq_;
    double period_;

    bool stopped_;
    int port_; 
    boost::asio::io_service& io_service_;

    uuv_ctrl_sub_ptr tcp_msg_sub_;

    boost::thread* parse_th_;
    boost::thread* send_th_;
    boost::thread* server_manage_th_;
    boost::thread* server_check_th_;

    boost::recursive_mutex uuv_ctrl_info_mutex_;
    uuv_ctrl_info ctrl_info_;

    boost::recursive_mutex uuv_status_info_mutex_;
    uuv_status_info status_info_; 

    ros::Publisher fins0_pub_; // upper port
    ros::Publisher fins1_pub_; // upper starboard
    ros::Publisher fins2_pub_; // lower port
    ros::Publisher fins3_pub_; // lower starboard
    ros::Publisher thruster0_pub_;
    
    ros::Subscriber imu_sub_, pressure_sub_, posegt_sub_, dvl_sub_;
    ros::Subscriber fins0_sub_, fins1_sub_, fins2_sub_, fins3_sub_;
    ros::Subscriber thruster0_sub_;
}; // uuv_ros_to_tcp

// ----------------------------------------------------------------------

typedef boost::shared_ptr<tcp_ros_bridge> tcp_ros_bridge_ptr;

class server
{
public:
    server(const std::string& auv_name, boost::asio::io_service& io_service,
         const tcp::endpoint& listen_endpoint)
       : io_service_(io_service),
         acceptor_(io_service, listen_endpoint),
         auv_name_(auv_name)
    {
        acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));

        std::cout << "[server]: Server initialize" << std::endl;
        
        start_accept();
    }

    void start_accept()
    {
      std::cout << "[server]: listen" << std::endl;
      // tcp_session_ptr new_session(new tcp_session(io_service_, channel_));
      session_ = boost::shared_ptr<tcp_ros_bridge>(new tcp_ros_bridge(auv_name_, io_service_, channel_));
    
      acceptor_.async_accept(session_->socket(),
          boost::bind(&server::handle_accept, this, session_,
                      boost::asio::placeholders::error));
    }

    void handle_accept(tcp_ros_bridge_ptr session,
        const boost::system::error_code& ec)
    {
      if (!ec)
      {
        std::cout << "[server]: start tcp session" << std::endl;
        session->start();
    
        start_accept();
      }
      else
      {
          std::cerr << "[server]: error in handle acceptance" << std::endl;
      }
    }

    void stop_server()
    {
        session_->stop();
    }

private:
    boost::asio::io_service& io_service_;
    tcp::acceptor acceptor_;
    channel channel_;
    std::string auv_name_;

    tcp_ros_bridge_ptr session_;

}; // server
}; // ns

