/*                                                                          
 * Filename: tcp_ros_bridge.h
 * Path: tcp_ros_bridge
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once

#include "async_tcp_server.h"
#include "message.h"

#include <memory>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

#define DEBUG_

namespace tcp_ros_bridge{

/**
 * @brief transform messages from TCP client to ROS messages, use for UUV control
 */
class uuv_tcp_ros_bridge
{
    typedef boost::shared_ptr<async_tcp_server> server_ptr;
public:
    uuv_tcp_ros_bridge(const std::string& auv_name, boost::asio::io_service& io_service, int port);
    ~uuv_tcp_ros_bridge();

private:
    /**
     * @param data parse thread
     */
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
#ifdef DEBUG_
            std::cout << "[tcp_ros_bridge]: head of messsage: " << std::hex << *msg << std::endl;
#endif
            for(int i = 0; i < bytes; ++i)
            {
               // parse
               if(MESSAGE_FRAMING_OK == message_parse(*msg++))
               {
                   uint8_t mode, status_fd, vel, light, elevator;
                   float course, depth;
                   
                   // get control parameters from command
                   message_get_control_cmd(&mode, &status_fd, &vel, &course, &depth, 
                                           &ctrl_info_.fin0_, &ctrl_info_.fin1_, &ctrl_info_.fin2_, &ctrl_info_.fin3_, &light, &elevator);

#ifdef DEBUG_
                   std::cout << "[tcp_ros_bridge]: recv fin: " << ctrl_info_.fin0_ << " "
                       << ctrl_info_.fin1_ << " "
                       << ctrl_info_.fin2_ << " "
                       << ctrl_info_.fin3_ << " " << std::endl;;

                   std::cout << "[tcp_ros_bridge]: thruster: " << vel << std::endl;
#endif
               
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

    std::string auv_name_;
    int seq_;
    double period_;

    bool stopped_;
    int port_; boost::asio::io_service& io_service_;

    uuv_ctrl_sub_ptr tcp_msg_sub_;
    server_ptr server_;

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

}; // ns

