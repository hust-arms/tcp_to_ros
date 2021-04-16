/*                                                                           
 * Filename: tcp_ros_bridge.cpp
 * Path: tcp_ros_bridge
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */


#include "tcp_ros_bridge/tcp_ros_bridge.h"

namespace tcp_ros_bridge
{

//////////////////////////////////
uuv_tcp_ros_bridge::uuv_tcp_ros_bridge(const std::string& auv_name, 
                               boost::asio::io_service& io_service, int port) : auv_name_(auv_name)
{
    // initialize ROS components
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("period", period_, 0.2);

    thruster0_pub_= nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name + "/thrusters/0/input", 1);
    fins0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name + "/fins/0/input", 1);
    fins1_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name + "/fins/1/input", 1);
    fins2_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name + "/fins/2/input", 1);
    fins3_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name + "/fins/3/input", 1);

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(auv_name + "/imu", 1, boost::bind(&uuv_tcp_ros_bridge::imuCb, this, _1));
    pressure_sub_ = nh.subscribe<sensor_msgs::FluidPressure>(auv_name + "/pressure", 1, boost::bind(&uuv_tcp_ros_bridge::pressureCb, this, _1));
    posegt_sub_ = nh.subscribe<nav_msgs::Odometry>(auv_name + "/pose_gt", 1, boost::bind(&uuv_tcp_ros_bridge::posegtCb, this, _1));
    dvl_sub_ = nh.subscribe<uuv_sensor_ros_plugins_msgs::DVL>(auv_name + "/dvl", 1, boost::bind(&uuv_tcp_ros_bridge::dvlCb, this, _1));


    // try create async server
    seq_ = 0;
    
    try
    {
        tcp_msg_sub_ = boost::shared_ptr<subscriber>(new uuv_ctrl_subscriber());
        tcp::endpoint listen_endpoint(tcp::v4(), port);

        server_ = boost::shared_ptr<async_tcp_server>(new async_tcp_server(io_service, listen_endpoint, tcp_msg_sub_));
    }
    catch(std::exception& e)
    {
        std::cerr << "[uuv_tcp_ros_bridge]: Error in server creating" << std::endl;
        exit(1);
    }

    parse_th_ = new boost::thread(boost::bind(&uuv_tcp_ros_bridge::uuv_ctrl_publish_thread, this));
}

//////////////////////////////////
uuv_tcp_ros_bridge::~uuv_tcp_ros_bridge()
{
    if(parse_th_ != nullptr)
    {
        parse_th_->interrupt();
        parse_th_->join();
        delete parse_th_;
        parse_th_ = nullptr;
    }
}

//////////////////////////////////
void uuv_tcp_ros_bridge::uuv_ctrl_publish_thread()
{
    ros::NodeHandle nh;

    while(nh.ok())
    {
        double fins0, fins1, fins2, fins3, rpm;

        {
            boost::unique_lock<boost::recursive_mutex> lock(uuv_ctrl_info_mutex_);
            
            fins0 = ctrl_info_.fin0_;
            fins1 = ctrl_info_.fin1_;
            fins2 = ctrl_info_.fin2_;
            fins3 = ctrl_info_.fin3_;

            rpm = ctrl_info_.rpm_;
        }

        // publish control params
        // thruster
        std_msgs::Header header;
        header.stamp.setNow(ros::Time::now());
        header.frame_id = auv_name_ + "/base_link";
        header.seq = ++seq_;

        uuv_gazebo_ros_plugins_msgs::FloatStamped thrusters_msg;
        thrusters_msg.header = header;
        thrusters_msg.data = rpm;
        thruster0_pub_.publish(thrusters_msg);

        // fins
        uuv_gazebo_ros_plugins_msgs::FloatStamped fins_msg;
        fins_msg.header = header;
        // upper port
        fins_msg.data = fins0;
        fins0_pub_.publish(fins_msg);
        // upper starboard
        fins_msg.data = fins1;
        fins1_pub_.publish(fins_msg);
        // lower port
        fins_msg.data = fins2;
        fins2_pub_.publish(fins_msg);
        // lower starboard
        fins_msg.data = fins3;
        fins3_pub_.publish(fins_msg);

        boost::this_thread::sleep(boost::posix_time::milliseconds(period_ * 1000));
    }
}

//////////////////////////////////
void uuv_tcp_ros_bridge::uuv_status_send_thread()
{
    ros::NodeHandle nh;

    while(nh.ok())
    {
        double x, y, z, u, v, w;
        double roll, pitch, yaw, droll, dpitch, dyaw;
        double fin0, fin1, fin2, fin3, rpm;

        {
            boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
            x = status_info_.x_;
            y = status_info_.y_;
            z = status_info_.z_;
            u = status_info_.u_;
            v = status_info_.v_;
            w = status_info_.w_;
            roll = status_info_.roll_;
            pitch = status_info_.pitch_;
            yaw = status_info_.yaw_;
            droll = status_info_.droll_;
            dpitch = status_info_.dpitch_;
            dyaw = status_info_.dyaw_;
            fin0 = status_info_.fin0_;
            fin1 = status_info_.fin1_;
            fin2 = status_info_.fin2_;
            fin3 = status_info_.fin3_;
            rpm = status_info_.rpm_;
        }

        // serialize status params and send
        
        server_->addSentMsg("");

        boost::this_thread::sleep(boost::posix_time::milliseconds(period_ * 1000));
    }
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::imuCb(const sensor_msgs::Imu::ConstPtr& msg) 
{
    // PoseStamped::Quaternion to tf::Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    // Quaternion to RPY
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    tf::Matrix3x3(quat).getRPY(status_info_.roll_, status_info_.pitch_, status_info_.yaw_);                                          

    status_info_.droll_ = msg->angular_velocity.x;
    status_info_.dpitch_ =  msg->angular_velocity.y;
    status_info_.dyaw_ = msg->angular_velocity.z;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg) // For pressure sensor
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.depth_ = static_cast<double>((msg->fluid_pressure - 101) / 10.1) - 0.25;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::posegtCb(const nav_msgs::Odometry::ConstPtr& msg) // For pose sensor
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.x_ = msg->pose.pose.position.x;
    status_info_.y_ = msg->pose.pose.position.y;
    status_info_.z_ = msg->pose.pose.position.z;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) // For DVL
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.u_ = msg->velocity.x;
    status_info_.v_ = msg->velocity.y;
    status_info_.w_ = msg->velocity.z;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::fin0Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.fin0_ = msg->data;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::fin1Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.fin1_ = msg->data;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::fin2Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.fin2_ = msg->data;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::fin3Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.fin3_ = msg->data;
}

/////////////////////////////////////
void uuv_tcp_ros_bridge::thruster0Cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(uuv_status_info_mutex_);
    status_info_.rpm_ = msg->data;
}

}; // ns
