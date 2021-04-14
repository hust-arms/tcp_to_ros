/*                                                                                             
 * Filename: uuv_status_msg_serialize.h
 * Path: tcp_to_ros
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <cstddef>

#include "byte_buffer.h"
#include "crc_check.h"

#pragma once

/**
 * @param UUV status message
 */
struct uuv_status_msg
{
    uint16_t head_ = 0xEBA1;
    uint16_t cnt_; // 0-65535
    uint8_t len_ = 79; // length of data segment

    uint8_t nav_mode_ = 0x00;
    uint16_t vis_x_;
    uint16_t vis_y_;
    uint16_t vis_z_;
    uint16_t vis_roll_;
    uint16_t vis_pitch_;
    uint16_t vis_yaw_;
    uint32_t usbl_x_;
    uint32_t usbl_y_;
    uint32_t usbl_z_;
    uint16_t usbl_roll_;
    uint16_t usbl_pitch_;
    uint16_t usbl_yaw_;

    uint16_t height_;
    uint16_t depth_;

    uint16_t yaw_;
    uint16_t pitch_;
    uint16_t roll_;
    uint16_t v_;
    uint16_t u_;
    uint16_t w_;

    uint32_t lat_;
    uint32_t lng_;

    uint16_t dot_v_;
    uint16_t dot_u_;
    uint16_t dot_w_;

    uint16_t upper_port_;
    uint16_t upper_starboard_;
    uint16_t lower_starboard_;
    uint16_t lower_port_;

    uint8_t command_;
    uint8_t light_;
    uint8_t lift_;

    uint16_t thruster_;

    uint16_t set1_ = 0xFFFF;
    uint16_t set2_ = 0xFFFF;
    uint16_t set3_ = 0xFFFF;
    uint16_t set4_ = 0xFFFF;
    uint16_t set5_ = 0xFFFF;

    uint8_t crc_ = 0;

    uint16_t tail_ = 0xEE1A;

    size_t total_len_ = 92;
}; 

/**
 * @brief UUV status message serialization
 */
class uuv_status_msg_serialize
{
public:
    /**
     * @brief constructor & message initalization
     */
    uuv_status_msg_serialize() 
    {
        msg_.cnt_ = 0;

        // visual module init (unused)
        msg_.vis_x_ = 0;
        msg_.vis_y_ = 0;
        msg_.vis_z_ = 0;
        msg_.vis_roll_ = 0;
        msg_.vis_pitch_ = 0;
        msg_.vis_yaw_ = 0;

        // acc (unused)
        msg_.dot_u_ = 0;
        msg_.dot_v_ = 0;
        msg_.dot_w_ = 0;

        // usbl module init (unused)
        // x, y, z  of usbl module are replaced as global x, y, z
        // msg_.usbl_x_ = 0;
        // msg_.usbl_y_ = 0;
        // msg_.usbl_z_ = 0;
        msg_.usbl_roll_ = 0;
        msg_.usbl_pitch_ = 0;
        msg_.usbl_yaw_ = 0;

        // GPS (unused)
        msg_.lat_ = 0.0;
        msg_.lng_ = 0.0;

        // other (unused)
        msg_.command_ = 0xFF;
        msg_.light_ = 0x00;
        msg_.lift_ = 0x00;
    }

    ~uuv_status_msg_serialize() 
    {}

private:
    /**
     * @brief UUV status message serialization
     */
    std::string serialize(const double& height, const double& depth, const double& x, const double& y, const double& z, 
                          const double& roll, const double& pitch, const double& yaw, const double& u, const double& v, const double& w, 
                          const double& up, const double& us, const double& ls, const double& lp, const double& thruster)
    {
        if(msg_.cnt_ > 65535)
            msg_.cnt_ = 0;
        else
            ++msg_.cnt_;

        msg_.height_ = uint16_t(height);
        msg_.depth_ = uint16_t(depth);

        msg_.usbl_x_ = uint32_t(x);
        msg_.usbl_y_ = uint32_t(y);
        msg_.usbl_z_ = uint32_t(z);

        msg_.roll_ = uint16_t(roll);
        msg_.pitch_ = uint16_t(pitch);
        msg_.yaw_ = uint16_t(yaw);
        
        msg_.u_ = uint16_t(u);
        msg_.v_ = uint16_t(v);
        msg_.w_ = uint16_t(w);

        msg_.upper_port_ = uint16_t(up);
        msg_.upper_starboard_ = uint16_t(us);
        msg_.lower_port_ = uint16_t(lp);
        msg_.lower_starboard_ = uint16_t(ls);
        msg_.thruster_ = uint16_t(thruster);

        byte_buffer buffer(msg_.total_len_);
        buffer.append_uint16(msg_.head_); // 0
        buffer.append_uint16(msg_.cnt_); // 2
        buffer.append_uint8(msg_.len_); // 4
        buffer.append_uint8(msg_.nav_mode_); // 5
        buffer.append_uint16(msg_.vis_x_); // 6
        buffer.append_uint16(msg_.vis_y_); // 8
        buffer.append_uint16(msg_.vis_z_); // 10
        buffer.append_uint16(msg_.vis_roll_); // 12
        buffer.append_uint16(msg_.vis_pitch_); // 14
        buffer.append_uint16(msg_.vis_yaw_); // 16
        buffer.append_uint32(msg_.usbl_x_); // 20
        buffer.append_uint32(msg_.usbl_y_); // 24
        buffer.append_uint32(msg_.usbl_z_); // 28
        buffer.append_uint16(msg_.usbl_roll_); // 30
        buffer.append_uint16(msg_.usbl_pitch_); // 32
        buffer.append_uint16(msg_.usbl_yaw_); // 34
        buffer.append_uint16(msg_.height_); // 36
        buffer.append_uint16(msg_.depth_); // 38
        buffer.append_uint16(msg_.yaw_); // 40
        buffer.append_uint16(msg_.pitch_); // 42
        buffer.append_uint16(msg_.roll_); // 44
        buffer.append_uint16(msg_.v_); // 46
        buffer.append_uint16(msg_.u_); // 48
        buffer.append_uint16(msg_.w_); // 50
        buffer.append_uint32(msg_.lat_); // 52
        buffer.append_uint32(msg_.lng_); // 56
        buffer.append_uint16(msg_.dot_v_); // 60
        buffer.append_uint16(msg_.dot_u_); // 62
        buffer.append_uint16(msg_.dot_w_); // 64
        buffer.append_uint16(msg_.upper_port_); // 66
        buffer.append_uint16(msg_.upper_starboard_); // 68
        buffer.append_uint16(msg_.lower_starboard_); // 70
        buffer.append_uint16(msg_.lower_port_); // 72
        buffer.append_uint8(msg_.command_); // 74
        buffer.append_uint8(msg_.light_); // 75
        buffer.append_uint8(msg_.lift_); // 76
        buffer.append_uint16(msg_.thruster_); // 77
        buffer.append_uint16(msg_.set1_); // 79
        buffer.append_uint16(msg_.set2_); // 81
        buffer.append_uint16(msg_.set3_); // 83
        buffer.append_uint16(msg_.set4_); // 85
        buffer.append_uint16(msg_.set5_); // 87

        // crc check
        msg_.crc_ = crc_calc(&buffer.getMsg()[5], msg_.len_); 

        buffer.append_uint8(msg_.crc_); // 89
        buffer.append_uint16(msg_.tail_); // 90

        size_t msg_len = buffer.getLen();
        std::string output(msg_len, 0);
        for(size_t i = 0; i < msg_len; ++i)
        {
            output[i] = static_cast<char>(buffer.getByte(i));
        }

        return output;
    }

private:
    uuv_status_msg msg_;
};

