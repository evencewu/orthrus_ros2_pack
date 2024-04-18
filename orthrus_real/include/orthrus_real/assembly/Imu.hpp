#pragma once

#include <inttypes.h>

#include <Eigen/Dense>  
#include <Eigen/Geometry>  

#include "orthrus_real/ethercat/TypeDef.hpp"

#define IMU1 0
#define IMU2 3
#define IMU3 6
#define IMU4 9
#define IMU5 12

namespace orthrus_real
{
    class Imu
    {
    public:
        void Init(uint8_t can_id, uint8_t device_id);
        void Analyze(Ecat_Inputs_Pack *pack);
        void get_angle(double standard_yaw);
        static void IfUseMag(bool flag, can_pack can);
        
        Eigen::Quaterniond gyro_;

        Eigen::Quaterniond unified_gyro_;

        Eigen::Quaterniond standard_gyro_;

        double yaw;
        double pitch;
        double roll;

    private:
        union
        {
            float f_data;
            uint8_t uint_data[4];
        }data;
        
        uint8_t can_id_;
        uint8_t device_id_;
    };
}