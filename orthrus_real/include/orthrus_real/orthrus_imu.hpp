#pragma once

#include <inttypes.h>

#include <Eigen/Dense>  
#include <Eigen/Geometry>  

#include "orthrus_real/ethercat/ecat_typedef.hpp"

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
        void init(uint8_t can_id, uint8_t device_id);
        void analyze(Ecat_Inputs_Pack *pack);

        double yaw;
        double pitch;
        double roll;

        Eigen::Quaterniond gyro_;

        Eigen::Quaterniond unified_gyro_;
    private:
        union
        {
            float f_data;
            uint8_t uint_data[4];
        }data;
        
        uint8_t can_id;
        uint8_t device_id;
    };
}