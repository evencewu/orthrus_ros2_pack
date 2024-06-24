#pragma once

#include <inttypes.h>

#define CAN1 0
#define CAN2 1

#define USE_SSC
// #define USE_SOES

namespace orthrus_control
{
    /// @brief Standard can packet
    typedef struct can_pack
    {
        uint16_t StdId;
        uint8_t ExtId;
        // uint32_t ExtId;
        uint8_t IDE;
        uint8_t RTR;
        uint8_t DLC;

        uint8_t Data[8];
    } __attribute__((packed)) can_pack;

    typedef struct A1MotorRxPack
    {
        uint16_t mode;
        uint8_t temp;
        uint8_t error;
        float T;
        float W;
        float Pos;
        uint16_t Acc;
    } __attribute__((packed)) A1MotorRxPack;

    typedef struct A1MotorTxPack
    {
        uint16_t mode;
        float pos;
        float w;
        float kp;
        float kd;
        float t;
    } __attribute__((packed)) A1MotorTxPack;

    typedef struct Ecat_Outputs_Pack
    {
        uint8_t power;
        uint8_t null;
        struct can_pack can;

        struct A1MotorTxPack motor;

        uint16_t motor_id;


    } __attribute__((packed)) Ecat_Outputs_Pack;

    /// @brief ecat pdo recive data (slv to master)
    typedef struct Ecat_Inputs_Pack
    {
        uint8_t switch_io;
        uint8_t null[5];

        struct can_pack can;

        struct A1MotorRxPack motor;

        uint16_t motor_id;
        
    } __attribute__((packed)) Ecat_Inputs_Pack;
}
