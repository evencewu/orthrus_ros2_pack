#pragma once

#include <inttypes.h>

#define CAN1 0
#define CAN2 1

#define USE_SSC
// #define USE_SOES

namespace orthrus_real
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
        uint8_t id;
        uint8_t mode;
        uint16_t temp;
        uint16_t error;
        uint32_t T;
        uint32_t W;
        uint32_t Pos;
        uint32_t LW;
        uint16_t Acc;
    } __attribute__((packed)) A1MotorRxPack;

    typedef struct A1MotorTxPack
    {
        uint16_t mode;
        uint32_t pos;
        uint32_t w;
        uint32_t kp;
        uint32_t kd;
        uint32_t t;
    } __attribute__((packed)) A1MotorTxPack;

    typedef struct Ecat_Outputs_Pack
    {
        uint8_t LED;
        struct can_pack can[2];
        uint8_t null;

        struct A1MotorTxPack motor[4][3];
    } __attribute__((packed)) Ecat_Outputs_Pack;

    /// @brief ecat pdo recive data (slv to master)
    typedef struct Ecat_Inputs_Pack
    {
        uint8_t switch_io;
        uint8_t null[5];

        struct can_pack can[2];

        struct A1MotorRxPack motor[4][3];

    } __attribute__((packed)) Ecat_Inputs_Pack;
}
