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

    typedef struct A1MotorPack
    {
        uint8_t motor_id;
        uint8_t motor_mode;
        uint16_t motor_temp;
        uint16_t motor_error;
        uint32_t motor_T;
        uint32_t motor_W;
        uint32_t motor_Pos;
        uint32_t motor_LW;
        uint16_t motor_Acc;
    } __attribute__((packed)) A1MotorPack;

    typedef struct Ecat_Outputs_Pack
    {
        uint8_t LED;
        struct can_pack can[2];
        uint8_t null;
    } __attribute__((packed)) Ecat_Outputs_Pack;

    /// @brief ecat pdo recive data (slv to master)
    typedef struct Ecat_Inputs_Pack
    {
        uint8_t switch_io;
        uint8_t null[5];

        struct can_pack can[2];

        //struct A1MotorPack Motor;

    } __attribute__((packed)) Ecat_Inputs_Pack;

}
