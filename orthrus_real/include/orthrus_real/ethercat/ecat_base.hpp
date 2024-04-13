#pragma once

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "soem_ros2/soem.h"
#include "orthrus_real/ethercat/TypeDef.hpp"

#define EC_VER1

namespace orthrus_real
{
    class EcatBase
    {
    public:
        EcatBase(int _slave_num);
        ~EcatBase();

        void EcatStart(char *ifname);
        void EcatSyncMsg();
        void EcatStop();

        Ecat_Outputs_Pack packet_tx[128];
        Ecat_Inputs_Pack packet_rx[128];

    private:
        int slave_num = 1;
        int pdo_output_byte = 74;//30
        int pdo_input_byte = 84;//34
        char IOmap[4096];
        volatile int wkc;
        int expectedWKC;
        int EC_TIMEOUTMON = 500;
    };
}

// #include "ecat_can_base/ecat_typedef.h"

