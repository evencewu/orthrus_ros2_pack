#include "orthrus_real/ecat_base.hpp"
#include "orthrus_real/ecat_typedef.hpp"
#include "orthrus_real/orthrus_leg.hpp"

#include <array>
#include <signal.h>
#include <unistd.h>

bool app_stopped = false;

void sigint_handler(int sig);
void safe_stop();

orthrus_real::EcatBase Ethercat(1);

orthrus_real::Leg leg[4];
orthrus_real::Imu body_imu;

int main()
{
    signal(SIGINT, sigint_handler);

    char phy[] = "enp3s0";
    Ethercat.EcatStart(phy);
    
    leg[0].init(CAN2,IMU1,USART1);
    leg[1].init(CAN2,IMU2,USART2);
    leg[2].init(CAN2,IMU3,USART3);
    leg[3].init(CAN2,IMU4,USART6);
    body_imu.init(CAN2,IMU5);

    printf("start\n");

    for (int i = 0; i < 100000; i++)
    {

        Ethercat.EcatSyncMsg();

        if (app_stopped)
        {
            break;
        }
    }

    safe_stop();
    Ethercat.EcatStop();

    printf("stop");

    return 0;
}

void sigint_handler(int sig)
{
    if (sig == SIGINT)
    {
        app_stopped = true;
    }
}

void safe_stop()
{
    Ethercat.packet_tx[0].LED = 0;
    Ethercat.EcatSyncMsg();
    printf("stop motor!");
}