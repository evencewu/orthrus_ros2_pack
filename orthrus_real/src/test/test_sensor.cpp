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

orthrus_real::Leg leg = orthrus_real::Leg(CAN2,IMU4,0);

int main()
{
    signal(SIGINT, sigint_handler);

    char phy[] = "enp3s0";
    Ethercat.EcatStart(phy);

    printf("start\n");

    for (int i = 0; i < 100000; i++)
    {
            
        Ethercat.packet_tx[0].LED = (i / 1000) % 8;

        Ethercat.EcatSyncMsg();

        leg.analyze(&Ethercat.packet_rx[0]);

        printf("%f %f %f %f\n",leg.imu.Gyro[0],leg.imu.Gyro[1],leg.imu.Gyro[2],leg.angle.Pos);

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