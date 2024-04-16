#include "orthrus_real/ethercat/ecat_base.hpp"
#include "orthrus_real/ethercat/TypeDef.hpp"
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

/// @brief 静止校准
/// @param pack 
/// @param can_port 
void ImuCalibrateStart(orthrus_real::Ecat_Outputs_Pack *pack, int can_port,int calibrate_flag)
{
    pack->can.StdId = 0x11;
    pack->can.DLC = 0x04;
    pack->can.Data[calibrate_flag] = 1;
    //pack->can[can_port].Data[0] = 0;//磁力计校准
    //pack->can[can_port].Data[1] = 0;//pos零点
    //pack->can[can_port].Data[2] = 1;//静止校准
    //pack->can[can_port].Data[3] = 0;//enable磁力计
}

void ImuCalibrateStop(orthrus_real::Ecat_Outputs_Pack *pack, int can_port)
{
    pack->can.StdId = 0x11;
    pack->can.DLC = 0x04;
    pack->can.Data[0] = 0;
    pack->can.Data[1] = 0;
    pack->can.Data[2] = 0;
    pack->can.Data[3] = 0;
}

int calibrate_flag = 2;

int main()
{
    signal(SIGINT, sigint_handler);

    char phy[] = "enp5s0";
    Ethercat.EcatStart(phy);

    leg[0].Init(CAN2, IMU1, USART1);
    leg[1].Init(CAN2, IMU2, USART2);
    leg[2].Init(CAN2, IMU3, USART3);
    leg[3].Init(CAN2, IMU4, USART6);
    body_imu.Init(CAN2, IMU5);
    for (int i = 0; i < 100000; i++)
    {
        //ImuCalibrateStart(&Ethercat.packet_tx[0], CAN2);
        ImuCalibrateStart(&Ethercat.packet_tx[0], CAN2,calibrate_flag);
        //ImuCalibrateStart_1(&Ethercat.packet_tx[0], CAN2);
        Ethercat.EcatSyncMsg();
        //printf("loop\n");
        //printf("imu: %f %f %f %f\n",body_imu.Gyro[0], body_imu.Gyro[1], body_imu.Gyro[2], body_imu.Gyro[3]);

        if (app_stopped)
        {
            break;
        }
    }

    safe_stop();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();
    ImuCalibrateStop(&Ethercat.packet_tx[0], CAN2);
    Ethercat.EcatSyncMsg();


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