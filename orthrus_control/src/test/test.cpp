#include "orthrus_control/ethercat/EcatBase.hpp"
#include "orthrus_control/ethercat/TypeDef.hpp"
#include "orthrus_control/assembly/Leg.hpp"
#include "orthrus_control/assembly/Imu.hpp"

#define theta1 18.82f
#define theta2 14.6f

orthrus_control::EcatBase Ethercat = EcatBase(2);
// hardware interface
orthrus_control::Leg leg[4];
orthrus_control::Imu body_imu;

bool ethercat_prepare_flag_ = false;

int main()
{
    char phy[] = "enp2s0";
    ecat_flag = Ethercat.EcatStart(phy);

    Ethercat.EcatSyncMsg();
}