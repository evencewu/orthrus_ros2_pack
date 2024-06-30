#include "orthrus_control/OrthrusInterface.hpp"

namespace orthrus_control
{
    void OrthrusSystemHardware::SafeStop()
    {
        Ethercat.packet_tx[0].power = 0x00;
        Ethercat.packet_tx[1].power = 0x00;

        for (int i = 0; i <= 20; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (j > 5)
                {
                    leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[1], 0, 0, 0, 0, 0, 0);
                    Ethercat.EcatSyncMsg();
                }
                else
                {
                    leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, 0, 0, 0);
                    Ethercat.EcatSyncMsg();
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "motor stop!");
    }
}