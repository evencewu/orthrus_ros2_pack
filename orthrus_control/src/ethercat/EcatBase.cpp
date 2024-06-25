#include "orthrus_control/ethercat/EcatBase.hpp"

#include <unistd.h>

namespace orthrus_control
{
    EcatBase::EcatBase(int _slave_num)
    {
        this->slave_num = _slave_num;
        memset(&packet_tx, 0, sizeof(Ecat_Outputs_Pack));
        memset(&packet_rx, 0, sizeof(Ecat_Inputs_Pack));
    }

    EcatBase::~EcatBase()
    {
        // EcatStop();
    }

    /// @brief Start your ecat port
    /// @param ifname Network port name
    bool EcatBase::EcatStart(char *ifname)
    {
        RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"), "\033[32m Ethercat start %s \033[0m \n", ifname);

        int chk;
        if (ec_init(ifname))
        {
            // ec_DCtime = 1000;
            RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m ec_init on %s succeeded. \033[0m\n", ifname);
            if (ec_config_init(FALSE) > 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m %d slaves found and configured.\033[0m \n", ec_slavecount);

                ec_config_map(&IOmap);

                RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m Slaves mapped, state to SAFE_OP.\033[0m \n");
                /* wait for all slaves to reach SAFE_OP state */
                ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 6);

                ec_configdc();

                RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m Request operational state for all slaves\033[0m \n");

                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                ec_slave[0].state = EC_STATE_OPERATIONAL;
                /* send one valid process data to make outputs in slaves happy*/

                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                /* request OP state for all slaves */
                ec_writestate(0);

                chk = 200;
                /* wait for all slaves to reach OP state */
                do
                {
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

                return true;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m No slaves found!\033[0m \n");
                return false;
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m No socket connection on %s\nExcecute as root\033[0m \n", ifname);
            return false;
        }
    }

    /// @brief Data sending and receiving, calling in a loop
    /// @param output_data Pointer to the output data
    /// @param input_data  Pointer to the input data
    bool EcatBase::EcatSyncMsg()
    {
        bool state_flag = true;
        for (int slave = 1; slave <= slave_num; slave++)
        {
            if (ec_slave[slave].state != EC_STATE_OPERATIONAL)
            {
                state_flag = false;
            }
        }

        if (state_flag == true)
        {
            for (int slave = 1; slave <= slave_num; slave++)
            {
                memcpy(ec_slave[slave].outputs, &packet_tx[slave - 1], pdo_output_byte);
            }

            ec_send_processdata();

            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            if (wkc >= expectedWKC)
            {
                for (int slave = 1; slave <= slave_num; slave++)
                {
                    memcpy(&packet_rx[slave - 1], ec_slave[slave].inputs, pdo_input_byte);
                }
            }
            return true;
        }
        else
        {
            int i = 0;
            RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m Not all slaves reached operational state.\033[0m \n");
            ec_readstate();
            for (i = 1; i <= ec_slavecount; i++)
            {
                if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                {
                    RCLCPP_INFO(rclcpp::get_logger("OrthrusEthercat"),"\033[32m Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\033[0m \n",
                           i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                }
            }
            return false;
        }
        osal_usleep(750);
    }

    /// @brief Stop the ecat connection and perform a secure stop before that
    void EcatBase::EcatStop()
    {
        //printf("\nRequest init state for all slaves\n");
        ec_slave[0].state = EC_STATE_INIT;
        /* request INIT state for all slaves */
        ec_writestate(0);
    }
}

