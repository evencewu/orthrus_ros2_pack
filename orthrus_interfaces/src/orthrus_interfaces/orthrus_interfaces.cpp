#include "orthrus_interfaces/orthrus_interfaces.hpp"

namespace orthrus_interfaces
{
    OrthrusInterfacesNode::OrthrusInterfacesNode() : Node("orthrus_interfaces")
    {

    }

    ~OrthrusInterfacesNode::OrthrusInterfacesNode()
    {
        safe_stop();
        Ethercat.EcatStop();
    }
void OrthrusInterfacesNode::init()
  {
    char phy[] = "enp5s0";
  
    RCLCPP_INFO(this->get_logger(), "wl_driver启动,网口%s\n", phy);
    Ethercat.EcatStart(phy);

  }

  /// @brief arx5a 主循环函数
  void OrthrusInterfacesNode::main_loop()
  {
    //TODO
    //s/r message
    Ethercat.EcatSyncMsg();
    analyze_all();

    geometry_msgs::msg::TransformStamped t;
  }

  void OrthrusInterfacesNode::safe_stop()
  {
    //TODO 
    //safe stop code
  }

  void OrthrusInterfacesNode::analyze_all()
  {

  }

  void OrthrusInterfacesNode::usleep(int usec)
  {
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000;
    nanosleep(&ts, NULL);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orthrus_interfaces::OrthrusInterfacesNode>());
  rclcpp::shutdown();
  return 0;
}