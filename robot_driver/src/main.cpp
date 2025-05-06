// main.cpp
#include "robot_driver_tx.h"
#include "robot_driver_rx.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto tx_node = std::make_shared<SocketCanSenderNode>("robot_driver_cantx");
  auto rx_node = std::make_shared<SocketCanReceiverNode>("robot_driver_canrx");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(tx_node);
  executor.add_node(rx_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}