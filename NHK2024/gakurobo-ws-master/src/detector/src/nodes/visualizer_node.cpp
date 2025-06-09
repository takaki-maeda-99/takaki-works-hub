#include <rclcpp/rclcpp.hpp>

#include "detector/visualizer.h"

int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Visualizer>());
   rclcpp::shutdown();
   return 0;
}
